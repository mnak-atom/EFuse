/**
 * @file comm.c
 * @brief Communication module implementation for framed UART protocol
 *
 * This module handles bidirectional UART communication with frame-based protocol:
 * Frame format: 0xAA | Type | Len | Payload | CRC(lo,hi) | 0xBB
 * CRC covers Type+Len+Payload using CRC-16/CCITT-FALSE
 *
 * @note RX has priority over TX to prevent data loss
 */

/*============================================================================*/
/*                             Includes                                       */
/*============================================================================*/
/* Standard includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* STM32 HAL includes */
#include "main.h"

/* Project includes */
#include "comm.h"
#include "eeprom.h"

/*============================================================================*/
/*                             External Dependencies                          */
/*============================================================================*/

/* HAL UART handle (USART2 assumed) */
extern UART_HandleTypeDef huart2;

/*============================================================================*/
/*                             Type Definitions                               */
/*============================================================================*/

/** RX State Machine States */
typedef enum
{
    RX_WAIT_START = 0, /**< Waiting for start byte 0xAA */
    RX_TYPE,           /**< Reading message type byte */
    RX_LEN,            /**< Reading payload length byte */
    RX_PAYLOAD,        /**< Reading payload bytes */
    RX_CRC_LO,         /**< Reading CRC low byte */
    RX_CRC_HI,         /**< Reading CRC high byte */
    RX_END             /**< Reading end byte 0xBB */
} comm_rx_state_t;

/*============================================================================*/
/*                             Debug Configuration                            */
/*============================================================================*/

#define COMM_DBG_BYTE_QSZ 128u
#define COMM_RX_DEBUG_QSZ 128u
#define COMM_DBGF_MAX_PAY 64u
#define COMM_DBGF_QDEPTH 12u

typedef struct
{
    uint8_t type;
    uint8_t len;
    uint8_t crc_ok;
    uint8_t pay[COMM_DBGF_MAX_PAY];
} comm_dbgf_t;

/*============================================================================*/
/*                             Private Variables                              */
/*============================================================================*/

/* Command Mailboxes */
static volatile efuse_comm_command_t s_cmd_mailbox = EFUSE_COMMAND_NONE;
static volatile uint8_t s_prot_update_pending = 0u;
static comm_prot_limits_t s_prot_update_buf;

/* TX State Variables */
#define TX_BUFFER_SIZE 256u

static uint8_t s_tx_buffer[TX_BUFFER_SIZE];
static volatile uint16_t s_tx_write = 0;
static volatile uint16_t s_tx_read = 0;
static volatile uint8_t s_tx_busy = 0;
static volatile uint8_t s_tx_pending = 0;

/* RX State Variables */
#define COMM_RX_PAYLOAD_MAX 256u

static volatile comm_rx_state_t s_rx_state = RX_WAIT_START;
static uint8_t s_rx_byte = 0u;
static uint8_t s_rx_type = 0u;
static uint8_t s_rx_len = 0u;
static uint8_t s_rx_idx = 0u;
static uint8_t s_rx_payload[COMM_RX_PAYLOAD_MAX];
static uint16_t s_crc_calc = 0u;
static uint16_t s_crc_recv = 0u;
static volatile uint8_t s_rx_pending = 0;

/* RX Frame Queue */
static volatile uint8_t s_have_frame = 0u;
static uint8_t s_q_type = 0u;
static uint8_t s_q_len = 0u;
static uint8_t s_q_payload[COMM_RX_PAYLOAD_MAX];

/* Debug Variables */
static volatile comm_dbgf_t s_dbgf_q[COMM_DBGF_QDEPTH];
static volatile uint8_t s_dbgf_w = 0u;
static volatile uint8_t s_dbgf_r = 0u;
static volatile uint8_t s_dbgb_q[COMM_DBG_BYTE_QSZ];
static volatile uint16_t s_dbgb_w = 0u;
static volatile uint16_t s_dbgb_r = 0u;
static volatile uint32_t s_dbg_irq_count = 0u;

/* UART test enable */
static volatile uint8_t s_test_enabled = 0u;

/*============================================================================*/
/*                         Private Function Prototypes                        */
/*============================================================================*/

static void rx_reset_to_start(void);
static uint16_t crc16_ccitt_update(uint16_t crc_state, uint8_t data_byte);
static uint16_t crc16_ccitt_update_buf(uint8_t message_type, const uint8_t *payload_ptr, uint8_t payload_len);
static inline int32_t read_i32_le(const uint8_t *buf, uint32_t offset);
static inline void write_u32_le(uint8_t *dest_ptr, uint32_t value);
static inline void Comm_Debug_ByteEnq(uint8_t b);
static inline void Comm_Debug_FrameEnq(uint8_t type, uint8_t len, uint8_t crc_ok, const uint8_t *pay);

/*============================================================================*/
/*                              RX Implementation                             */
/*============================================================================*/

/**
 * @brief Initialize and start UART reception
 */
void Comm_BeginRx(void)
{
    rx_reset_to_start();
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}

/* HAL weak callback hook */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    uint8_t rx_byte = s_rx_byte;
    /* NEW: enqueue for debug print */
    // Comm_DebugQueue(rx_byte);

    s_dbg_irq_count++;
    Comm_Debug_ByteEnq(rx_byte);

    switch (s_rx_state)
    {
    case RX_WAIT_START:
        if (rx_byte == COMM_START_BYTE)
        {
            s_crc_calc = 0xFFFFu;
            s_rx_state = RX_TYPE;
        }
        break;

    case RX_TYPE:
        s_rx_type = rx_byte;
        s_crc_calc = crc16_ccitt_update(s_crc_calc, rx_byte);
        s_rx_state = RX_LEN;
        break;

    case RX_LEN:
        s_rx_len = rx_byte;
        s_rx_idx = 0u;
        s_crc_calc = crc16_ccitt_update(s_crc_calc, rx_byte);
        s_rx_state = (s_rx_len > 0u) ? RX_PAYLOAD : RX_CRC_LO;
        break;

    case RX_PAYLOAD:
        if (s_rx_idx < COMM_RX_PAYLOAD_MAX)
        {
            s_rx_payload[s_rx_idx] = rx_byte;
            s_rx_idx++;
            s_crc_calc = crc16_ccitt_update(s_crc_calc, rx_byte);
            if (s_rx_idx >= s_rx_len)
            {
                s_rx_state = RX_CRC_LO;
            }
        }
        else
        {
            rx_reset_to_start(); /* overflow guard */
        }
        break;

    case RX_CRC_LO:
        s_crc_recv = (uint16_t)rx_byte;
        s_rx_state = RX_CRC_HI;
        break;

    case RX_CRC_HI:
        s_crc_recv |= (uint16_t)rx_byte << 8;
        s_rx_state = RX_END;
        break;

    case RX_END:
        uint8_t end_ok = (rx_byte == COMM_END_BYTE) ? 1u : 0u;
        uint8_t crc_ok = (s_crc_recv == s_crc_calc) ? 1u : 0u;
        uint8_t ok = (uint8_t)(end_ok & crc_ok);

        /* ---- DEBUG: enqueue frame snapshot (valid OR invalid) ---- */
        Comm_Debug_FrameEnq(s_rx_type, s_rx_len, ok, s_rx_payload);

        if (ok)
        {

            if ((rx_byte == COMM_END_BYTE) && (s_crc_recv == s_crc_calc))
            {
                /* 1) Internal handling for minimal changes elsewhere */
                if (s_rx_type == COMM_MSG_SET_STATE)
                {
                    if (s_rx_len == 1u)
                    {
                        Comm_Set_Received_Command((s_rx_payload[0] != 0u) ? EFUSE_COMMAND_CLOSED
                                                                          : EFUSE_COMMAND_OPEN);
                    }
                }
                else if (s_rx_type == COMM_MSG_EE_TEXT)
                {
                    /* Push ASCII control line to EEPROM module */
                    char ee_line_buffer[256];
                    uint8_t copy_len = s_rx_len;
                    if (copy_len >= (uint8_t)sizeof(ee_line_buffer))
                    {
                        copy_len = (uint8_t)(sizeof(ee_line_buffer) - 1u);
                    }
                    if (copy_len > 0u)
                    {
                        (void)memcpy(ee_line_buffer, s_rx_payload, copy_len);
                    }
                    ee_line_buffer[copy_len] = '\0';
                    EE_RequestLine(ee_line_buffer);
                }
                else if (s_rx_type == COMM_MSG_PROT_UPDATE)
                {
                    if (s_rx_len >= (6u * 4u))
                    {
                        uint32_t idx = 0u;
                        s_prot_update_buf.v_load_min_mv = read_i32_le(s_rx_payload, idx);
                        idx += 4u;
                        s_prot_update_buf.v_load_max_mv = read_i32_le(s_rx_payload, idx);
                        idx += 4u;
                        s_prot_update_buf.i_tcc_max_ma = read_i32_le(s_rx_payload, idx);
                        idx += 4u;
                        s_prot_update_buf.i_nom_max_ma = read_i32_le(s_rx_payload, idx);
                        idx += 4u;
                        s_prot_update_buf.t_ext_min_c = read_i32_le(s_rx_payload, idx);
                        idx += 4u;
                        s_prot_update_buf.t_ext_max_c = read_i32_le(s_rx_payload, idx);
                        idx += 4u;

                        s_prot_update_pending = 1u; /* flag for scheduler */
                    }
                }

                /* 2) Queue the frame for any external consumer */
                if (s_have_frame == 0u)
                {
                    s_q_type = s_rx_type;
                    s_q_len = s_rx_len;
                    if (s_q_len > 0u)
                    {
                        (void)memcpy(s_q_payload, s_rx_payload, s_q_len);
                    }
                    s_have_frame = 1u;
                }
            }
        }
        rx_reset_to_start();
        break;

    default:
        rx_reset_to_start();
        break;
    }

    /* re-arm */
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
    s_rx_pending = 1;
}

int Comm_Frame_TryDequeue(uint8_t *message_type_ptr, uint8_t *payload_out_ptr, uint8_t *payload_len_ptr)
{
    if (s_have_frame == 0u)
    {
        return 0;
    }
    s_have_frame = 0u;

    if (message_type_ptr != NULL)
    {
        *message_type_ptr = s_q_type;
    }
    if (payload_len_ptr != NULL)
    {
        *payload_len_ptr = s_q_len;
    }
    if ((payload_out_ptr != NULL) && (s_q_len > 0u))
    {
        (void)memcpy(payload_out_ptr, s_q_payload, s_q_len);
    }
    return 1;
}

/**
 * @brief Reset RX state machine to initial state
 */
static void rx_reset_to_start(void)
{
    s_rx_state = RX_WAIT_START;
    s_crc_calc = 0xFFFFu;
    s_rx_len = 0u;
    s_rx_idx = 0u;
}

/*============================================================================*/
/*                              TX Implementation                             */
/*============================================================================*/
/**
 * @brief UART TX complete callback (HAL weak override)
 */
// void Comm_Frame_Send(uint8_t message_type, const uint8_t *payload_ptr, uint8_t payload_len)
// {
//     const uint8_t start_byte = COMM_START_BYTE;
//     const uint8_t end_byte   = COMM_END_BYTE;

//     (void)HAL_UART_Transmit(&huart2, (uint8_t *)&start_byte, 1, HAL_MAX_DELAY);
//     (void)HAL_UART_Transmit(&huart2, &message_type, 1, HAL_MAX_DELAY);
//     (void)HAL_UART_Transmit(&huart2, &payload_len, 1, HAL_MAX_DELAY);
//     if ((payload_ptr != NULL) && (payload_len > 0u)) {
//         (void)HAL_UART_Transmit(&huart2, (uint8_t *)payload_ptr, payload_len, HAL_MAX_DELAY);
//     }
//     uint16_t crc_value = crc16_ccitt_update_buf(message_type, payload_ptr, payload_len);
//     uint8_t crc_le_bytes[2] = { (uint8_t)(crc_value & 0xFFu), (uint8_t)(crc_value >> 8) };
//     (void)HAL_UART_Transmit(&huart2, crc_le_bytes, 2, HAL_MAX_DELAY);
//     (void)HAL_UART_Transmit(&huart2, (uint8_t *)&end_byte, 1, HAL_MAX_DELAY);
// }

/* Enqueue a framed packet into the circular TX buffer and start IT transmit if idle.
   This implementation writes bytes one-by-one with wrapping to avoid any out-of-bounds
   writes that corrupt the HAL state. It also reserves one byte so read==write always
   means empty. */
void Comm_Frame_Send(uint8_t message_type, const uint8_t *payload_ptr, uint8_t payload_len)
{
    // Disable interrupts while checking/updating TX buffer
    __disable_irq();

    /* frame: START | TYPE | LEN | PAYLOAD... | CRC_LO | CRC_HI | END */
    uint16_t frame_size = (uint16_t)(6u + payload_len);
    if (frame_size >= TX_BUFFER_SIZE)
    { /* cannot fit even if empty */
        __enable_irq();
        return;
    }

    /* compute used bytes and free space (reserve one byte) */
    uint16_t used = (uint16_t)((s_tx_write + TX_BUFFER_SIZE - s_tx_read) % TX_BUFFER_SIZE);
    uint16_t free_space = (uint16_t)(TX_BUFFER_SIZE - used - 1u);
    if (free_space < frame_size)
    {
        __enable_irq();
        return;
    } /* not enough room */

/* helper to push a single byte with wrap */
    auto_push_byte:;
    /* enqueue: start */
    s_tx_buffer[s_tx_write] = COMM_START_BYTE;
    s_tx_write = (uint16_t)((s_tx_write + 1u) % TX_BUFFER_SIZE);

    /* type, len */
    s_tx_buffer[s_tx_write] = message_type;
    s_tx_write = (uint16_t)((s_tx_write + 1u) % TX_BUFFER_SIZE);
    s_tx_buffer[s_tx_write] = payload_len;
    s_tx_write = (uint16_t)((s_tx_write + 1u) % TX_BUFFER_SIZE);

    /* payload */
    if ((payload_len > 0u) && (payload_ptr != NULL))
    {
        for (uint8_t i = 0u; i < payload_len; ++i)
        {
            s_tx_buffer[s_tx_write] = payload_ptr[i];
            s_tx_write = (uint16_t)((s_tx_write + 1u) % TX_BUFFER_SIZE);
        }
    }

    /* crc */
    uint16_t crc = crc16_ccitt_update_buf(message_type, payload_ptr, payload_len);
    s_tx_buffer[s_tx_write] = (uint8_t)(crc & 0xFFu);
    s_tx_write = (uint16_t)((s_tx_write + 1u) % TX_BUFFER_SIZE);
    s_tx_buffer[s_tx_write] = (uint8_t)(crc >> 8);
    s_tx_write = (uint16_t)((s_tx_write + 1u) % TX_BUFFER_SIZE);

    /* end */
    s_tx_buffer[s_tx_write] = COMM_END_BYTE;
    s_tx_write = (uint16_t)((s_tx_write + 1u) % TX_BUFFER_SIZE);

    /* Start transmission if not already busy.
       We only ever transmit one byte at a time in IT mode; Transfer continues
       from HAL_UART_TxCpltCallback. */
    if (!s_tx_busy)
    {
        s_tx_busy = 1;
        s_tx_pending = 1;
        (void)HAL_UART_Transmit_IT(&huart2, (uint8_t *)&s_tx_buffer[s_tx_read], 1);
    }
    __enable_irq();
}

/* Existing names preserved → now they emit frames instead of ASCII */
void Comm_Send_Efuse_Command(efuse_comm_command_t command_code)
{
    uint8_t state_byte;
    switch (command_code)
    {
    case EFUSE_COMMAND_OPEN:
        state_byte = 0u;
        break;
    case EFUSE_COMMAND_CLOSED:
        state_byte = 1u;
        break;
    default:
        return;
    }
    Comm_Frame_Send(COMM_MSG_SET_STATE, &state_byte, 1u);
}

void Comm_Send_Efuse_State(uint8_t state_on)
{
    uint8_t state_byte = (state_on != 0u) ? 1u : 0u;
    Comm_Frame_Send(COMM_MSG_STATE_OUT, &state_byte, 1u);
}

/* Typed helpers used by scheduler (if desired) */
void Comm_Send_MetrologyOut(uint8_t state_on, uint32_t vline_mv, uint32_t vload_mv, uint32_t t_current_ma, uint32_t n_current_ma, uint32_t ext_temp)
{
    uint8_t payload_buffer[1u + 4u + 4u + 4u + 4u + 4u];
    payload_buffer[0] = (state_on != 0u) ? 1u : 0u;
    write_u32_le(&payload_buffer[1], vline_mv);
    write_u32_le(&payload_buffer[5], vload_mv);
    write_u32_le(&payload_buffer[9], t_current_ma);
    write_u32_le(&payload_buffer[13], n_current_ma);
    write_u32_le(&payload_buffer[17], ext_temp);
    Comm_Frame_Send(COMM_MSG_METROLOGY_OUT, payload_buffer, (uint8_t)sizeof(payload_buffer));
}

/* Logger → frames (0xF8) */
void Comm_Logf(const char *format_str, ...)
{
    char log_buf[200];
    va_list args;
    va_start(args, format_str);
    int print_len = vsnprintf(log_buf, (int)sizeof(log_buf), format_str, args);
    va_end(args);

    if (print_len <= 0)
    {
        return;
    }
    if (print_len > (int)sizeof(log_buf))
    {
        print_len = (int)sizeof(log_buf);
    }

    Comm_Frame_Send(COMM_MSG_LOG_TEXT, (uint8_t *)log_buf, (uint8_t)print_len);
}

// TX callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
        return;

    s_tx_read = (s_tx_read + 1) % TX_BUFFER_SIZE;

    if (s_tx_read != s_tx_write)
    {
        // More data to send
        HAL_UART_Transmit_IT(&huart2, &s_tx_buffer[s_tx_read], 1);
        s_tx_pending = 1;
    }
    else
    {
        s_tx_busy = 0; // Done sending
    }
}

/*============================================================================*/
/*                              Command Interface                             */
/*============================================================================*/

/**
 * @brief Set received command in mailbox
 * @param command_code Command to store
 */
void Comm_Set_Received_Command(efuse_comm_command_t command_code)
{
    s_cmd_mailbox = command_code;
}

/**
 * @brief Get and clear received command
 * @return Last received command, clears to NONE after read
 */
efuse_comm_command_t Comm_Get_Received_Command(void)
{
    efuse_comm_command_t command_snapshot = s_cmd_mailbox;
    s_cmd_mailbox = EFUSE_COMMAND_NONE;
    return command_snapshot;
}

int Comm_TryGetPendingProtUpdate(comm_prot_limits_t *out)
{
    if ((out == NULL) || (s_prot_update_pending == 0u))
    {
        return 0;
    }
    *out = s_prot_update_buf;
    s_prot_update_pending = 0u; /* consume once */
    return 1;
}

/*============================================================================*/
/*                              Debug Implementation                          */
/*============================================================================*/

static inline void Comm_Debug_ByteEnq(uint8_t b)
{
    uint16_t n = (uint16_t)((s_dbgb_w + 1u) % COMM_DBG_BYTE_QSZ);
    if (n != s_dbgb_r)
    {
        s_dbgb_q[s_dbgb_w] = b;
        s_dbgb_w = n;
    }
}

static inline void Comm_Debug_FrameEnq(uint8_t type, uint8_t len, uint8_t crc_ok, const uint8_t *pay)
{
    uint8_t next = (uint8_t)((s_dbgf_w + 1u) % COMM_DBGF_QDEPTH);
    if (next == s_dbgf_r)
    {
        return;
    } /* drop on overflow */
    s_dbgf_q[s_dbgf_w].type = type;
    s_dbgf_q[s_dbgf_w].len = len;
    s_dbgf_q[s_dbgf_w].crc_ok = crc_ok;
    uint8_t n = len;
    if (n > COMM_DBGF_MAX_PAY)
        n = COMM_DBGF_MAX_PAY;
    for (uint8_t i = 0u; i < n; ++i)
    {
        s_dbgf_q[s_dbgf_w].pay[i] = pay[i];
    }
    s_dbgf_w = next;
}

/* ---- call from scheduler comm event ---- */
void Comm_Debug_Dump(void)
{
    /* 0) ISR heartbeat – prints once every call so you know ISR is alive */
    static uint32_t last_irq = 0u;
    if (s_dbg_irq_count != last_irq)
    {
        Comm_Logf("DBG: irq_bytes=%lu", (unsigned long)s_dbg_irq_count);
        last_irq = s_dbg_irq_count;
    }

    /* 1) Byte-by-byte (optional; comment out if too chatty) */
    while (s_dbgb_r != s_dbgb_w)
    {
        uint8_t b = s_dbgb_q[s_dbgb_r];
        s_dbgb_r = (uint16_t)((s_dbgb_r + 1u) % COMM_DBG_BYTE_QSZ);
        Comm_Logf("RX byte: 0x%02X", b);
    }

    /* 2) Frame summaries (valid and invalid) */
    while (s_dbgf_r != s_dbgf_w)
    {
        comm_dbgf_t f = s_dbgf_q[s_dbgf_r];
        s_dbgf_r = (uint8_t)((s_dbgf_r + 1u) % COMM_DBGF_QDEPTH);

        Comm_Logf("FRAME: type=0x%02X len=%u crc=%s", f.type, f.len, f.crc_ok ? "OK" : "FAIL");
        if (f.len > 0u)
        {
            char line[96];
            uint8_t shown = (f.len > COMM_DBGF_MAX_PAY) ? COMM_DBGF_MAX_PAY : f.len;
            uint8_t i = 0u;
            while (i < shown)
            {
                int pos = 0;
                pos += snprintf(line + pos, sizeof(line) - (size_t)pos, "  PAY: ");
                uint8_t chunk = (shown - i > 16u) ? 16u : (uint8_t)(shown - i);
                for (uint8_t k = 0u; k < chunk; ++k)
                {
                    pos += snprintf(line + pos, sizeof(line) - (size_t)pos, "%02X ", f.pay[i + k]);
                }
                Comm_Logf("%s", line);
                i = (uint8_t)(i + chunk);
            }
            if (f.len > COMM_DBGF_MAX_PAY)
            {
                Comm_Logf("  (payload truncated; total %u bytes)", f.len);
            }
        }
    }
}
/*============================================================================*/
/*                       UART TX/RX BASIC TESTER (new)                        */
/*============================================================================*/

void Comm_Test_Enable(uint8_t enable)
{
    s_test_enabled = (enable != 0u) ? 1u : 0u;
}

/* Transmit a simple line without framing */
void Comm_Test_TxHello(void)
{
    if (s_test_enabled == 0u)
    {
        return;
    }
    static const char hello[] = "Hello, World!\r\n";
    (void)HAL_UART_Transmit_IT(&huart2, (uint8_t *)hello, (uint16_t)(sizeof(hello) - 1u));
}

/* Echo back any raw bytes received (no framing). Safe: runs in thread context. */
void Comm_Test_Poll(void)
{
    if (s_test_enabled == 0u)
    {
        return;
    }

    /* Drain raw byte tap into a small local buffer and write in chunks */
    /* Avoid calling blocking HAL transmit while IT-driven TX may be active.
           Forward received debug bytes into framed logger (uses same IT queue safely). */
    uint8_t chunk[32];
    while (s_dbgb_r != s_dbgb_w)
    {
        uint8_t n = 0u;
        while ((s_dbgb_r != s_dbgb_w) && (n < sizeof(chunk)))
        {
            chunk[n++] = s_dbgb_q[s_dbgb_r];
            s_dbgb_r = (uint16_t)((s_dbgb_r + 1u) % COMM_DBG_BYTE_QSZ);
        }
        if (n > 0u)
        {
            /* log raw bytes as a framed text message (may be truncated) */
            Comm_Frame_Send(COMM_MSG_LOG_TEXT, chunk, n);
        }
    }
}

/*============================================================================*/
/*                              Helper Functions                              */
/*============================================================================*/
static inline void write_u32_le(uint8_t *dest_ptr, uint32_t value)
{
    dest_ptr[0] = (uint8_t)(value);
    dest_ptr[1] = (uint8_t)(value >> 8);
    dest_ptr[2] = (uint8_t)(value >> 16);
    dest_ptr[3] = (uint8_t)(value >> 24);
}

/* Local helper: read 32-bit little-endian signed int */
static inline int32_t read_i32_le(const uint8_t *buf, uint32_t offset)
{
    return (int32_t)(((uint32_t)buf[offset + 0]) |
                     ((uint32_t)buf[offset + 1] << 8) |
                     ((uint32_t)buf[offset + 2] << 16) |
                     ((uint32_t)buf[offset + 3] << 24));
}

/*============================================================================*/
/*                              Process Remaining Frames                      */
/*============================================================================*/
void Comm_ProcessPending(void)
{
    // Handle TX pending
    if (s_tx_pending) {
        s_tx_pending = 0;
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)&s_tx_buffer[s_tx_read], 1);
    }

    // Handle RX pending
    if (s_rx_pending) {
        s_rx_pending = 0;
        HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
    }
}

/*============================================================================*/
/*                              CRC Implementation                            */
/*============================================================================*/

/* CRC-16/CCITT-FALSE */
static uint16_t crc16_ccitt_update(uint16_t crc_state, uint8_t data_byte)
{
    crc_state = (uint16_t)(crc_state ^ ((uint16_t)data_byte << 8));
    for (uint8_t bit_index = 0u; bit_index < 8u; bit_index++)
    {
        if ((crc_state & 0x8000u) != 0u)
        {
            crc_state = (uint16_t)((crc_state << 1) ^ 0x1021u);
        }
        else
        {
            crc_state = (uint16_t)(crc_state << 1);
        }
    }
    return crc_state;
}

static uint16_t crc16_ccitt_update_buf(uint8_t message_type, const uint8_t *payload_ptr, uint8_t payload_len)
{
    uint16_t crc_state = 0xFFFFu;
    crc_state = crc16_ccitt_update(crc_state, message_type);
    crc_state = crc16_ccitt_update(crc_state, payload_len);
    for (uint8_t index = 0u; index < payload_len; index++)
    {
        crc_state = crc16_ccitt_update(crc_state, payload_ptr[index]);
    }
    return crc_state;
}

/*============================================================================*/
/*                              Error Handling                                */
/*============================================================================*/

/**
 * @brief UART error callback
 * @param huart UART handle that had error
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    // Reset RX state machine on any UART error
    rx_reset_to_start();

    // Re-enable reception
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}
/* ================= END COMM DEBUG MODULE ================= */