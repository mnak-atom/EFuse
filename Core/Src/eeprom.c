/**
 * @file eeprom.c
 * @author mnak
 * @brief This file provides EEPROM read/write functions.
 * @version 0.1
 * @date 2025-09-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "eeprom.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_flash_ex.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define SETPOINT_SIZE_BYTES 8U
#define SETPOINT_SIZE_ID 2U
#define SETPOINT_SIZE_VALUE 4U
#define SETPOINT_SIZE_CRC 2U
#define SETPOINT_EEPROM_BASE  (DATA_EEPROM_BASE + 0x100U) // adjust offset as needed

/* ========================================================================== */
/* Internal utilities (file-scope, static)                                    */
/* ========================================================================== */

// static uint16_t ComputeCRC16(const uint8_t *data, size_t length);

/* ========================= CRC16/CCITT-FALSE (0x1021) ========================== */
/* poly=0x1021, init=0xFFFF, no xorout, input not reflected                        */
static uint16_t crc16_ccitt_false_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                               : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t crc16_ccitt_false(const uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0; i < len; ++i) crc = crc16_ccitt_false_update(crc, buf[i]);
    return crc;
}

/* =============================== Little-endian IO ============================== */
static inline void u16_to_le(uint8_t out[2], uint16_t v)
{ out[0]=(uint8_t)(v); out[1]=(uint8_t)(v>>8); }

static inline void u32_to_le(uint8_t out[4], uint32_t v)
{ out[0]=(uint8_t)(v); out[1]=(uint8_t)(v>>8); out[2]=(uint8_t)(v>>16); out[3]=(uint8_t)(v>>24); }


/* ========================================================================== */
bool EEPROM_WriteSetpoint(uint16_t id, uint32_t value)
{
    // uint16_t crc16 = 0;
    const uint32_t addr = EE_GetAddressForId(id);
    if (addr == 0u) {
        return false; // Unknown Ids
    }

    uint8_t rec[EE_REC_SIZE_BYTES];
    u16_to_le(&rec[0], id);
    u32_to_le(&rec[2], value);
    const uint16_t crc = crc16_ccitt_false(rec, 6u); /* CRC over {Id(2),Value(4)} */
    u16_to_le(&rec[6], crc);

    // TODO: Compute CRC over ID + value

    if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK) {
        return false;
    }

    // // Write ID (2 bytes)
    // if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD,
    //                                    addr,
    //                                    id) != HAL_OK) {
    //     HAL_FLASHEx_DATAEEPROM_Lock();
    //     return false;
    // }

    // // Write value (4 bytes)
    // if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,
    //                                    (addr + 2U),
    //                                    value) != HAL_OK) {
    //     HAL_FLASHEx_DATAEEPROM_Lock();
    //     return false;
    // }

    // // Write CRC (2 bytes)
    // if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD,
    //                                    (addr + 6U),
    //                                    crc16) != HAL_OK) {
    //     HAL_FLASHEx_DATAEEPROM_Lock();
    //     return false;
    // }

    // HAL_FLASHEx_DATAEEPROM_Lock();
    // return true;

    for (uint32_t i = 0; i < EE_REC_SIZE_BYTES; i += 2u) {
        const uint16_t hw = (uint16_t)rec[i] | ((uint16_t)rec[i+1] << 8);
        if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD,
                                           addr + i, hw) != HAL_OK) {
            (void)HAL_FLASHEx_DATAEEPROM_Lock();
            return false;
        }
    }

    (void)HAL_FLASHEx_DATAEEPROM_Lock();
    return true;
}



/**
 * @brief  Return true if [address, address + byte_count - 1] fits EEPROM range.
 */
static bool address_range_is_valid(uint32_t address, size_t byte_count)
{
    bool is_valid = false;

    if (byte_count > 0U) {
        if (address >= DATA_EEPROM_BASE) {
            uint32_t last_address = address + (uint32_t)byte_count - 1UL;
            if (last_address <= DATA_EEPROM_END) {
                is_valid = true;
            }
        }
    }
    return is_valid;
}

/**
 * @brief  Map one hex digit to its 0..15 value; returns 0xFF on invalid.
 */
static uint8_t hex_nibble_value(char character)
{
    if ((character >= '0') && (character <= '9')) {
        return (uint8_t)(character - '0');
    }

    char lower = (char)tolower((unsigned char)character);
    if ((lower >= 'a') && (lower <= 'f')) {
        return (uint8_t)(10 + (lower - 'a'));
    }

    return 0xFFU;
}

/**
 * @brief  Parse an unsigned 32-bit integer written in hex (no 0x prefix).
 * @param  text            Pointer to ASCII hex.
 * @param  parsed_value    Output value on success.
 * @return Pointer to first non-hex char, or NULL on error/overflow/no digits.
 */
static const char *parse_hex_u32(const char *text, uint32_t *parsed_value)
{
    uint32_t value = 0U;
    int digit_count = 0;

    while (true) {
        uint8_t nibble = hex_nibble_value(*text);
        if (nibble == 0xFFU) {
            break;
        }
        if (digit_count == 8) {           /* prevent overflow */
            return NULL;
        }
        value = (value << 4) | nibble;
        ++text;
        ++digit_count;
    }

    if (digit_count == 0) {
        return NULL;
    }

    *parsed_value = value;
    return text;
}

/**
 * @brief  Parse a string of hex digits into a byte buffer.
 *         Accepts spaces/underscores; requires even digit count.
 * @param  text                 Input ASCII.
 * @param  output_buffer        Destination buffer.
 * @param  output_capacity      Size of destination buffer in bytes.
 * @param  output_length_bytes  Number of bytes parsed (out).
 * @return true on success; false on invalid/odd digits or overflow.
 */
static bool parse_hex_bytes(const char *text,
                            uint8_t *output_buffer,
                            size_t output_capacity,
                            size_t *output_length_bytes)
{
    size_t out_index = 0U;
    bool have_high_nibble = false;
    uint8_t high_nibble_value = 0U;

    while (*text != '\0') {
        if ((*text == ' ') || (*text == '\t') || (*text == '_')) {
            ++text;
            continue;
        }

        uint8_t nibble = hex_nibble_value(*text);
        if (nibble == 0xFFU) {
            break; /* stop at first non-hex */
        }

        if (have_high_nibble == false) {
            high_nibble_value = nibble;
            have_high_nibble = true;
        } else {
            if (out_index >= output_capacity) {
                return false; /* overflow */
            }
            output_buffer[out_index] = (uint8_t)((high_nibble_value << 4) | nibble);
            ++out_index;
            have_high_nibble = false;
        }
        ++text;
    }

    if (have_high_nibble == true) {
        return false; /* odd number of hex digits */
    }

    *output_length_bytes = out_index;
    return true;
}

/**
 * @brief  Case-insensitive prefix match; returns pointer past the prefix or NULL.
 */
static const char *skip_case_insensitive_prefix(const char *text, const char *prefix)
{
    while (*prefix != '\0') {
        if (tolower((unsigned char)*text) != tolower((unsigned char)*prefix)) {
            return NULL;
        }
        ++text;
        ++prefix;
    }
    return text;
}

/**
 * @brief  Skip spaces and a single ':' or '=' separator if present.
 */
static const char *skip_separators(const char *text)
{
    while ((*text == ' ') || (*text == '\t')) {
        ++text;
    }
    if ((*text == ':') || (*text == '=')) {
        ++text;
    }
    while ((*text == ' ') || (*text == '\t')) {
        ++text;
    }
    return text;
}

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

/**
 * @brief  Initialize the Data-EEPROM interface (unlock/lock sanity check).
 */
bool EEPROM_Init(void)
{
    if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK) {
        return false;
    }
    if (HAL_FLASHEx_DATAEEPROM_Lock() != HAL_OK) {
        return false;
    }
    return true;
}

/**
 * @brief  Clear the entire Data-EEPROM region to 0x00 (byte programming).
 */
bool EEPROM_ClearAll(void)
{
    const uint32_t base_address = DATA_EEPROM_BASE;
    const uint32_t end_address  = DATA_EEPROM_END;

    if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK) {
        return false;
    }

    for (uint32_t current_address = base_address;
         current_address <= end_address;
         ++current_address) {

        if (*(volatile uint8_t*)current_address != 0x00U) {
            if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE,
                                               current_address,
                                               0x00U) != HAL_OK) {
                (void)HAL_FLASHEx_DATAEEPROM_Lock();
                return false;
            }
            if (*(volatile uint8_t*)current_address != 0x00U) {
                (void)HAL_FLASHEx_DATAEEPROM_Lock();
                return false;
            }
        }
    }

    (void)HAL_FLASHEx_DATAEEPROM_Lock();
    return true;
}

/**
 * @brief  Clear all EEPROM and then write a banner at DATA_EEPROM_BASE.
 */
bool EEPROM_WriteBannerRaw(const char *banner_text)
{
    if (banner_text == NULL) {
        return false;
    }

    if (EEPROM_ClearAll() == false) {
        (void)printf("[EE] clear-all failed\r\n");
        return false;
    }

    size_t banner_length = strlen(banner_text); /* omit trailing NUL */
    if (address_range_is_valid(DATA_EEPROM_BASE, banner_length) == false) {
        (void)printf("[EE] banner out of range (len=%u)\r\n",
                     (unsigned)banner_length);
        return false;
    }

    if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK) {
        (void)printf("[EE] unlock failed\r\n");
        return false;
    }

    for (size_t byte_index = 0U; byte_index < banner_length; ++byte_index) {
        uint32_t target_address = DATA_EEPROM_BASE + (uint32_t)byte_index;
        uint8_t desired_value   = (uint8_t)banner_text[byte_index];

        if (*(volatile uint8_t*)target_address != desired_value) {
            if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE,
                                               target_address,
                                               desired_value) != HAL_OK) {
                (void)HAL_FLASHEx_DATAEEPROM_Lock();
                (void)printf("[EE] program fail @0x%08lX\r\n",
                             (unsigned long)target_address);
                return false;
            }
            if (*(volatile uint8_t*)target_address != desired_value) {
                (void)HAL_FLASHEx_DATAEEPROM_Lock();
                (void)printf("[EE] verify fail @0x%08lX\r\n",
                             (unsigned long)target_address);
                return false;
            }
        }
    }

    (void)HAL_FLASHEx_DATAEEPROM_Lock();
    (void)printf("[EE] wrote banner @0x%08lX\r\n",
                 (unsigned long)DATA_EEPROM_BASE);
    return true;
}

/**
 * @brief  Write arbitrary bytes into Data-EEPROM (byte-only programming).
 */
bool EEPROM_WriteBytes(uint32_t address, const void *source_bytes, size_t byte_count)
{
    if (address_range_is_valid(address, byte_count) == false) {
        return false;
    }
    if (source_bytes == NULL) {
        return false;
    }

    const uint8_t *source = (const uint8_t *)source_bytes;

    if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK) {
        return false;
    }

    for (size_t byte_index = 0U; byte_index < byte_count; ++byte_index) {
        uint32_t target_address = address + (uint32_t)byte_index;
        uint8_t current_value   = *(volatile uint8_t *)target_address;
        uint8_t desired_value   = source[byte_index];

        /* If any bit requires 1->0, pre-clear to 0x00 (erased state on L0). */
        if ((current_value & (uint8_t)(~desired_value)) != 0U) {
            if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE,
                                               target_address,
                                               0x00U) != HAL_OK) {
                (void)HAL_FLASHEx_DATAEEPROM_Lock();
                return false;
            }
            current_value = 0x00U;
        }

        if (current_value != desired_value) {
            if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE,
                                               target_address,
                                               desired_value) != HAL_OK) {
                (void)HAL_FLASHEx_DATAEEPROM_Lock();
                return false;
            }
            if (*(volatile uint8_t *)target_address != desired_value) {
                (void)HAL_FLASHEx_DATAEEPROM_Lock();
                return false;
            }
        }
    }

    (void)HAL_FLASHEx_DATAEEPROM_Lock();
    return true;
}

/**
 * @brief  Read bytes from Data-EEPROM into a RAM buffer.
 */
bool EEPROM_ReadBytes(uint32_t address, void *destination_buffer, size_t byte_count)
{
    if (address_range_is_valid(address, byte_count) == false) {
        return false;
    }
    if (destination_buffer == NULL) {
        return false;
    }

    (void)memcpy(destination_buffer, (const void *)address, byte_count);
    return true;
}

/**
 * @brief  Print a hex dump (16 bytes per line) from Data-EEPROM.
 */
void EEPROM_Dump(uint32_t address, size_t byte_count)
{
    if ((byte_count == 0U) || (address_range_is_valid(address, byte_count) == false)) {
        return;
    }

    for (size_t line_offset = 0U; line_offset < byte_count; line_offset += 16U) {
        (void)printf("0x%08lX: ", (unsigned long)(address + (uint32_t)line_offset));

        for (size_t byte_index = 0U;
             (byte_index < 16U) && ((line_offset + byte_index) < byte_count);
             ++byte_index) {

            uint32_t current_address = address + (uint32_t)line_offset + (uint32_t)byte_index;
            (void)printf("%02X ", *(volatile uint8_t *)current_address);
        }
        (void)printf("\r\n");
    }
}

/* ========================================================================== */
/* UART command hook                                                          */
/* ========================================================================== */

static volatile uint8_t g_command_ready_flag = 0U;
static char g_command_line_buffer[EE_CMD_BUF_SIZE];

/**
 * @brief  Queue a full text command line (safe to call from ISR).
 */
void EE_RequestLine(const char *command_line)
{
    if (command_line == NULL) {
        return;
    }

    size_t copied_length = 0U;
    while ((command_line[copied_length] != '\0') &&
           (copied_length < (EE_CMD_BUF_SIZE - 1U))) {
        g_command_line_buffer[copied_length] = command_line[copied_length];
        ++copied_length;
    }
    g_command_line_buffer[copied_length] = '\0';
    g_command_ready_flag = 1U;
}

/* ---- individual command handlers (run in main context) ------------------- */

static void handle_cmd_clear(void)
{
    if (EEPROM_ClearAll() != false) {
        (void)printf("[EE] CLEARED\r\n");
    } else {
        (void)printf("[EE] CLEAR FAILED\r\n");
    }
}

static void handle_cmd_banner(void)
{
    (void)EEPROM_WriteBannerRaw(EEPROM_BANNER_TEXT);
}

static void handle_cmd_dump(const char *parameter_text)
{
    /* Syntax: EE:DUMP[:<offset_hex>[:<len_hex>]]; defaults to base, 64 bytes */
    uint32_t offset_bytes = 0U;
    uint32_t length_bytes = 64U;

    if (*parameter_text != '\0') {
        const char *cursor_after_sep = skip_separators(parameter_text);
        const char *after_offset = parse_hex_u32(cursor_after_sep, &offset_bytes);
        if (after_offset == NULL) {
            (void)printf("[EE] bad offset\r\n");
            return;
        }

        if (*after_offset != '\0') {
            const char *after_sep_2 = skip_separators(after_offset);
            const char *after_len   = parse_hex_u32(after_sep_2, &length_bytes);
            if (after_len == NULL) {
                (void)printf("[EE] bad len\r\n");
                return;
            }
        }
    }

    if (length_bytes > (uint32_t)EE_CMD_MAX_BYTES) {
        length_bytes = (uint32_t)EE_CMD_MAX_BYTES;
    }

    uint32_t dump_address = DATA_EEPROM_BASE + offset_bytes;
    EEPROM_Dump(dump_address, (size_t)length_bytes);
}

static void handle_cmd_write(const char *parameter_text)
{
    /* Syntax: EE:WRITE:<offset_hex>:<hexbytes> */
    const char *cursor_after_sep = skip_separators(parameter_text);

    uint32_t offset_bytes = 0U;
    const char *after_offset = parse_hex_u32(cursor_after_sep, &offset_bytes);
    if (after_offset == NULL) {
        (void)printf("[EE] bad offset\r\n");
        return;
    }

    const char *hex_start = skip_separators(after_offset);

    uint8_t write_buffer[EE_CMD_MAX_BYTES];
    size_t  write_length = 0U;

    if ((parse_hex_bytes(hex_start, write_buffer, sizeof write_buffer, &write_length) == false) ||
        (write_length == 0U)) {
        (void)printf("[EE] bad hex data\r\n");
        return;
    }

    uint32_t target_address = DATA_EEPROM_BASE + offset_bytes;
    if (address_range_is_valid(target_address, write_length) == false) {
        (void)printf("[EE] out of range\r\n");
        return;
    }

    if (EEPROM_WriteBytes(target_address, write_buffer, write_length) != false) {
        (void)printf("[EE] wrote %u byte(s) @0x%08lX\r\n",
                     (unsigned)write_length, (unsigned long)target_address);
    } else {
        (void)printf("[EE] write failed\r\n");
    }
}

static void handle_cmd_read(const char *parameter_text)
{
    /* Syntax: EE:READ:<offset_hex>:<len_hex> */
    const char *cursor_after_sep = skip_separators(parameter_text);

    uint32_t offset_bytes = 0U;
    uint32_t length_bytes = 0U;

    const char *after_offset = parse_hex_u32(cursor_after_sep, &offset_bytes);
    if (after_offset == NULL) {
        (void)printf("[EE] bad offset\r\n");
        return;
    }

    const char *after_len = parse_hex_u32(skip_separators(after_offset), &length_bytes);
    if ((after_len == NULL) || (length_bytes == 0U)) {
        (void)printf("[EE] bad len\r\n");
        return;
    }

    if (length_bytes > (uint32_t)EE_CMD_MAX_BYTES) {
        length_bytes = (uint32_t)EE_CMD_MAX_BYTES;
    }

    uint32_t source_address = DATA_EEPROM_BASE + offset_bytes;
    if (address_range_is_valid(source_address, length_bytes) == false) {
        (void)printf("[EE] out of range\r\n");
        return;
    }

    uint8_t read_buffer[EE_CMD_MAX_BYTES];
    if (EEPROM_ReadBytes(source_address, read_buffer, (size_t)length_bytes) == false) {
        (void)printf("[EE] read failed\r\n");
        return;
    }

    /* Print bytes as hex: "0xDE 0xAD 0xBE 0xEF ..." */
    for (uint32_t byte_index = 0U; byte_index < length_bytes; ++byte_index) {
        (void)printf("%02X%s",
                     read_buffer[byte_index],
                     ((byte_index + 1U) < length_bytes) ? " " : "");
    }
    (void)printf("\r\n");
}

/**
 * @brief  Parse and execute any queued command line (call in while(1)).
 */
void EE_Poll(void)
{
    if (g_command_ready_flag == 0U) {
        return;
    }

    g_command_ready_flag = 0U;

    /* Trim leading spaces */
    const char *input_ptr = g_command_line_buffer;
    while ((*input_ptr == ' ') || (*input_ptr == '\t')) {
        ++input_ptr;
    }

    const char *parameters_ptr = NULL;

    parameters_ptr = skip_case_insensitive_prefix(input_ptr, "EE:CLEAR");
    if (parameters_ptr != NULL) {
        handle_cmd_clear();
        return;
    }

    parameters_ptr = skip_case_insensitive_prefix(input_ptr, "EE:BANNER");
    if (parameters_ptr != NULL) {
        handle_cmd_banner();
        return;
    }

    parameters_ptr = skip_case_insensitive_prefix(input_ptr, "EE:DUMP");
    if (parameters_ptr != NULL) {
        handle_cmd_dump(parameters_ptr);
        return;
    }

    parameters_ptr = skip_case_insensitive_prefix(input_ptr, "EE:WRITE");
    if (parameters_ptr != NULL) {
        handle_cmd_write(parameters_ptr);
        return;
    }

    parameters_ptr = skip_case_insensitive_prefix(input_ptr, "EE:READ");
    if (parameters_ptr != NULL) {
        handle_cmd_read(parameters_ptr);
        return;
    }

    (void)printf("[EE] unknown cmd\r\n");
}
