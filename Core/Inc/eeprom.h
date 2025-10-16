#ifndef EEPROM_H
#define EEPROM_H

/*
 * STM32L0 Data-EEPROM helper (byte-only, UART-friendly, MISRA-style naming).
 *
 * Features:
 *  - Clear entire Data-EEPROM to 0x00
 *  - Write/read arbitrary bytes at any offset safely (handles 1->0 transitions)
 *  - Optional UART command interface (parse in main context, not ISR):
 *      EE:CLEAR
 *      EE:BANNER
 *      EE:DUMP[:<offset_hex>[:<len_hex>]]
 *      EE:WRITE:<offset_hex>:<hex_data>
 *      EE:READ:<offset_hex>:<len_hex>
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "comm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Device EEPROM window                                                       */
/* -------------------------------------------------------------------------- */
/* DATA_EEPROM_BASE is provided by the STM32L0 device header.
 * Define DATA_EEPROM_END if your header does not provide it.
 * Adjust size for your specific part if needed.
 */
#ifndef DATA_EEPROM_END
#define DATA_EEPROM_END        (DATA_EEPROM_BASE + 6UL * 1024UL - 1UL) /* 6 KB typical */
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */
#define EEPROM_BANNER_TEXT     "ATOM_POWER_EFUSE"  /* max 16 chars, no NUL */

/* Limit for a single UART READ/WRITE to avoid huge prints/buffers */
#ifndef EE_CMD_MAX_BYTES
#define EE_CMD_MAX_BYTES       (128U)
#endif

#ifndef EE_CMD_BUF_SIZE
#define EE_CMD_BUF_SIZE        (160U)
#endif

/* =============================================================================
 * 1) Immutable IDs — freeze these; no other file should invent IDs
 * =============================================================================
 */
typedef enum {
  EE_ID_EFUSE_STATE        = 0x0001, /* 0 = Open, 1 = Closed */

  EE_ID_VLOAD_MIN_mV       = 0x0101, /* mV */
  EE_ID_VLOAD_MAX_mV       = 0x0102, /* mV */
  EE_ID_I_NOM_MAX_mA       = 0x0103, /* mA */
  EE_ID_T_EXT_MAX_C        = 0x0104, /* degC integer */

  EE_ID_DAC_THRESHOLD_mV   = 0x0201, /* mV */

  EE_ID_DEBOUNCE_HOLD_ms   = 0x0301, /* ms */

  /* Reserve more ranges here as you grow (do NOT change existing IDs) */
} eeprom_id_t;

/* =============================================================================
 * 2) Fixed address map — true in-place updates (no wear leveling)
 *    Example: DATA EEPROM base at 0x0808_0000; EFuse state record at 0x0808_0010
 * =============================================================================
 */
#ifndef EE_BASE_ADDR
  #define EE_BASE_ADDR          ((uint32_t)0x08080000UL)  /* adjust if needed */
#endif
#define EE_REC_SIZE_BYTES       (8u)  /* Id(2) + Value(4) + CRC(2) */

/* Choose deterministic, non-overlapping 8-byte slots (aligned) */
#define EE_ADDR_EFUSE_STATE        (0x08080010UL)
#define EE_ADDR_VLOAD_MIN_mV       (0x08080018UL)
#define EE_ADDR_VLOAD_MAX_mV       (0x08080020UL)
#define EE_ADDR_I_NOM_MAX_mA       (0x08080028UL)
#define EE_ADDR_T_EXT_MAX_C        (0x08080030UL)
#define EE_ADDR_DAC_THRESHOLD_mV   (0x08080038UL)
#define EE_ADDR_DEBOUNCE_HOLD_ms   (0x08080040UL)

/* Id → address */
static inline uint32_t EE_GetAddressForId(uint16_t id)
{
    switch (id) {
    case EE_ID_EFUSE_STATE:       return EE_ADDR_EFUSE_STATE;
    case EE_ID_VLOAD_MIN_mV:      return EE_ADDR_VLOAD_MIN_mV;
    case EE_ID_VLOAD_MAX_mV:      return EE_ADDR_VLOAD_MAX_mV;
    case EE_ID_I_NOM_MAX_mA:      return EE_ADDR_I_NOM_MAX_mA;
    case EE_ID_T_EXT_MAX_C:       return EE_ADDR_T_EXT_MAX_C;
    case EE_ID_DAC_THRESHOLD_mV:  return EE_ADDR_DAC_THRESHOLD_mV;
    case EE_ID_DEBOUNCE_HOLD_ms:  return EE_ADDR_DEBOUNCE_HOLD_ms;
    default:                      return 0u; /* unknown/not stored */
    }
}


/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize the Data-EEPROM interface (unlock/lock sanity check).
 * @return true on success; false if unlock/lock fails.
 *
 * Call once after clock/UART init. Leaves the interface locked when idle.
 */
bool EEPROM_Init(void);

/**
 * @brief  Clear the entire Data-EEPROM region to 0x00 (byte programming).
 * @return true if all bytes become 0x00; false on any HAL error/verify failure.
 */
bool EEPROM_ClearAll(void);

/**
 * @brief  Clear all EEPROM and then write a banner at DATA_EEPROM_BASE.
 * @param  banner_text  Zero-terminated ASCII string (NUL not written).
 * @return true on success; false on error.
 */
bool EEPROM_WriteBannerRaw(const char *banner_text);

/**
 * @brief  Write arbitrary bytes into Data-EEPROM (byte-only programming).
 * @param  address       Absolute EEPROM address.
 * @param  source_bytes  Pointer to bytes to write.
 * @param  byte_count    Number of bytes to write.
 * @return true on success; false on range/HAL/verify failure.
 *
 * If any bit needs a 1->0 transition, the byte is first set to 0x00
 * (erased state on STM32L0 EEPROM) and then programmed to the desired value.
 */
bool EEPROM_WriteBytes(uint32_t address, const void *source_bytes, size_t byte_count);

/**
 * @brief  Read bytes from Data-EEPROM into a RAM buffer.
 * @param  address                Absolute EEPROM address to read from.
 * @param  destination_buffer     Pointer to destination buffer.
 * @param  byte_count             Number of bytes to read.
 * @return true on success; false on range error.
 */
bool EEPROM_ReadBytes(uint32_t address, void *destination_buffer, size_t byte_count);

/**
 * @brief  Print a hex dump (16 bytes per line) for quick inspection.
 * @param  address     Start address in Data-EEPROM.
 * @param  byte_count  Number of bytes to print.
 */
void EEPROM_Dump(uint32_t address, size_t byte_count);

/* -------------------------------------------------------------------------- */
/* UART command hook                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Queue a full text command line (safe to call from ISR).
 * @param  command_line  Zero-terminated received line,
 *                       e.g. "EE:WRITE:0000:DEADBEEF".
 *
 * Copies the line into an internal buffer and returns immediately.
 * Parsing/execution happens later in EE_Poll() (main context).
 */
void EE_RequestLine(const char *command_line);

/**
 * @brief  Parse and execute a pending command (call regularly in while(1)).
 *
 * Recognized commands (case-insensitive):
 *   EE:CLEAR
 *   EE:BANNER
 *   EE:DUMP[:<offset_hex>[:<len_hex>]]
 *   EE:WRITE:<offset_hex>:<hexbytes>
 *   EE:READ:<offset_hex>:<len_hex>
 */
void EE_Poll(void);

/* -------------------------------------------------------------------------- */
/* 3-byte setpoint storage API (Id + 4-byte value + 2-byte CRC)               */
/* -------------------------------------------------------------------------- */
bool EEPROM_WriteSetpoint(uint16_t id, uint32_t value);

#ifdef __cplusplus
}
#endif
#endif /* EEPROM_H */
