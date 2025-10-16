/**
 * @file dac_event.h                                                     // header: file name
 * @brief Lightweight DAC wrapper (STM32L0 DAC) similar to adc_event.*   // brief: purpose
 *        - No malloc; simple blocking HAL writes                        // design: constraints
 *        - Channel map: uses DAC_CHANNEL_2 on PA5 by default            // default mapping
 *        - Provides code/mV setters and a periodic apply()              // API overview
 */
#ifndef DAC_EVENT_H                                                      // include guard start
#define DAC_EVENT_H                                                      // include guard define

#include <stdint.h>                                                      // fixed-width ints
#include <stdbool.h>                                                     // bool type
#include "stm32l0xx_hal.h"                                               // HAL base (L0 family)
#include "main.h"                                                        // board/peripheral handles

#ifdef __cplusplus                                                       // C++ interop
extern "C" {                                                             // extern "C" begin
#endif

/* ---------------- Public types ---------------- */                     // section: types

typedef enum {                                                           // enum of DAC ch ids
    DAC_CH_THRESHOLD = 0,                                                // logical threshold ch
    DAC_CH_COUNT                                                          // count of channels
} dac_channel_id_t;                                                      // typedef name

typedef struct {                                                         // setpoint struct
    uint16_t pending_code;                                               // code requested (0..4095)
    uint16_t latched_code;                                               // code last applied
    uint16_t last_mv;                                                    // last mv requested (for info)
} dac_output_t;                                                          // typedef name

/* ---------------- Public data ----------------- */                     // section: data

extern DAC_HandleTypeDef hdac;                                           // HAL DAC handle (CubeMX)
extern volatile dac_output_t dac_out[DAC_CH_COUNT];                      // outputs state

/* ---------------- Public API ------------------ */                     // section: API

/**
 * @brief Initialize and start the DAC channels used by this module.     // brief
 * @return 0 on success, negative on error.                               // return
 * @note Safe to call once after HAL init; idempotent per run.            // note
 */
int dac_init(void);                                                      // prototype

/**
 * @brief Set a raw 12-bit DAC code (0..4095) for a channel.             // brief
 * @param ch Logical channel id (e.g., DAC_CH_THRESHOLD).                 // param
 * @param code 12-bit code to output (clamped).                           // param
 * @return 0 on success, negative on error.                               // return
 */
int dac_set_code(dac_channel_id_t ch, uint16_t code);                    // prototype

/**
 * @brief Set a DAC output in millivolts; converts to 12-bit code.        // brief
 * @param ch Logical channel id.                                          // param
 * @param mv Millivolts to output (0..VREF mv).                           // param
 * @param vref_mv Reference in millivolts (e.g., 3300).                   // param
 * @return 0 on success, negative on error.                               // return
 */
int dac_set_mv(dac_channel_id_t ch, uint16_t mv, uint16_t vref_mv);      // prototype

/**
 * @brief Periodic service: applies any pending setpoints once.           // brief
 * @note Call from a periodic task (Style 3) or inline where convenient.  // note
 */
void dac_event_run(void);                                                // prototype

#ifdef __cplusplus                                                       // C++ interop
}                                                                        // extern "C" end
#endif

#endif /* DAC_EVENT_H */                                                 // include guard end
