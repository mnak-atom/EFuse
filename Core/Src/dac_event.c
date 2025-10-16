/**
 * @file dac_event.c                                                     // file header
 * @brief Simple DAC driver wrapper mirroring adc_event style.           // brief
 */
#include "dac_event.h"                                                   // own header
#include <string.h>                                                      // memset
#include <stdio.h>                                                       // optional printf

/* ---------------- Local map (STM32L0) ---------------- */              // section

/* Map our logical channel(s) to HAL channels; keep array for extensibility. */ // note
static const uint32_t k_hal_dac_channel[DAC_CH_COUNT] = {               // map table
    DAC_CHANNEL_1                                                        // DAC_CH_THRESHOLD → CH2
};                                                                       // end table

/* Storage for each logical DAC output. */                               // comment
volatile dac_output_t dac_out[DAC_CH_COUNT];                             // globals (volatile)

/* ---------------- Helpers --------------------------- */               // section

static inline uint16_t clamp_u16(uint16_t x, uint16_t hi)               // clamp helper
{                                                                       // {
    return (x > hi) ? hi : x;                                           // clamp to hi
}                                                                       // }

/* ---------------- Public API ------------------------ */               // section

int dac_init(void)                                                      // init function
{                                                                       // {
    /* Start DAC channel(s) we use; assumes MX_DAC_Init() already ran. */ // note
    HAL_StatusTypeDef status;                                               // HAL status
    memset((void*)dac_out, 0, sizeof(dac_out));                         // clear state

    /* Start CH2 by default (PA5). */                                   // which channel
    status = HAL_DAC_Start(&hdac, k_hal_dac_channel[DAC_CH_THRESHOLD]);     // start channel
    if (status != HAL_OK) {                                                 // check
        return -1;                                                      // error
    }                                                                   // end if
    /* Apply initial 0V. */                                             // init value
    (void)HAL_DAC_SetValue(&hdac, k_hal_dac_channel[DAC_CH_THRESHOLD],
                           DAC_ALIGN_12B_R, 0u);                        // write zero
    dac_out[DAC_CH_THRESHOLD].pending_code = 0u;                        // record
    dac_out[DAC_CH_THRESHOLD].latched_code = 0u;                        // record
    dac_out[DAC_CH_THRESHOLD].last_mv      = 0u;                        // record
    return 0;                                                           // ok
}                                                                       // }

int dac_set_code(dac_channel_id_t ch, uint16_t code)                    // set raw code
{                                                                       // {
    if ((unsigned)ch >= DAC_CH_COUNT) return -2;                        // guard id
    uint16_t c = clamp_u16(code, 4095u);                                // clamp to 12-bit
    dac_out[ch].pending_code = c;                                       // store pending
    return 0;                                                           // ok
}                                                                       // }

int dac_set_mv(dac_channel_id_t ch, uint16_t mv, uint16_t vref_mv)      // set millivolts
{                                                                       // {
    if ((unsigned)ch >= DAC_CH_COUNT) return -2;                        // guard id
    if (vref_mv == 0u) return -3;                                       // guard div0
    if (mv > vref_mv) mv = vref_mv;                                     // clamp to VREF
    /* Convert mv→code: code = round( mv * (4095 / vref_mv) ). */       // formula
    uint32_t code = ((uint32_t)mv * 4095u + (vref_mv/2u)) / (uint32_t)vref_mv; // scaled
    dac_out[ch].last_mv = mv;                                           // store mv
    return dac_set_code(ch, (uint16_t)code);                            // reuse setter
}                                                                       // }

void dac_event_run(void)                                                // periodic apply
{                                                                       // {
    /* Apply any pending value once; do not spam the peripheral. */      // behavior
    for (int ch = 0; ch < (int)DAC_CH_COUNT; ++ch) {                    // loop channels
        uint16_t pend = dac_out[ch].pending_code;                       // read pending
        if (pend != dac_out[ch].latched_code) {                         // needs update?
            uint32_t hal_ch = k_hal_dac_channel[ch];                    // hal channel
            if (HAL_DAC_SetValue(&hdac, hal_ch, DAC_ALIGN_12B_R, pend)  // write code
                == HAL_OK) {                                            // ok?
                dac_out[ch].latched_code = pend;                        // commit
                /* Only logs if LOG_GLOBAL_ENABLE && LOG_DAC_ENABLE are set */
                LOG_DAC("DAC ch=%u code=%lu mv=%lu",
                        (unsigned)ch,
                        (unsigned long)dac_out[ch].latched_code,
                        (unsigned long)dac_out[ch].last_mv);

            
            } else {                                                    // otherwise
                /* optional: log error here */                          // note
            }                                                           // end if
        }                                                               // end if changed
    }                                                                   // end for
}                                                                       // }
