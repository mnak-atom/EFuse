/**
 * @file adc_event.c
 * @brief ADC event/state machine and scaling logic for STM32L0xx.
 *
 * Handles ADC initialization, polling, reading, scaling, and state transitions.
 * Supports voltage, current, temperature (internal/external), and VREFINT channels.
 */

#include "stm32l0xx_hal.h"
#include <stdio.h>
#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>   // abs()
#include "stm32l0xx.h"
#include "adc_event.h"
#include "efuse_log.h"   

extern ADC_HandleTypeDef hadc;

/**
 * @brief Storage for ADC readings (raw and scaled).
 */
adc_reading_t adc_data[ADC_CHANNEL_COUNT];

/**
 * @brief ADC channel configuration (matches hardware pins).
 */
static const uint32_t channels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0,          /**< V_LOAD */
    ADC_CHANNEL_1,          /**< V_LINE */
    ADC_CHANNEL_8,          /**< TEMP_EXT (TMP235) */
    ADC_CHANNEL_10,         /**< I_TCC */
    ADC_CHANNEL_11,         /**< I_NOM */
    ADC_CHANNEL_TEMPSENSOR, /**< TEMP_INT */
    ADC_CHANNEL_VREFINT     /**< VREFINT */
};

/**
 * @brief VDDA (mV), refreshed each cycle from VREFINT.
 */
static uint16_t adc_ref_mv = 3300;
uint32_t voltage_mv = 0u;
/* --------- Constants ---------- */
/** @brief 12-bit full-scale code */
#define ADC_FS_CODE            4095U
/** @brief Voltage divider ratio (e.g., 11:1) */
#define VOLTAGE_DIVIDER_RATIO  11U
/** @brief Current sense gain in µV/A (185 mV/A → 185000 µV/A) */
#define CURRENT_SENSE_GAIN_UV  185000U
/** @brief TMP235 offset in mV (500 mV @ 0°C) */
#define TMP235_OFFSET_MV       500

/** @brief Factory calibration addresses (STM32L073/L0x3; VDDA = 3.0 V) */
#define TEMP30_CAL_ADDR        ((uint16_t*) (0x1FF8007A))  /**< raw @ 30°C */
#define TEMP110_CAL_ADDR       ((uint16_t*) (0x1FF8007E))  /**< raw @ 110°C */
#define VREFINT_CAL_ADDR       ((uint16_t*) (0x1FF80078))  /**< raw VREFINT @ 3.0 V */

/* --------- State Machine ---------- */
/**
 * @brief ADC event state machine states.
 */
typedef enum {
    ADCEventStateInit = 0,                /**< Initialization */
    ADCEventStateDisabled,                /**< Disabled */
    ADCEventStateIdle,                    /**< Idle */
    ADCEventStateNormal,                  /**< Normal operation */
    ADCEventStateReadValues,              /**< Read ADC values */
    ADCEventStatePerformCalculations,     /**< Perform scaling/calculations */
    ADCEventStateFault,                   /**< Fault state */
    ADCEventStateWaitingForStateChange    /**< Waiting for state change */
} ADC_EventState_t;

/**
 * @brief ADC event state machine context.
 */
typedef struct {
    ADC_EventState_t state;   /**< Current state */
} ADC_Event_t;

static ADC_Event_t adcEvent = { .state = ADCEventStateInit };

/* ====== Helpers ====== */

/**
 * @brief Set the ADC state machine to a new state.
 * @param new_state New state to set.
 */
static void Set_ADC_State(ADC_EventState_t new_state)
{
    adcEvent.state = new_state;
    // static const char* names[] = {
    //     "Init","Disabled","Idle","Normal","ReadValues","PerformCalculations","Fault","WaitingForStateChange"
    // };
    // int idx = (new_state <= ADCEventStateWaitingForStateChange) ? new_state : 7;
    // printf("[ADC] State: %s\r\n", names[idx]);
}

/**
 * @brief Optionally enable internal ADC channels (VREFINT, TempSensor).
 *        Some HAL variants require explicit enable.
 */
static void adc_optional_enable_internals(void)
{
    /* These are present on many L0 HALs; if missing in your headers, it's safe to remove */
    #ifdef HAL_ADC_MODULE_ENABLED
    #if defined(HAL_ADCEx_EnableVREFINT)
    HAL_ADCEx_EnableVREFINT();
    #endif
    #if defined(HAL_ADCEx_EnableTempSensor)
    HAL_ADCEx_EnableTempSensor();
    #endif
    #endif
    /* Tiny settle time for internal paths */
    HAL_Delay(1);
}

/**
 * @brief Print all ADC readings (raw and scaled) to stdout.
 */
void ADC_Print_All(void)
{
    static const char* name[ADC_CHANNEL_COUNT] = {
        "V_LOAD_mv",    // scaled in mV (at source after divider)
        "V_LINE_mv",    // scaled in mV (at source after divider)
        "TEMP_EXT_cx10",// °C x10 (as produced by scaling)
        "I_TCC_mA",
        "I_NOM_mA",
        "TEMP_INT_cx10",// °C x10
        "VREF_mV"
    };

    for (adc_channel_id_t ch = 0; ch < ADC_CHANNEL_COUNT; ++ch) {
        printf("ADC[%d] %-14s raw=%u  scaled=%d\r\n",
               (int)ch, name[ch], adc_data[ch].raw, (int)adc_data[ch].scaled);
    }
    printf("-----------------------------\r\n");
}

/**
 * @brief Initialize ADC in polling mode and calibrate.
 * @return true if successful, false on failure.
 */
bool adc_init_polling(void)
{
    /* One-time calibration */
    if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK) {
        LOG_ADC("-F- ADC Calibration Failed\r\n");
        return false;
    }
    /* Optional: make sure internal channels are enabled (depends on HAL version) */
    adc_optional_enable_internals();

    LOG_ADC("-I- ADC Polling Init OK\r\n");
    return true;
}

/**
 * @brief Read one conversion per channel (single-channel scan).
 *        Clears CHSELR each time for single channel selection.
 */
static void adc_read_values(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    for (adc_channel_id_t ch = 0; ch < ADC_CHANNEL_COUNT; ++ch)
    {
        /* Make sure ADC is disabled before touching CHSELR */
        if ((hadc.Instance->CR & ADC_CR_ADEN) != 0U) {
            __HAL_ADC_DISABLE(&hadc);
        }

        /* Clear any previously selected channels (force single-channel) */
        hadc.Instance->CHSELR = 0;

        /* Configure the one channel we want now */
        memset(&sConfig, 0, sizeof(sConfig));
        sConfig.Channel = channels[ch];
        #if defined(ADC_REGULAR_RANK_1)
          sConfig.Rank = ADC_REGULAR_RANK_1;
        #elif defined(ADC_RANK_CHANNEL_NUMBER)
          sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
        #endif

        if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
            printf("-F- ADC Config failed (ch=%d)\r\n", ch);
            continue;
        }

        if (HAL_ADC_Start(&hadc) != HAL_OK) {
            printf("-F- ADC Start failed\r\n");
            continue;
        }

        /* With EOC_SINGLE_CONV in MX_ADC_Init(), this returns per channel */
        if (HAL_ADC_PollForConversion(&hadc, 20) != HAL_OK) {
            printf("-F- ADC Poll failed (ch=%d)\r\n", ch);
            HAL_ADC_Stop(&hadc);
            continue;
        }

        adc_data[ch].raw = HAL_ADC_GetValue(&hadc);
        HAL_ADC_Stop(&hadc);
    }
}

/**
 * @brief Perform scaling/calculations for a given ADC channel.
 * @param channel Channel to scale.
 */
static void adc_preform_scaling(adc_channel_id_t channel)
{
    voltage_mv = ((uint32_t)adc_data[channel].raw * adc_ref_mv) / ADC_FS_CODE;

    switch (channel)
    {
    /* Voltage channels → mV at input after divider */
    case ADC_CHANNEL_V_SENSE_LOAD:
    case ADC_CHANNEL_V_SENSE_LINE:
        adc_data[channel].scaled = (uint32_t)(voltage_mv * VOLTAGE_DIVIDER_RATIO);
        break;

    /* Current channels → mA = (mV * 1000) / (mV/A) */
    case ADC_CHANNEL_I_TCC:
    case ADC_CHANNEL_I_NOMINAL:
        adc_data[channel].scaled = (uint32_t)((voltage_mv * 1000U) / (CURRENT_SENSE_GAIN_UV / 1000U));
        break;

    /* TMP235 external temperature: store °C×10 = (mV - 500) */
    case ADC_CHANNEL_TEMP_SENSE_EXT:
        adc_data[channel].scaled = (float)((int32_t)voltage_mv - (int32_t)TMP235_OFFSET_MV); // °C×10
        break;

    /* Internal temperature: factory cal at 30/110°C, compensate to 3.0 V using VREFINT */
    case ADC_CHANNEL_TEMP_SENSE_INTERNAL: {
        uint32_t ts_raw = adc_data[channel].raw;

        /* Map raw reading to the calibration-voltage domain (3.0 V) */
        if (adc_ref_mv == 0U) adc_ref_mv = 1U;
        // uint32_t ts_raw_at_3v = (ts_raw * 3000U) / adc_ref_mv;
        // RIGHT: normalize to the 3.0 V cal domain
        uint32_t ts_raw_at_3v = (ts_raw * adc_ref_mv + 1500U) / 3000U; // +1500 for rounding

        uint16_t ts_cal1 = *TEMP30_CAL_ADDR;   // code @ 30°C
        uint16_t ts_cal2 = *TEMP110_CAL_ADDR;  // code @ 110°C

        int32_t T_x10 = 0;
        if (ts_cal2 != ts_cal1) {
            /* T(0.1°C) = ((Ts-TS_CAL1) * (1100-300)) / (TS_CAL2-TS_CAL1) + 300 */
            T_x10 = ((int32_t)(ts_raw_at_3v - ts_cal1) * (1100 - 300))
                    / (int32_t)(ts_cal2 - ts_cal1) + 300;
        }
        adc_data[channel].scaled = (int16_t)T_x10;  // store as 0.1°C

        int16_t temp_internal_cx10 = adc_data[channel].scaled;
        LOG_ADC("-I- TEMP_INT: %d.%d°C\r\n",
                    temp_internal_cx10 / 10,
                    (temp_internal_cx10 < 0) ? -(temp_internal_cx10 % 10) : (temp_internal_cx10 % 10));

        break;
    }

    /* VREFINT: update VDDA estimate using factory cal value */
    case ADC_CHANNEL_VREF_INTERNAL: {
        uint32_t vrefint_raw = adc_data[channel].raw;
        if (vrefint_raw == 0U) vrefint_raw = 1U;
        uint16_t vrefint_cal = *VREFINT_CAL_ADDR; // raw @ 3.0 V
        adc_ref_mv = (uint32_t)(3000U * vrefint_cal) / vrefint_raw;
        // printf("-I- VREF: %u mV\r\n", adc_ref_mv);
        break;
    }

    default:
        break;
    }
}

/* ====== Main FSM ====== */

/**
 * @brief Run the ADC event state machine.
 *
 * Handles initialization, reading, scaling, and fault recovery.
 */
void adc_event_run(void)
{
    switch (adcEvent.state)
    {
    case ADCEventStateInit:
        if (adc_init_polling()) {
            Set_ADC_State(ADCEventStateIdle);
        } else {
            Set_ADC_State(ADCEventStateFault);
        }
        break;

    case ADCEventStateDisabled:
        /* no-op */
        break;

    case ADCEventStateIdle:
        Set_ADC_State(ADCEventStateNormal);
        break;

    case ADCEventStateNormal:
        Set_ADC_State(ADCEventStateReadValues);
        break;

    case ADCEventStateReadValues:
        adc_read_values();
        Set_ADC_State(ADCEventStatePerformCalculations);
        break;

    case ADCEventStatePerformCalculations: {
        /* 1) Update VDDA first (VREFINT) */
        adc_preform_scaling(ADC_CHANNEL_VREF_INTERNAL);

        /* 2) Scale all other channels using the refreshed adc_ref_mv */
        for (adc_channel_id_t ch = 0; ch < ADC_CHANNEL_COUNT; ++ch) {
            if (ch == ADC_CHANNEL_VREF_INTERNAL) continue;
            adc_preform_scaling(ch);
        }
        Set_ADC_State(ADCEventStateIdle);
        break;
    }

    case ADCEventStateFault:
        /* HAL-only: gracefully stop and disable */
        HAL_ADC_Stop(&hadc);
        __HAL_ADC_DISABLE(&hadc);
        Set_ADC_State(ADCEventStateInit);
        break;

    case ADCEventStateWaitingForStateChange:
        /* no-op */
        break;

    default:
        break;
    }
}