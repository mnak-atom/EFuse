// #ifndef INC_ADC_EVENT_H_
// #define INC_ADC_EVENT_H_

// #include <stdint.h>
// #include <stdbool.h>

// /* -------------------- ADC Channel Indexes -------------------- */
// typedef enum {
//     ADC_CHANNEL_V_SENSE_LOAD = 0,
//     ADC_CHANNEL_V_SENSE_LINE,
//     ADC_CHANNEL_I_TCC,
//     ADC_CHANNEL_I_NOMINAL,
//     ADC_CHANNEL_TEMP_SENSE_EXT,
//     ADC_CHANNEL_TEMP_SENSE_INTERNAL,
//     ADC_CHANNEL_INTERNAL_VREFINT,
//     ADC_CHANNEL_COUNT
// } adc_channel_id_t;

// /* -------------------- (Optional) State enum kept for logs -------------------- */
// typedef enum {
//     ADCEventStateInit,
//     ADCEventStateDisabled,
//     ADCEventStateIdle,
//     ADCEventStateNormal,
//     ADCEventStateReadValues,
//     ADCEventStatePerformCalculations,
//     ADCEventStateFault,
//     ADCEventStateWaitingForStateChange
// } ADC_EventState_t;

// /* -------------------- ADC Reading Structure -------------------- */
// typedef struct {
//     uint16_t raw;     /* raw 12-bit sample */
//     int16_t  scaled;  /* engineering units (mV, mA, °C depending on channel) */
// } adc_reading_t;

// /* -------------------- Globals -------------------- */
// extern adc_reading_t adc_data[ADC_CHANNEL_COUNT];

// /* -------------------- API -------------------- */
// /** Calibrate ADC (once). Returns true on success. */
// bool Init_Adc_Event(void);

// /** One-shot POLLING read of all channels + scaling + (minimal) print. */
// void StartAdcEvent(void);

// /** Scaling helper (exposed for reuse if needed). vref_mv typically 3300. */
// // void Perform_Adc_Scaling(adc_channel_id_t channel, uint32_t vref_mv);
// void Perform_Adc_Scaling(adc_channel_id_t channel);

// /** Minimal debugging output of scaled values. */
// void UART2_Print_ADC_Values(void);

// #endif /* INC_ADC_EVENT_H_ */

#ifndef INC_ADC_EVENT_H_
#define INC_ADC_EVENT_H_

#include <stdint.h>
#include <stdbool.h>

#define ADC_CHANNEL_COUNT 7

// -------------------- ADC Channel Indexes --------------------
typedef enum
{
    ADC_CHANNEL_V_SENSE_LOAD = 0,
    ADC_CHANNEL_V_SENSE_LINE,
    ADC_CHANNEL_TEMP_SENSE_EXT,
    ADC_CHANNEL_I_TCC,
    ADC_CHANNEL_I_NOMINAL,
    ADC_CHANNEL_TEMP_SENSE_INTERNAL,
    ADC_CHANNEL_VREF_INTERNAL
} adc_channel_id_t;

// -------------------- ADC Reading Structure --------------------
typedef struct
{
    uint16_t raw;
    float scaled;
} adc_reading_t;



// -------------------- Global Buffers --------------------
extern adc_reading_t adc_data[ADC_CHANNEL_COUNT];

// -------------------- Function Prototypes --------------------
/**
 * @brief Inits ADC Polling
 */
bool adc_init_polling(void);

/**
 * @brief Starts ADC Polling
 */
void adc_event_run(void);

void ADC_Print_All(void);


#endif /* INC_ADC_EVENT_H_ */
