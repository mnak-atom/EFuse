#include "efuse_protection.h"
#include "main.h"

void EfuseProtection_Eval(const adc_reading_t *adc, const efuse_prot_limits_t *Lim,
                          efuse_prot_result_t *out)
{
    // default = healthy
    out->trip = false;
    out->reason = PROT_OK;

    // read scaled engineering units produced by adc_event
    const int32_t v_load = adc[ADC_CHANNEL_V_SENSE_LOAD].scaled;     // mV at source
    const int32_t i_tcc  = adc[ADC_CHANNEL_I_TCC].scaled;
    const int32_t i_nom  = adc[ADC_CHANNEL_I_NOMINAL].scaled;        // mA
    const int32_t t_ext  = adc[ADC_CHANNEL_TEMP_SENSE_EXT].scaled;   // °C (int)

    if (v_load > Lim->v_load_max_mv || v_load < Lim->v_load_min_mv) {
        out->trip = true; out->reason = PROT_TRIP_LINE_LOAD; return;
    }
    if (i_tcc > Lim->i_tcc_max_ma) {
        out->trip = true; out->reason = PROT_TRIP_OVERCURRENT; return;
    }
    if (i_nom > Lim->i_nom_max_ma) {
        out->trip = true; out->reason = PROT_TRIP_OVERCURRENT; return;
    }
    if (t_ext > Lim->t_ext_max_c || t_ext < Lim->t_ext_min_c) {
        out->trip = true; out->reason = PROT_TRIP_OVERTEMP; return;
    }
}
/**
 * \brief Returns true if the hardware trip input is asserted.            
 * \return bool true if asserted, else false.                             
 * \thread_safety Safe in task/superloop.                                  
 * \side_effects None.                                                     
 */
bool hw_trip_is_asserted(void)                                            
{
    GPIO_PinState State = HAL_GPIO_ReadPin(HW_TRIP_DETECT_GPIO_Port,HW_TRIP_DETECT_Pin);                  
    return (State == GPIO_PIN_RESET) ? SWITCH_ON : SWITCH_OFF;
}                                                                         
