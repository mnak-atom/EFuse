/** @file efuse_protection.h
 *  @brief Simple limit checks for voltages, current, and temperature.
 */
#ifndef EFUSE_PROTECTION_H
#define EFUSE_PROTECTION_H
#include <stdint.h>
#include <stdbool.h>
#include "adc_event.h"   // for adc_reading_t and channel enum

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PROT_OK = 0,
    PROT_TRIP_LINE_LOAD,     // over/under voltage
    PROT_TRIP_OVERCURRENT,   // i_nom beyond limit
    PROT_TRIP_OVERTEMP,      // external temp beyond range
} efuse_prot_trip_t;

typedef struct {
    bool                trip;       // true -> scheduler should force EFuse OFF
    efuse_prot_trip_t   reason;
} efuse_prot_result_t;

/* thresholds (integer-scaled to what adc_event produces) */
typedef struct {
    int32_t v_load_min_mv;   // e.g., 100000 mV (100 V)
    int32_t v_load_max_mv;   // e.g., 800000 mV (800 V)
    int32_t i_tcc_max_ma;    // e.g., 30000 mA
    int32_t i_nom_max_ma;    // e.g., 30000 mA
    int32_t t_ext_min_c;     // e.g., -10 C
    int32_t t_ext_max_c;     // e.g., 85 C
} efuse_prot_limits_t;

/* Evaluate protection against current ADC snapshot */
void EfuseProtection_Eval(const adc_reading_t *adc, const efuse_prot_limits_t *lim,
                          efuse_prot_result_t *out);

#ifdef __cplusplus
}
#endif
#endif /* EFUSE_PROTECTION_H */