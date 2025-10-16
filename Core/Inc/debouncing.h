#ifndef DEBOUNCING_H
#define DEBOUNCING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATE_DEFAULT = 0,
    STATE_PREVIOUS,
    STATE_ACTIVE
} SwitchState;

typedef enum {
    SWITCH_OFF = 0,
    SWITCH_ON
} SwitchLevel;

/* NEW: callback type; on==1 → EFuse ON, on==0 → EFuse OFF */
typedef void (*DebounceStateCb)(uint8_t on);

typedef struct {
    SwitchState current_state;
    SwitchLevel current_level;
    SwitchLevel previous_level;
    uint32_t    press_start_time;
    uint8_t     output_state;     /* 0=OFF, 1=ON */
    /* NEW: optional callback invoked when output_state changes */
    DebounceStateCb on_change;
} DebounceSwitch;

void DebounceSwitch_Init(DebounceSwitch *sw, SwitchLevel initial_state, uint32_t current_time);
void DebounceSwitch_Update(DebounceSwitch *sw, uint32_t current_time);

/* NEW: register the EFuse state change callback */
static inline void Debounce_RegisterCallback(DebounceSwitch *sw, DebounceStateCb cb) {
    sw->on_change = cb;
}

#ifdef __cplusplus
}
#endif

#endif  // DEBOUNCING_H
