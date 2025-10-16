#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "comm.h"
#include "adc_event.h"

// EFuse main states
typedef enum {
    EFUSE_STATE_START = 0,
    EFUSE_STATE_INIT,
    EFUSE_STATE_DISABLED,
    EFUSE_STATE_IDLE,
    EFUSE_STATE_CLOSED,
    EFUSE_STATE_OPEN,
    EFUSE_STATE_FAULT
} efuse_state_t;

// Fault Groups
typedef enum {
    FAULT_GROUP_NONE = 0,
    FAULT_GROUP_INIT,
    FAULT_GROUP_CLOSED,
    FAULT_GROUP_OPEN
} efuse_fault_group_t;

// Fault Codes (within groups)
typedef enum {
    FAULT_NONE = 0,

    // INIT Faults
    FAULT_ADC_INIT,
    FAULT_COMM_INIT,
    FAULT_RT_INIT,

    // CLOSED Faults
    FAULT_GATE_DRIVE,
    FAULT_COMM_SWITCH,
    FAULT_LINE_LOAD,

    // OPEN Faults
    FAULT_OPEN_UNKNOWN

} efuse_fault_code_t;

// Public API
// void EFuse_StateMachine_Run(void);
void EFuse_StateMachine_Run(volatile efuse_state_t *p_state);
void EFuse_TriggerFault(efuse_fault_group_t group, efuse_fault_code_t code);
void EFuse_RGB_Set(uint8_t color);
void gate_drive_enable(bool on);
#ifdef __cplusplus
}
#endif

#endif // INC_STATE_MACHINE_H_
