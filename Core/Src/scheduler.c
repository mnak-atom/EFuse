/**
 * @file scheduler.c
 * @author mnak_baremetal
 * @brief This file provides a simple cooperative scheduler for periodic tasks.
 * @version 0.1
 * @date 2025-09-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "adc_event.h"
#include "debouncing.h"
#include "comm.h"
#include "efuse_protection.h"
#include "state_machine.h"
#include "scheduler.h"
#include "main.h"
#include "efuse_log.h"
#include "dac_event.h"   
#include "eeprom.h"      


/* ---------------- Scheduler event infra ---------------- */
#define MAX_EVENTS 7

typedef void (*event_callback_t)(void);
typedef struct {
    event_callback_t callback;
    int interval_ms;
    int next_run_ms;
} scheduled_event_t;

static scheduled_event_t s_sched_events[MAX_EVENTS];
static int s_sched_event_count = 0;

static void scheduler_add_event(event_callback_t callback, uint32_t interval_ms) {
    if (s_sched_event_count < MAX_EVENTS) {
        s_sched_events[s_sched_event_count].callback    = callback;
        s_sched_events[s_sched_event_count].interval_ms = (int)interval_ms;
        s_sched_events[s_sched_event_count].next_run_ms = (int)interval_ms;
        s_sched_event_count++;
    }
}

static inline int now_ms(void) { return (int)HAL_GetTick(); }

static void scheduler_event_handler(int current_time) {
    for (int i = 0; i < s_sched_event_count; i++) {
        if (current_time >= s_sched_events[i].next_run_ms) {
            s_sched_events[i].callback();
            s_sched_events[i].next_run_ms += s_sched_events[i].interval_ms;
        }
    }
}

/* ---------------- EFuse & I/O ownership ---------------- */
/* Hardware ON/OFF latch (maps to CLOSED/OPEN) */
static uint8_t  s_efuse_on     = 0;        // 0=OFF/open, 1=ON/closed
static uint32_t s_last_adc_ms  = 0;        // adc cadence
static DebounceSwitch s_switch;

/* State exposed to the State Machine */
static volatile efuse_state_t s_efuse_state = EFUSE_STATE_START;

/* Intent aggregator for SM */
static volatile efuse_comm_command_t s_desired_cmd = EFUSE_COMMAND_OPEN; /* default: OFF/open at boot */
static volatile uint8_t              s_trip_flag   = 0;

/* printf via USART2 (optional; __io_putchar is already in main.c) */
extern UART_HandleTypeDef huart2;

/* Active (in-use) limits and factory defaults */                         // #ADDED
static efuse_prot_limits_t s_active_lim = {
    .v_load_min_mv = 100000,   /* 100 V */ 
    .v_load_max_mv = 800000,   /* 800 V */ 
    .i_tcc_max_ma  = 30000,     /* 30 A  */
    .i_nom_max_ma  = 30000,    /* 30 A  */
    .t_ext_min_c   = -10, 
    .t_ext_max_c   = 85
};                                                                        
static const efuse_prot_limits_t s_factory_lim = {                        
    100000, 800000, 30000, 30000, -10, 85                                 
};                                                                        
static uint8_t s_limits_initialized = 0u;                                  

/* Hardware side-effect when SM decides a new state */
static void Apply_EFuse_State(uint8_t on)
{
    s_efuse_on = on ? 1U : 0U;

    /* Example output: single LED pin; map to your gate driver/pins */
    HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin,
                      s_efuse_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* Broadcast human-readable state */
    Comm_Send_Efuse_State(s_efuse_on);
    // printf("EFuse: %s\r\n", s_efuse_on ? "ON" : "OFF");
}

/* ---------------- Debounce callback → desired intent ---------------- */
static void on_switch_change(uint8_t on)
{
    /* ON means request CLOSED; OFF means request OPEN */
    s_desired_cmd = on ? EFUSE_COMMAND_CLOSED : EFUSE_COMMAND_OPEN;
}

/* ---------------- Periodic events ---------------- */

/* 1) Debounced switch (every 5ms) */
static void event_switch(void)
{
    DebounceSwitch_Update(&s_switch, HAL_GetTick());
}

/* 2) External commands (checked every 10ms) → desired intent */
static void event_comms(void)
{
    // Comm_BeginRx();
    // Process any pending UART operations
    // Comm_ProcessPending();

    efuse_comm_command_t cmd = Comm_Get_Received_Command();
    // Comm_Debug_Dump();   /* <— prints heartbeat, raw bytes, and frame summaries */

    /* NEW: drain/echo any raw bytes in test mode */
    // Comm_Test_Poll();


    if (cmd == EFUSE_COMMAND_OPEN || cmd == EFUSE_COMMAND_CLOSED) {
        Comm_Logf("CMD: EFUSE %s", cmd == EFUSE_COMMAND_CLOSED ? "CLOSED" : "OPEN");
        s_desired_cmd = cmd;
    }
    /* No direct hardware action here; SM arbitrates final state */
}

/* 3) ADC sampling + print (every 200ms), ignoring protection for now */
static void event_adc(void)
{
    uint32_t adc_tick = HAL_GetTick();

    if ((adc_tick - s_last_adc_ms) >= 200U)
    {
        s_last_adc_ms = adc_tick;

        /* Single owner of ADC sampling */
        adc_event_run();
        
        #if Test_Protect
        /* Evaluate protection on fresh snapshot */
        /* after adc_event_run(); */
        if (s_limits_initialized == 0u) {                                         
            s_active_lim = s_factory_lim;                                         
            s_limits_initialized = 1u;                                            
        }

        /* Pull a pending update from comms (non-blocking, one-shot) */            
        comm_prot_limits_t req;                                                   
        if (Comm_TryGetPendingProtUpdate(&req)) {                                 
            extern volatile efuse_state_t s_efuse_state; /* already in scheduler */ 
            if (s_efuse_state == EFUSE_STATE_OPEN) {                               //   gate on OPEN
                /* apply new limits while open */                                   
                s_active_lim.v_load_min_mv = req.v_load_min_mv;                    
                s_active_lim.v_load_max_mv = req.v_load_max_mv;                    
                s_active_lim.i_nom_max_ma  = req.i_nom_max_ma;                     
                s_active_lim.t_ext_min_c   = req.t_ext_min_c;                      
                s_active_lim.t_ext_max_c   = req.t_ext_max_c;                      
                s_active_lim.i_tcc_max_ma  = req.i_tcc_max_ma;                     
                /* (flag is already cleared by Comm_TryGetPendingProtUpdate) */     
            } else {
                /* EFuse is CLOSED → do nothing; update remains consumed.
                If you prefer to retry later, store req back into a local 'pending' and keep a flag. */
            }
        }


        // static const efuse_prot_limits_t lim = {
        //     .v_load_min_mv = 100000,  // 100 V
        //     .v_load_min_mv = 0,  // 0 V
        //     .v_load_max_mv = 800000,  // 800 V
        //     .i_nom_max_ma  = 30000,   // 30 A
        //     .i_tcc_max_ma  = 30000,   // 30 A
        //     .t_ext_min_c   = -10,
        //     .t_ext_max_c   = 85
        // };

        efuse_prot_result_t res;
        extern adc_reading_t adc_data[ADC_CHANNEL_COUNT]; // from adc_event.c
        // EfuseProtection_Eval(adc_data, &lim, &res);
        EfuseProtection_Eval(adc_data, &s_active_lim, &res);

        /* Latch trip; SM will move to FAULT and force OPEN */
        if (res.trip) {
            s_trip_flag   = 1U;
            s_desired_cmd = EFUSE_COMMAND_OPEN;  // ensure OPEN intent too
            printf("[PROT] Trip detected (reason=%d)\r\n", (int)res.reason);
        }
        #endif
        /* Print latest snapshot */
        // ADC_Print_All();
        /* Publish metrology as a frame (0xF0) */
        extern adc_reading_t adc_data[ADC_CHANNEL_COUNT];
        Comm_Send_MetrologyOut(
            s_efuse_on,
            (uint32_t)adc_data[ADC_CHANNEL_V_SENSE_LINE].scaled,
            (uint32_t)adc_data[ADC_CHANNEL_V_SENSE_LOAD].scaled,
            (uint32_t)adc_data[ADC_CHANNEL_I_TCC].scaled,
            (uint32_t)adc_data[ADC_CHANNEL_I_NOMINAL].scaled,
            (uint32_t)adc_data[ADC_CHANNEL_TEMP_SENSE_EXT].scaled
        );
        // Comm_Send_MetrologyOut(
        //     s_efuse_on,
        //     (uint32_t)adc_data[ADC_CHANNEL_V_SENSE_LINE].raw,
        //     (uint32_t)adc_data[ADC_CHANNEL_V_SENSE_LOAD].raw,
        //     (uint32_t)adc_data[ADC_CHANNEL_I_TCC].raw,
        //     (uint32_t)adc_data[ADC_CHANNEL_I_NOMINAL].raw,
        //     (uint32_t)adc_data[ADC_CHANNEL_TEMP_SENSE_EXT].raw
        // );
    }
}



/* 4) State machine tick (every 10ms): feed intent & trip, then apply hardware if state changes */
static void event_state_machine(void)
{
    /* Feed inputs to SM */
    extern void EFuse_SM_SetDesired(efuse_comm_command_t cmd);
    extern void EFuse_SM_SetTrip(uint8_t trip);
    EFuse_SM_SetDesired(s_desired_cmd);
    EFuse_SM_SetTrip(s_trip_flag);

    efuse_state_t before = s_efuse_state;
    EFuse_StateMachine_Run(&s_efuse_state);

    /* Apply hardware change ONCE at edge */
    if (s_efuse_state != before) {
        switch (s_efuse_state) {
        case EFUSE_STATE_CLOSED:
            Apply_EFuse_State(1);  /* ON */
            break;
        case EFUSE_STATE_OPEN:
            Apply_EFuse_State(0);  /* OFF */
            break;
        case EFUSE_STATE_FAULT:
            Apply_EFuse_State(0);  /* conservative: OFF on fault */
            break;
        default:
            /* INIT/START/DISABLED: no immediate hardware action */
            break;
        }
    }

    /* If we’re OPEN and protection was tripped earlier, allow clear when reopened */
    if (s_efuse_state == EFUSE_STATE_OPEN) {
        s_trip_flag = 0; /* clear latch once we’re safely open */
    }
}

/* 5) Dac output (every 50ms): Provide certain threshold voltage */
static void event_dac(void)          // #CHANGED
{
    dac_event_run();
}

/* 6) EEPROM Service (every 500ms): Write neccesary log or safety program */

static void event_eeprom(void)       // #CHANGED
{
    EE_Poll();
    EEPROM_WriteSetpoint((uint16_t)EE_ID_EFUSE_STATE, (uint32_t)s_efuse_on);
    EEPROM_WriteSetpoint((uint16_t)EE_ID_VLOAD_MIN_mV, (uint32_t)s_active_lim.v_load_min_mv);
    EEPROM_WriteSetpoint((uint16_t)EE_ID_VLOAD_MAX_mV, (uint32_t)s_active_lim.v_load_max_mv);
    EEPROM_WriteSetpoint((uint16_t)EE_ID_I_NOM_MAX_mA, (uint32_t)s_active_lim.i_nom_max_ma);
    EEPROM_WriteSetpoint((uint16_t)EE_ID_T_EXT_MAX_C, (uint32_t)s_active_lim.t_ext_max_c);
    // EEPROM_WriteSetpoint(EE_ID_DAC_THRESHOLD_mV, <value>);
    // EEPROM_WriteSetpoint(EE_ID_DEBOUNCE_HOLD_ms, <value>);
}

/* 7) Optional diagnostics (every 1s) */
static void event_diag(void)
{
    /* e.g., HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); */
    Comm_Test_TxHello();
}

/* ---------------- Entry ---------------- */

/* ---------------- System Init (moved from main) ---------------- */
static void scheduler_system_init(void)   
{
    /* Inputs */
    DebounceSwitch_Init(&s_switch, SWITCH_OFF, HAL_GetTick());
    Debounce_RegisterCallback(&s_switch, on_switch_change);

    /* Arm comms RX (interrupt-driven) */
    // extern void Comm_BeginRx(void);
    Comm_BeginRx();

    
    /* Enable simple UART test (echo + hello) */
    Comm_Test_Enable(0u);

    /* Default EFuse OFF/Open at boot */
    Apply_EFuse_State(0);
    s_efuse_state = EFUSE_STATE_START;  /* Let SM follow START→INIT path */
    s_desired_cmd = EFUSE_COMMAND_OPEN; /* default intent = OPEN */

    /* Low-level module inits */
    adc_init_polling();      
    dac_init();             
    EEPROM_Init();           
}

/* ---------------- Run ---------------- */
int32_t scheduler_run(void)
{
    printf("scheduler start\r\n");

    /* Centralized init */   // #CHANGED
    scheduler_system_init();

;

    /* Register events */
    scheduler_add_event(event_switch,         5);    // debounce
    scheduler_add_event(event_comms,         10);    // mailbox
    scheduler_add_event(event_state_machine, 10);    // SM tick
    scheduler_add_event(event_adc,          200);    // ADC + metrology
    scheduler_add_event(event_dac,           50);    // DAC update
    scheduler_add_event(event_eeprom,       500);    // EEPROM poll
    // scheduler_add_event(event_diag,        1000);   // optional

    /* Run loop */
    int current_time = 0;
    int last_tick    = now_ms();

    while (1) {
        int n = now_ms();
        current_time += (n - last_tick);
        last_tick = n;

        scheduler_event_handler(current_time);
        HAL_Delay(5);   // small slice to avoid busy looping
    }
}
