/**
 * @file state_machine.c
 * @author mnak
 * @brief This file implements the EFuse state machine. 
 * @version 0.1
 * @date 2025-09-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "state_machine.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>
#include "efuse_log.h"

/* ---- External I/O (printf over USART2) ---- */
extern UART_HandleTypeDef huart2;

// #define LOG(s) HAL_UART_Transmit(&huart2, (uint8_t *)(s), (uint16_t)strlen(s), 100)
// #undef LOG
#define LOG(s)  LOG_SM("%s", (s))   // reuse existing LOG("...") sites without editing lines

/* ---- PWM (TIM2 CH1) for gate drive ---- */
extern TIM_HandleTypeDef htim2;                                      // TIM2 handle
#define pwm_channel TIM_CHANNEL_1                                   // channel
#define pwm_duty_on  1000                                       // ~100% duty
#define pwm_duty_off 0                                         // 0% duty

/*-----RGB Defines----*/
#define RGB_OFF   (0u)
#define RGB_RED   (1u)
#define RGB_GREEN (2u)
#define RGB_BLUE  (3u)

/* --------------------------------------------------------------------------
   Intent inputs (owned by scheduler) → set via the two setters below
   - desired_cmd: external intent (button/comm). OPEN = OFF, CLOSED = ON
   - trip_flag  : protection result (1 = trip → force FAULT)
   -------------------------------------------------------------------------- */
static volatile efuse_comm_command_t g_desired_cmd = EFUSE_COMMAND_NONE;
static volatile uint8_t              g_trip_flag   = 0;

/* Simple setters the scheduler calls each tick before running SM */
void EFuse_SM_SetDesired(efuse_comm_command_t cmd) { g_desired_cmd = cmd; }
void EFuse_SM_SetTrip(uint8_t trip)                 { g_trip_flag   = trip; }

/* Optional fault bookkeeping (kept for your logs) */
static efuse_fault_group_t current_fault_group = FAULT_GROUP_NONE;
static efuse_fault_code_t  current_fault_code  = FAULT_NONE;

void EFuse_TriggerFault(efuse_fault_group_t group, efuse_fault_code_t code)
{
    current_fault_group = group;
    current_fault_code  = code;
}

/* Fault logger */
static inline void Handle_Fault_Log(void)
{
    static const char *group_names[] = { "NONE", "INIT", "CLOSED", "OPEN" };
    static const char *fault_names[] = {
        "NONE",
        "ADC_INIT", "COMM_INIT", "RT_INIT",
        "GATE_DRIVE", "COMM_SWITCH", "LINE_LOAD",
        "OPEN_UNKNOWN"
    };

    LOG("[FAULT] Group: ");
    LOG(group_names[current_fault_group]);
    LOG(" | Code: ");
    LOG(fault_names[current_fault_code]);
    LOG("\r\n");
}

/* ==========================================================================
   PURE STATE MACHINE (no HAL/ADC/UART triggers here)
   States: START → INIT → (OPEN | CLOSED) → FAULT
   - OPEN   (hardware OFF)
   - CLOSED (hardware ON)
   - Any trip → FAULT
   - From FAULT: require trip cleared; go to INIT when a new command asks to CLOSE
   ========================================================================== */
void EFuse_StateMachine_Run(volatile efuse_state_t *p_state)
{
    efuse_state_t state_curr = *p_state;

    switch (state_curr)
    {
    case EFUSE_STATE_START:
        LOG("State: START\r\n");
        EFuse_RGB_Set(RGB_OFF);    // All RGB OFF
        /* Bring-up completed elsewhere (main/scheduler). Move to INIT. */
        state_curr = EFUSE_STATE_INIT;
        break;

    case EFUSE_STATE_INIT:
        LOG("State: INIT\r\n");
        EFuse_RGB_Set(RGB_BLUE);
        gate_drive_enable(false);   // Ensure gate drive is OFF
        /* If protection is already tripped, go straight to FAULT */
        if (g_trip_flag) {
            EFuse_TriggerFault(FAULT_GROUP_INIT, FAULT_ADC_INIT); /* reason placeholder */
            state_curr = EFUSE_STATE_FAULT;
            break;
        }
        /* Choose initial branch from desired command:
           CLOSED -> ON; otherwise OPEN -> OFF */
        if (g_desired_cmd == EFUSE_COMMAND_CLOSED) state_curr = EFUSE_STATE_CLOSED;
        else                                       state_curr = EFUSE_STATE_OPEN;
        break;

    case EFUSE_STATE_OPEN:
        LOG("State: OPEN\r\n");
        EFuse_RGB_Set(RGB_RED);    // Red = OPEN
        gate_drive_enable(false);   // Ensure gate drive is OFF
        if (g_trip_flag) {
            EFuse_TriggerFault(FAULT_GROUP_OPEN, FAULT_OPEN_UNKNOWN);
            state_curr = EFUSE_STATE_FAULT;
            break;
        }
        /* Transition to CLOSED only when explicitly requested */
        if (g_desired_cmd == EFUSE_COMMAND_CLOSED) {
            state_curr = EFUSE_STATE_CLOSED;
        }
        break;

    case EFUSE_STATE_CLOSED:
        LOG("State: CLOSED\r\n");
        EFuse_RGB_Set(RGB_GREEN);  // Green = CLOSED
        gate_drive_enable(true);    // Enable gate drive
        if (g_trip_flag) {
            EFuse_TriggerFault(FAULT_GROUP_CLOSED, FAULT_LINE_LOAD); /* example reason */
            state_curr = EFUSE_STATE_FAULT;
            break;
        }
        /* Any OPEN request re-opens */
        if (g_desired_cmd == EFUSE_COMMAND_OPEN) {
            state_curr = EFUSE_STATE_OPEN;
        }
        break;

    case EFUSE_STATE_FAULT:
        LOG("State: FAULT\r\n");
        EFuse_RGB_Set(RGB_RED);    // RED = Opened
        gate_drive_enable(false);   // Ensure gate drive is OFF
        Handle_Fault_Log();
        /* Stay latched FAULT while trip is present.
           Clear policy: when trip is cleared AND user requests CLOSED again,
           re-enter INIT to re-check preconditions. */
        if (!g_trip_flag && g_desired_cmd == EFUSE_COMMAND_CLOSED) {
            state_curr = EFUSE_STATE_INIT;
        }
        break;

    case EFUSE_STATE_DISABLED:
        LOG("State: DISABLED\r\n");
        /* Keep disabled unless your system wants an enable path */
        break;

    default:
        /* Safety net */
        EFuse_TriggerFault(FAULT_GROUP_INIT, FAULT_NONE);
        state_curr = EFUSE_STATE_FAULT;
        break;
    }

    *p_state = state_curr;
}
/* Define color macros for readability */
#define RGB_OFF   (0u)
#define RGB_RED   (1u)
#define RGB_GREEN (2u)
#define RGB_BLUE  (3u)

/**
 * @brief  Set the RGB LED to one color at a time.
 *         Passing RGB_OFF turns all LEDs OFF.
 * @param  color  One of RGB_OFF, RGB_RED, RGB_GREEN, RGB_BLUE
 */
void EFuse_RGB_Set(uint8_t color)
{
    GPIO_PinState redState   = GPIO_PIN_RESET;
    GPIO_PinState greenState = GPIO_PIN_RESET;
    GPIO_PinState blueState  = GPIO_PIN_RESET;

    switch (color)
    {
        case RGB_RED:
            redState = GPIO_PIN_SET;
            break;

        case RGB_GREEN:
            greenState = GPIO_PIN_SET;
            break;

        case RGB_BLUE:
            blueState = GPIO_PIN_SET;
            break;

        case RGB_OFF:
        default:
            /* all remain RESET */
            break;
    }

    HAL_GPIO_WritePin(RGB_LED_RED_GPIO_Port,   RGB_LED_RED_Pin,   redState);
    HAL_GPIO_WritePin(RGB_LED_GREEN_GPIO_Port, RGB_LED_GREEN_Pin, greenState);
    HAL_GPIO_WritePin(RGB_LED_BLUE_GPIO_Port,  RGB_LED_BLUE_Pin,  blueState);
}
/**
 * @brief  Enable or disable the gate drive signal.
 *         Latches the hardware and sets PWM duty cycle.
 * @param  on  true to enable (CLOSED), false to disable (OPEN)
 */
void gate_drive_enable(bool on)                                          // ADDED
{
    HAL_GPIO_WritePin(HW_LATCH_RESET_GPIO_Port,   HW_LATCH_RESET_Pin,   GPIO_PIN_SET); // Latch reset HIGH
    HAL_Delay(30);                                                        // wait 30ms
    HAL_GPIO_WritePin(HW_LATCH_RESET_GPIO_Port,   HW_LATCH_RESET_Pin,   GPIO_PIN_RESET); // Latch reset LOW
    if (on) {                                                            // if enable
        __HAL_TIM_SET_COMPARE(&htim2, pwm_channel, pwm_duty_on);         // set duty
        HAL_TIM_PWM_Start(&htim2, pwm_channel);                          // start PWM
    } else {                                                             // disable

        __HAL_TIM_SET_COMPARE(&htim2, pwm_channel, pwm_duty_off);        // set 0%
        HAL_TIM_PWM_Stop(&htim2, pwm_channel);                           // stop PWM
    }                                                                    // end if
}      
