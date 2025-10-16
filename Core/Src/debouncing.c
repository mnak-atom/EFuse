#include "debouncing.h"
#include "main.h"
#include "stm32l0xx_hal.h" // for HAL_GetTick, GPIO read
#include "build_mode.h"             /* ADDED: board selection flags */ // include flags

#define BTN_HOLD_MS (3000U)  /* 3s hold for Nucleo button */

/* USER button on PC13 → active LOW */
static SwitchLevel Read_User_Button(void)
{
    #if defined(BM_NUCLEO)  
        GPIO_PinState state = HAL_GPIO_ReadPin(TEST_Switch_Port, TEST_Switch_Pin);
        return (state == GPIO_PIN_RESET) ? SWITCH_ON : SWITCH_OFF;
    #elif defined(BM_PROTO)
        GPIO_PinState Open_State = HAL_GPIO_ReadPin(HMI_OPEN_GPIO_Port, HMI_OPEN_Pin);
        GPIO_PinState Closed_State = HAL_GPIO_ReadPin(HMI_CLOSED_GPIO_Port, HMI_CLOSED_Pin);
        if (Open_State == GPIO_PIN_RESET) {
            return SWITCH_OFF;                // OPEN active
        } else if (Closed_State == GPIO_PIN_RESET) {
            return SWITCH_ON;                 // CLOSED active
        } else {
        return SWITCH_OFF;                    // Default to OFF if neither is active
        }
    #endif  
}

void DebounceSwitch_Init(DebounceSwitch *sw, SwitchLevel initial_state, uint32_t current_time)
{
    sw->current_state   = STATE_DEFAULT;
    sw->current_level   = initial_state;
    sw->previous_level  = initial_state;
    sw->press_start_time= current_time;
    sw->output_state    = 0;      /* start EFuse OFF */
    sw->on_change       = 0;      /* no callback yet */
}

void DebounceSwitch_Update(DebounceSwitch *sw, uint32_t current_time)
{
    SwitchLevel physical_state = Read_User_Button();

    switch (sw->current_state)
    {
    case STATE_DEFAULT:
        if (physical_state == SWITCH_ON)
        {
            sw->press_start_time = current_time;
            sw->previous_level   = sw->current_level;
            sw->current_level    = SWITCH_ON;
            sw->current_state    = STATE_PREVIOUS;
        }
        break;

    case STATE_PREVIOUS:
        if (physical_state == SWITCH_ON)
        {   
            #if defined(BM_NUCLEO) 
            if ((current_time - sw->press_start_time) >= BTN_HOLD_MS)
            {
                /* 3s hold satisfied */
                sw->current_state = STATE_ACTIVE;
            }
            #elif defined(BM_PROTO)
                sw->current_state = STATE_ACTIVE;
            #endif  
        }
        else
        {
            /* bounced/released early → reset */
            sw->current_state = STATE_DEFAULT;
        }
        break;

    case STATE_ACTIVE:
        if (physical_state == SWITCH_OFF)
        {
            /* Toggle EFuse only when released after valid hold */
            sw->output_state ^= 1U;

            /* NEW: immediately inform EFuse controller */
            if (sw->on_change) {
                sw->on_change(sw->output_state);   /* 1=ON, 0=OFF */
            }

            sw->current_state = STATE_DEFAULT;
        }
        break;

    default:
        sw->current_state = STATE_DEFAULT;
        break;
    }
}
