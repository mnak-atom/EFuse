/* build_mode.h */
/* ========================================================================== */
/* Purpose:                                                                   */
/*  - Global switch: Nucleo vs Proto                                      */
/*  - Central includes so main.c needn't include every header                 */
/*  - Build-time config toggles (test LED/switch)                             */
/*  - Uniform task init context shared by all tasks                           */
/* ========================================================================== */
#ifndef BUILD_MODE_H /* include guard start */
#define BUILD_MODE_H /* include guard define */

/* ---------------- Build Mode (choose one) ---------------- */ // section: mode
#define BM_NUCLEO	1                           // define nucleo
#define BM_PROTO	0 	                        // define Proto Developed

/* ----------------- Protection Setup------------------------------*/
#define Test_Protect 0                   /* 1=use, 0=ignore    */

/* ---------------- Common includes ---------------- */
#include <stdio.h>                            /* printf              */
#include <stdint.h>                           /* fixed-size types    */
#include <stddef.h>                           /* size_t, NULL        */
#include <stdbool.h>                          /* bool type           */
#include <string.h>                           /* memcpy, memset      */

#include "main.h"                             /* HAL handles, pins   */

/* Event headers gathered once here to simplify main.c includes. */

#include "adc_event.h"                        /* ADC helpers         */
#include "comm.h"                             /* framed comm         */
#include "debouncing.h"                       /* switch debounce     */
#include "dac_event.h"                        /* DAC helpers         */
#include "efuse_log.h"                        /* LOG_* macros        */
#include "efuse_protection.h"                 /* protection logic    */
#include "eeprom.h"                           /* eeprom helpers      */
#include "state_machine.h"                    /* EFuse SM            */ 
#include "scheduler.h"                        /* scheduler           */  
 

/* Build-time config toggles (adjust to board mapping). */
#define Comms_USART        1                   /* 1=use, 0=ignore    */
#define Comms_I2C          0                   /* 1=use, 0=ignore    */

/* I2C 7-bit address for host (only if Comms_I2C==1) */
#ifndef COMM_I2C_ADDR_7B
#define COMM_I2C_ADDR_7B   (0x28u)                       
#endif

// #ifdef BM_NUCLEO
//     #define Test_Protect 0                   /* 1=use, 0=ignore    */
// #elif   BM_PROTO
//     #define Test_Protect 1                   /* 1=use, 0=ignore    */
// #endif

#endif /* BUILD_MODE_H */                     /* include guard end   */