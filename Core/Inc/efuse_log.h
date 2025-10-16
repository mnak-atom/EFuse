#ifndef EFUSE_LOG_H
#define EFUSE_LOG_H

/* --------- Compile-time global ON/OFF switches (edit here) --------- */
/* Master switch (set 0 to silence everything at once) */
#define LOG_GLOBAL_ENABLE   1

/* Per-module switches */
#define LOG_ADC_ENABLE      0   /* <- for now we only print ADC internal temp */
#define LOG_SM_ENABLE       0   /* state machine */
#define LOG_COMM_ENABLE     1   /* comms */
#define LOG_SCHED_ENABLE    0   /* scheduler */
#define LOG_PROT_ENABLE     0   /* protection */
#define LOG_DAC_ENABLE      0   /* 1=enable DAC logs, 0=disable */


/* ---------------- Do not edit below this line ---------------- */
#include <stdio.h>

#if LOG_GLOBAL_ENABLE && LOG_ADC_ENABLE
  #define LOG_ADC(...)     do{ Comm_Logf(__VA_ARGS__); }while(0)
#else
  #define LOG_ADC(...)     do{}while(0)
#endif

#if LOG_GLOBAL_ENABLE && LOG_SM_ENABLE
  #define LOG_SM(...)      do{ Comm_Logf(__VA_ARGS__); }while(0)
#else
  #define LOG_SM(...)      do{}while(0)
#endif

#if LOG_GLOBAL_ENABLE && LOG_COMM_ENABLE
  #define LOG_COMM(...)    do{ Comm_Logf(__VA_ARGS__); }while(0)
#else
  #define LOG_COMM(...)    do{}while(0)
#endif

#if LOG_GLOBAL_ENABLE && LOG_SCHED_ENABLE
  #define LOG_SCHED(...)   do{ Comm_Logf(__VA_ARGS__); }while(0)
#else
  #define LOG_SCHED(...)   do{}while(0)
#endif

#if LOG_GLOBAL_ENABLE && LOG_PROT_ENABLE
  #define LOG_PROT(...)    do{ Comm_Logf(__VA_ARGS__); }while(0)
#else
  #define LOG_PROT(...)    do{}while(0)
#endif
#if LOG_GLOBAL_ENABLE && LOG_DAC_ENABLE
  #define LOG_DAC(...)     do{ Comm_Logf(__VA_ARGS__); }while(0)
#else
  #define LOG_DAC(...)     do{}while(0)
#endif


#endif /* EFUSE_LOG_H */
