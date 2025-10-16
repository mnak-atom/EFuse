// /*============================== comm.h ==============================*/
#ifndef INC_COMM_H_
#define INC_COMM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdarg.h>

/* --------- High-level command mailbox (unchanged API) --------- */
typedef enum {
    EFUSE_COMMAND_NONE = 0,
    EFUSE_COMMAND_OPEN,     /* request EFuse OFF  */
    EFUSE_COMMAND_CLOSED    /* request EFuse ON   */
} efuse_comm_command_t;

/* Threshold “mailbox” for the scheduler (units = same as efuse_prot_limits_t) */
typedef struct {                                                     
    int32_t v_load_min_mv;                                           
    int32_t v_load_max_mv;    
    int32_t i_tcc_max_ma;                                       
    int32_t i_nom_max_ma;                                           
    int32_t t_ext_min_c;                                             
    int32_t t_ext_max_c;                                                                                            
} comm_prot_limits_t;     

/* TX convenience (existing names preserved) */
void Comm_Send_Efuse_Command(efuse_comm_command_t command_code);
void Comm_Send_Efuse_State(uint8_t state_on);

/* RX mailbox (existing names preserved) */
void Comm_Set_Received_Command(efuse_comm_command_t command_code);
efuse_comm_command_t Comm_Get_Received_Command(void);

/* Arm RX (byte-by-byte IT on USART2) */
void Comm_BeginRx(void);

/* --------- Framed UART (per Communications API) ---------------
   Frame: 0xAA | Type(1) | Len(1) | Payload(n) | CRC16(2 LE) | 0xBB
   CRC16: CCITT-FALSE (poly 0x1021, init 0xFFFF, no xorout), LE in frame.
--------------------------------------------------------------- */
#define COMM_START_BYTE            (0xAA)
#define COMM_END_BYTE              (0xBBu)

/* Core message types (extend as needed) */
#define COMM_MSG_SET_STATE         (0x01u)  /* payload: state(1B: 0=open,1=closed) */
#define COMM_MSG_GET_STATE         (0x71u)  /* request: len=0; reply payload: state(1B) */
#define COMM_MSG_METROLOGY_OUT     (0xF0u)  /* payload: state(1) + Vline(u32) + Vload(u32) + I_tcc(u32) + I_nom(u32) + Ext_Temp(u32)*/
#define COMM_MSG_STATE_OUT         (0xF1u)  /* payload: state(1) */
#define COMM_MSG_LOG_TEXT          (0xF8u)  /* payload: UTF-8 text bytes */
#define COMM_MSG_PROT_UPDATE       (0x91u)  //  host→device thresholds

/* EEPROM framed command:
   Send ASCII control lines in a frame so eeprom.c can parse them unchanged */
#define COMM_MSG_EE_TEXT           (0xA0u)  /* payload: ASCII like 'EE:CLEAR' or 'EE:READ:0000:0040' */

/* Generic frame TX */
void Comm_Frame_Send(uint8_t message_type, const uint8_t *payload_ptr, uint8_t payload_len);

/* Typed frame TX helpers */
void Comm_Send_StateOut(uint8_t state_on);
void Comm_Send_MetrologyOut(uint8_t state_on, uint32_t vline_mv, uint32_t vload_mv, uint32_t t_current_ma, uint32_t n_current_ma, uint32_t ext_temp);

/* Framed logger for LOG_* macros (optional) */
void Comm_Logf(const char *format_str, ...);

/* Optional RX dequeue (single-slot) for modules that want raw frames */
int  Comm_Frame_TryDequeue(uint8_t *message_type_ptr, uint8_t *payload_out_ptr, uint8_t *payload_len_ptr);

/* --------- Protection limits update (new) ---------------
   Payload: vload_min_mv(4) | vload_max_mv(4) | i_tcc_max_ma(4) | inom_max_ma(4) | 
            text_min_c(4) | text_max_c(4)
   All multi-byte values are little-endian.
Set by comms when a PROT_UPDATE frame arrives; read by scheduler
*/  
int  Comm_TryGetPendingProtUpdate(comm_prot_limits_t *out);

/* --------- UART TX/RX basic tester (new) ----------------
   Enable simple loopback/echo without interfering with framed protocol:
   - Comm_Test_TxHello(): sends "Hello, World!\r\n"
   - Comm_Test_Poll(): echoes back any raw bytes seen by the ISR (no framing)
   These use the existing debug byte queue that taps all RX bytes.
*/
void Comm_Test_Enable(uint8_t enable);
void Comm_Test_TxHello(void);
void Comm_Test_Poll(void);

void Comm_ProcessPending(void);

/*
Debug dump called from scheduler after reading a command
*/
void Comm_Debug_Dump(void);
#ifdef __cplusplus
}
#endif

#endif /* INC_COMM_H_ */
/*============================== comm.h ==============================*/