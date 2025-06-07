#ifndef _YB_PROTOCOL_H_
#define _YB_PROTOCOL_H_
#include "stdint.h"


#define PTO_BUF_LEN_MAX           (50)


#define PTO_HEAD                  (0x24)
#define PTO_TAIL                  (0x23)


void Pto_Data_Receive(uint8_t Rx_Temp);
void Pto_Data_Parse(uint8_t *data_buf, uint8_t num);
void Pto_Clear_CMD_Flag(void);
void Pto_Loop(void);



#endif
