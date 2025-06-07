#include "yb_protocol.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"



#define PTO_FUNC_ID        9


uint8_t RxBuffer[PTO_BUF_LEN_MAX];
/* 接收数据下标 */
uint8_t RxIndex = 0;
/* 接收状态机 */
uint8_t RxFlag = 0;
/* 新命令接收标志 */
uint8_t New_CMD_flag;
/* 新命令数据长度 */
uint8_t New_CMD_length;


// 清除命令数据和相关标志
void Pto_Clear_CMD_Flag(void)
{
	for (uint8_t i = 0; i < PTO_BUF_LEN_MAX; i++)
	{
		RxBuffer[i] = 0;
	}
	New_CMD_length = 0;
	New_CMD_flag = 0;
}


// 接收数据进入协议缓存
void Pto_Data_Receive(uint8_t Rx_Temp)
{
	switch (RxFlag)
	{
	case 0:
		if (Rx_Temp == PTO_HEAD)
		{
			RxBuffer[0] = PTO_HEAD;
			RxFlag = 1;
            RxIndex = 1;
		}
		break;

	case 1:
		RxBuffer[RxIndex] = Rx_Temp;
		RxIndex++;
		if (Rx_Temp == PTO_TAIL)
		{
			New_CMD_flag = 1;
            New_CMD_length = RxIndex;
			RxFlag = 0;
			RxIndex = 0;
		}
        else if (RxIndex >= PTO_BUF_LEN_MAX)
        {
            New_CMD_flag = 0;
            New_CMD_length = 0;
			RxFlag = 0;
			RxIndex = 0;
            Pto_Clear_CMD_Flag();
        }
		break;

	default:
		break;
	}
}

// 将字符串数字转成数字。示例："12"->12
int Pto_Char_To_Int(char* data)
{
    return atoi(data);
}

/**
 * @Brief: 数据分析
 * @Note: 
 * @Parm: 传入接受到的一个数据帧和长度
 * @Retval: 
 */
void Pto_Data_Parse(uint8_t *data_buf, uint8_t num)
{
    uint8_t pto_head = data_buf[0];
	uint8_t pto_tail = data_buf[num-1];
    if (!(pto_head == PTO_HEAD && pto_tail == PTO_TAIL))
    {
	    printf("pto error:pto_head=0x%02x , pto_tail=0x%02x\n", pto_head, pto_tail);
        return;
    }
    uint8_t data_index = 1;
    uint8_t field_index[PTO_BUF_LEN_MAX] = {0};
    int i = 0;
    int values[PTO_BUF_LEN_MAX] = {0};
    for (i = 1; i < num-1; i++)
    {
        if (data_buf[i] == ',')
        {
            data_buf[i] = 0;
            field_index[data_index] = i;
            data_index++;
        }
    }
    
    for (i = 0; i < data_index; i++)
    {
        values[i] = Pto_Char_To_Int((char*)data_buf+field_index[i]+1);
    }
    
    uint8_t pto_len = values[0];
    
    if (pto_len != num)
    {
        printf("pto_len error:%d , data_len:%d\n", pto_len, num);
        return;
    }
    uint8_t pto_id = values[1];
    if (pto_id != PTO_FUNC_ID)
    {
        printf("pto_id error:%d, func_id:%d\n", pto_id, PTO_FUNC_ID);
        return;
    }
    int x = values[2];
    int y = values[3];
    int w = values[4];
    int h = values[5];
    printf("person:x:%d, y:%d, w:%d, h:%d\n", x, y, w, h);
}

// 循环处理协议内容
void Pto_Loop(void)
{
    
    while (1)
    {
        if (New_CMD_flag)
		{
			Pto_Data_Parse((uint8_t*)RxBuffer, New_CMD_length);
			Pto_Clear_CMD_Flag();
		}
    }
}

