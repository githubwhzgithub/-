#include "bsp_usart.h"
#include "yb_protocol.h"
#include "stdio.h"



//USART1 ---- 与PC通信，输出调试信息
void USART1_init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启接收中断       
	USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_Cmd(USART1, ENABLE);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/**
 * @Brief: UART1发送数据
 * @Note: 
 * @Parm: ch:待发送的数据 
 * @Retval: 
 */
void USART1_Send_U8(uint8_t ch)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART1, ch);
}

/**
 * @Brief: UART1发送数据
 * @Note: 
 * @Parm: BufferPtr:待发送的数据  Length:数据长度
 * @Retval: 
 */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		USART1_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}

//串口中断服务函数
void USART1_IRQHandler(void)
{
	uint8_t Rx1_Temp = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Rx1_Temp = USART_ReceiveData(USART1);
		USART1_Send_U8(Rx1_Temp);
	}
}





//USART2
void USART2_init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);  
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //开启接收中断       
	//USART_ClearFlag(USART2,USART_FLAG_TC);
	USART_Cmd(USART2, ENABLE);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}


//发送一个字符
void USART2_Send_U8(uint8_t ch)
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART2, ch);
}

//发送一个字符串
/**
 * @Brief: UsART2发送数据
 * @Note: 
 * @Parm: BufferPtr:待发送的数据  Length:数据长度
 * @Retval: 
 */
void USART2_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		USART2_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}

//串口中断服务函数
void USART2_IRQHandler(void)
{
	uint8_t Rx2_Temp;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		Rx2_Temp = USART_ReceiveData(USART2);
		Pto_Data_Receive(Rx2_Temp);
		// USART2_Send_U8(Rx2_Temp);
	}
}


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////


///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口 */
	USART_SendData(USART1, (uint8_t)ch);

	/* 等待发送完毕 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
	/* 等待串口输入数据 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
		;
	return (int)USART_ReceiveData(USART1);
}
