#include "bsp_common.h"
#include "delay.h"
#include "bsp_usart.h"
#include "yb_protocol.h"

int main()
{
	SystemInit();
	
	delay_init();

    USART1_init(115200);
	USART2_init(115200);

	Pto_Loop();
}

