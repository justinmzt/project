#include "led.h"
#include "beep.h" 
#include "lcd.h"
#include "key.h"
#include "rs485.h"
#include "usart1.h"

int main(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init();       //初始化延时函数
	LED_Init();                    //初始化LED
	BEEP_Init();        //初始化蜂鸣器
	LCD_Init();                    //LCD初始化
	KEY_Init();                //按键初始化
	TIM2_Init(99, 8399);    //定时器2时钟84M，分频系数8400，84M/8400=10K 所以计数100次为10ms
	RS485_Init(9600);        //初始化RS485串口2
	uart1_init(9600);        //串口初始化波特率为9600
	uart3_init(9600);        //串口初始化波特率为9600
	uart5_init(9600);        //串口初始化波特率为9600
	uart6_init(9600);
	BRUSH_COLOR = RED;    //设置字体为红色
	MODBUS_load(1, 1, 0, 16);
	while (1) {
		MODBUS_send();
		key_scan(0);
////		MODBUS_load(2,1,16); //发送读取信息（串口号,地址,数据个数）
		if (keyup_data == KEY0_DATA) {
			u8 gprs_str_test[] = {0x73, 0x74, 0x72, 0x3D, 0x02, 0x03, 0x06, 0x00, 0x24, 0x01, 0x02, 0x01, 0x05, 0x00,
								  0x11};
			GPRS_Send(gprs_str_test, 15);
		}
//			u5SendChars("123456",6);
//		  load_next();
//		}
	}
}

