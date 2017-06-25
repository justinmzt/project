#include "led.h"
#include "beep.h" 
#include "lcd.h"
#include "key.h"
#include "rs485.h"
#include "usart1.h"

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init();       //初始化延时函数
	LED_Init();					//初始化LED 
	BEEP_Init();        //初始化蜂鸣器
 	LCD_Init();					//LCD初始化 
	KEY_Init(); 				//按键初始化  
	RS485_Init(9600);		//初始化RS485串口2
	uart1_init(9600);	    //串口初始化波特率为9600
	uart3_init(9600);	    //串口初始化波特率为9600
	uart6_init(9600);
 	BRUSH_COLOR=RED;    //设置字体为红色 
	while(1)
	{
		MODBUS_send();
		key_scan(0);
//		MODBUS_load(2,1,16); //发送读取信息（串口号,地址,数据个数）
		if(keyup_data==KEY0_DATA) test();
	}
}

