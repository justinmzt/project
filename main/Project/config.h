#ifndef __CONFIG_H
#define __CONFIG_H
#include "stdio.h"
#include "stdbool.h"
#include "common.h"
//////////////////////////////////////////////////////////////////////////////////

/** 站号:01
* ele[1]~ele[77]=====add[1]~add[77]
*/
u16 ele[78]={0};
/** 站号:02
* ups1[1]~ups1[112]=====add[1]~add[113]
* ups1[113]~ups1[145]=====add[1001]~add[1033]
* ups1[146]~ups1[289]=====add[1101]~add[1244]
* ups1[290]~ups1[433]=====add[1501]~add[1644]
* ups1[434]~ups1[517]=====add[1901]~add[1984]
* ups1[518]~ups1[520]=====add[2301]~add[2303]
*/
u16 ups1[521]={1};
/** 站号:03
* ups2[1]~ups2[112]=====add[1]~add[113]
* ups2[113]~ups2[145]=====add[1001]~add[1033]
* ups2[146]~ups2[289]=====add[1101]~add[1244]
* ups2[290]~ups2[433]=====add[1501]~add[1644]
* ups2[434]~ups2[517]=====add[1901]~add[1984]
* ups2[518]~ups2[520]=====add[2301]~add[2303]
*/
u16 ups2[521]={0};
u16 ele2[5] = {0};
u16 ele3[2] = {0};
/*****16位CRC检验表，低位在前，高位在后*******/
//////////////*高位表*///////////////////
u8 auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;

//////////////*低位表*///////////////////
u8 auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
} ;


#define USART1_BYTE_COUNT		8  	  //定义usart1接收端接收长度
#define UART5_BYTE_COUNT		8  	  //定义uart5接收端接收长度
#define USART_REC_NUM  			200  	//定义最大接收字节数 200
#define USART_SEND_NUM  		200  	//定义最大接收字节数 200
#define LOAD_LENGTH      		21  	//定义Load队列长度

/*设备相关*/
u8 receive_str[USART1_BYTE_COUNT] = {0};//PLC-HMI接收数组
u8 use_str[USART1_BYTE_COUNT] = {0};    //接收数组临时存储数组
u8 receive_str2[USART_REC_NUM] = {0};   //设备-PLC接收数组（长度200）
//u8 receive_str3[UART5_BYTE_COUNT] = {0};//设备-SCADA接收数组
u8 receive_gprs[UART5_BYTE_COUNT] = {0};//设备-gprs接收数组
u8 Send_buf[USART_SEND_NUM] = {0}; 			//发送数据数组（长度200）

/*标志位相关*/
u8 flag_byte_count=0;             //PLC-HMI接收位数标识位 
u8 flag2_byte_count=0;            //设备-PLC接收位数标识位
//u8 flag3_byte_count=0;            //PLC-SCADA接收位数标识位
u8 gprs_byte_count=0;             //PLC-GPRS接收位数标识位
u8 uart2_byte_count=8;						//设备对应串口接收端接收长度（USART2、USART3、USART6）
u8 flag_hmi_send=0;							  //HMI发送指令结束标志位
u8 flag_scada_send=0;							//SCADA发送指令结束标志位
u8 flag_gprs=0;							      //gprs标志位
u8 gprs_queue_ready=0;            //gprs发送队列载入完成标志位
u8 flag_finish=0;								  //设备数据传入结束标志位	
u16 flag_add=1;									  //载入设备数据的地址

/****************************************************/
/******************** load相关：*********************/
/**** 将所要载入的信息的各种配置信息输入到下表中 ****/
/************** 注意数据所在位置要对齐 **************/
/************** load_uart_num: 串口号 ***************/
/************ load_func_code: 数据功能码 ************/
/************** load_add: 数据起始地址 **************/
/************** load_qua: 载入数据个数 **************/
/****************************************************/
u8 location = 1;
u8 load_uart_num[] = {1,1,1,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3};
u8 load_func_code[] = {1,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};
u16 load_add[] = {0,0,0,1,81,1901,1001,1101,1173,1501,1573,2301,1,81,1901,1001,1101,1173,1501,1573,2301};
u8 load_qua[] = {16,40,78,55,32,84,34,72,72,72,72,3,55,32,84,34,72,72,72,72,3};

u8 gprs_queue[21][USART_REC_NUM] = {0};
u8 gprs_count[21] = {0};
u8 gprs_location = 0;
u8 gprs_queue_location = 0;

u8 gprs_str_test[] = {0x73,0x74,0x72,0x3D,0x02,0x03,0x06,0x00,0x24,0x01,0x02,0x01,0x05,0x00,0x11};
#endif