#include "usart1.h"
#include "string.h"
#include "stdlib.h"  
#include "led.h" 
#include "beep.h" 
#include "lcd.h"
#include "config.h"

void TIM2_Init(u16 auto_data, u16 fractional) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);      //使能TIM2时钟

    TIM_TimeBaseInitStructure.TIM_Period = auto_data;         //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = fractional;      //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);//初始化TIM2

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //允许定时器2更新中断
    TIM_Cmd(TIM2, ENABLE);                    //使能定时器2

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //定时器2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;  //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


//初始化IO USART1   bound:波特率
void uart1_init(u32 bound) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//使能USART1时钟
    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  //GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10复用为USART1
    //USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);          //初始化PA9，PA10
    //USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_Cmd(USART1, ENABLE);  //使能串口1

    USART_ClearFlag(USART1, USART_FLAG_TC);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);         //开启相关中断
    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;      //串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;           //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);      //根据指定的参数初始化VIC寄存器、
}

/*************UART1发送一个字符***************/
void uart1SendChar(u8 ch) {
    while ((USART1->SR & 0x40) == 0);
    USART1->DR = (u8) ch;
}

/*************UART1发送一个字符串***************/
void u1SendChars(u8 *str, u16 strlen) {
    u16 k = 0;
    do {
        uart1SendChar(*(str + k));
        k++;
    }   //循环发送,直到发送完毕
    while (k < strlen);
}

//初始化IO USART2   bound:波特率
void RS485_Init(u32 bound) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//使能USART2时钟

    //串口2引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //GPIOA2复用为USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //GPIOA3复用为USART2

    //USART2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2，PA3

    //PG8推挽输出，485模式控制
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //GPIOG6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOG, &GPIO_InitStructure); //初始化PG8

    //USART2 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口2

    USART_Cmd(USART2, ENABLE);  //使能串口 2

    USART_ClearFlag(USART2, USART_FLAG_TC);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启接受中断

    //Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化VIC寄存器、

    RS485_TX_EN = 0;                //初始化默认为接收模式
}

//RS485发送len个字节
void u2SendChars(u8 *buf, u8 len) {
    u8 t;
    RS485_TX_EN = 1;                //设置为发送模式
    for (t = 0; t < len; t++)        //循环发送数据
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET); //等待发送结束
        USART_SendData(USART2, buf[t]); //发送数据
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);//等待发送结束
    flag2_byte_count = 0;
    RS485_TX_EN = 0;                //发送完设置为接收模式
}

//RS485查询接收到的数据
void RS485_Receive_Data(u8 *buf, u8 *len) {
    u8 rxlen = flag2_byte_count;
    u8 i = 0;
    *len = 0;                   //默认为0
    delay_ms(10);         //等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束

    if (rxlen == flag2_byte_count && rxlen) //接收到了数据,且接收完成了
    {
        for (i = 0; i < rxlen; i++) {
            buf[i] = receive_str2[i];
        }
        *len = flag2_byte_count;    //记录本次数据长度
        flag2_byte_count = 0;          //清零
    }
}

void uart3_init(u32 bound) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //使能GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //使能USART3时钟
    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);  //GPIOB10复用为USART3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); //GPIOB11复用为USART3
    //USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10与GPIOB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);          //初始化PB10，PB11
    //USART3 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //收发模式
    USART_Init(USART3, &USART_InitStructure); //初始化串口1
    USART_Cmd(USART3, ENABLE);  //使能串口1

    USART_ClearFlag(USART3, USART_FLAG_TC);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);         //开启相关中断
    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;      //串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;           //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);      //根据指定的参数初始化VIC寄存器、
}

void uart3SendChar(u8 ch) {
    while ((USART3->SR & 0x40) == 0);
    USART3->DR = (u8) ch;
}

void u3SendChars(u8 *str, u16 strlen) {
    u16 k = 0;
    do {
        uart3SendChar(*(str + k));
        k++;
    } while (k < strlen);
}

void uart5_init(u32 bound) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);
    USART_Cmd(UART5, ENABLE);

    USART_ClearFlag(UART5, USART_FLAG_TC);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void uart5SendChar(u8 ch) {
    while ((UART5->SR & 0x40) == 0);
    UART5->DR = (u8) ch;
}

void u5SendChars(u8 *str, u16 strlen) {
    u16 k = 0;
    do {
        uart5SendChar(*(str + k));
        k++;
    } while (k < strlen);
}

void uart6_init(u32 bound) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &USART_InitStructure);
    USART_Cmd(USART6, ENABLE);

    USART_ClearFlag(USART6, USART_FLAG_TC);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void uart6SendChar(u8 ch) {
    while ((USART6->SR & 0x40) == 0);
    USART6->DR = (u8) ch;
}

void u6SendChars(u8 *str, u16 strlen) {
    u16 k = 0;
    do {
        uart6SendChar(*(str + k));
        k++;
    }   //循环发送,直到发送完毕
    while (k < strlen);
}

/*************16位CRC校验函数,查表法***************/
u16 crc16(u8 *puchMsg, u8 usDataLen) {
    u8 uchCRCHi = 0xFF;
    u8 uchCRCLo = 0xFF;
    u16 uIndex;
    while (usDataLen--) {
        uIndex = uchCRCHi ^ *puchMsg++;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (((u16)(uchCRCLo) << 8) | uchCRCHi);
}

u16 *data_search(n, add) {  //Modbus_03 查询函数
    if (n == 0x01) {
        if(add <= 77) return &ele[add];
        if(add == 78) return &err_1;
    } else if (n == 0x02) {
        if (add <= 112) return &ups1[add];
        if (add == 113) return &err_2;
        if (add >= 1001 && add <= 1033) return &ups1[add - 1001 + 113];
        if (add >= 1101 && add <= 1244) return &ups1[add - 1101 + 146];
        if (add >= 1501 && add <= 1644) return &ups1[add - 1501 + 290];
        if (add >= 1901 && add <= 1984) return &ups1[add - 1901 + 434];
        if (add >= 2301 && add <= 2303) return &ups1[add - 2301 + 518];
    } else if (n == 0x03) {
        if (add <= 112) return &ups2[add];
        if (add == 113) return &err_3;
        if (add >= 1001 && add <= 1033) return &ups2[add - 1001 + 113];
        if (add >= 1101 && add <= 1244) return &ups2[add - 1101 + 146];
        if (add >= 1501 && add <= 1644) return &ups2[add - 1501 + 290];
        if (add >= 1901 && add <= 1984) return &ups2[add - 1901 + 434];
        if (add >= 2301 && add <= 2303) return &ups2[add - 2301 + 518];
    }
}
u16 *data_search_scada(n, add) {  //Modbus_03 查询函数
    if (n == 0x01) {
        if(add >= 3200 && add <= 3271) return &ele[add - 3200];
    } else if (n == 0x02) {
        if (add >= 3278 && add <= 3332) return &ups1[add - 3277];
        if (add >= 3335 && add <= 3342) return &ups1[add - 3335 + 113];
        if (add >= 3343 && add <= 3486) return &ups1[add - 3343 + 146];
        if (add >= 3487 && add <= 3630) return &ups1[add - 3487 + 290];
        if (add == 3984) return &ups1[81];
        if (add == 3985) return &ups1[82];
    } else if (n == 0x03) {
        if (add >= 3631 && add <= 3685) return &ups2[add - 3630];
        if (add >= 3688 && add <= 3695) return &ups2[add - 3688 + 113];
        if (add >= 3696 && add <= 3839) return &ups2[add - 3696 + 146];
        if (add >= 3840 && add <= 3983) return &ups2[add - 3840 + 290];
        if (add == 3987) return &ups2[81];
        if (add == 3988) return &ups2[82];
    }
}
void MODBUS_01() {    //MODBUS_01功能码
    u16 crc = 0;
    u8 hi = 0, low = 0;
    u16 length = 0;
    u16 num = 0;
    u8 i = 0;
    u16 begin_address = 0;
    begin_address = (use_str[2] << 8) + use_str[3];
    crc = crc16(use_str, 6);                       //校验
    if (crc == (use_str[7] << 8 | use_str[6]))      //当校验一致时
    {
        length = (use_str[4] << 8) + use_str[5];
        num = length / 8;
        if (length % 8) num++;
        for (i = 0; i < num; i++) {
            Send_buf[i + 3] = ele3[begin_address / 8 + i];//寻找数据
        }
        Send_buf[0] = use_str[0];                      //站号
        Send_buf[1] = use_str[1];                      //功能码
        Send_buf[2] = num;                              //字节数
        crc = crc16(Send_buf, 5);                      //校验
        Send_buf[num + 3] = crc % 256;                      //校验低位
        Send_buf[num + 4] = crc / 256;                      //校验高位
        u1SendChars(Send_buf, num + 5);                          //发送返回屏
    }
}
void MODBUS_02() {    //MODBUS_02功能码
    u16 crc = 0;
    u8 hi = 0, low = 0;
    u16 length = 0;
    u16 num = 0;
    u8 i = 0;
    u16 begin_address = 0;
    begin_address = (use_str[2] << 8) + use_str[3];
    crc = crc16(use_str, 6);                       //校验
    if (crc == (use_str[7] << 8 | use_str[6]))      //当校验一致时
    {
        length = (use_str[4] << 8) + use_str[5];
        num = length / 8;
        if (length % 8) num++;
        for (i = 0; i < num; i++) {
            Send_buf[i + 3] = ele2[begin_address / 8 + i];
        }
        Send_buf[0] = use_str[0];                      //站号
        Send_buf[1] = use_str[1];                      //功能码
        Send_buf[2] = num;                              //字节数
        crc = crc16(Send_buf, num + 3);                      //校验
        Send_buf[num + 3] = crc % 256;                      //校验低位
        Send_buf[num + 4] = crc / 256;                      //校验高位
        u1SendChars(Send_buf, num + 5);                          //发送返回屏
    }
}
void MODBUS_03() {   //MODBUS_03功能码
    u16 legg;//03功能码发送的位的长度
    u16 receive_length;
    u16 plc;
    u8 hi = 0, low = 0;
    u8 time = 0;
    u16 begin_address = 0;
    u16 crc = crc16(use_str, 6);                    //校验
    if (crc == (use_str[7] << 8 | use_str[6]))    //校验一致
    {
        begin_address = use_str[2] << 8 | use_str[3];
        receive_length = use_str[4] << 8 | use_str[5];     //总寄存器长度
        Send_buf[0] = use_str[0];                    //站号
        plc = (u16) use_str[0];
        Send_buf[1] = use_str[1];                    //功能码
        legg = (u16) receive_length * 2;                    //字节数
        Send_buf[2] = receive_length * 2;;                    //字节数
        while(time < legg) {
            u16 data = *(data_search(plc, begin_address + time / 2));
            Send_buf[time + 3] = data >> 8;
            Send_buf[time + 4] = data & 0x00ff;
            time = time + 2;
        }
//        for (time = 0; time < legg; time++)                //发送相应字节
//        {
//            if (!(time % 2))                            //字高位为0
//                Send_buf[time + 3] =
//                        *(time / 2 + data_search(plc, begin_address)) >> 8;        //字高字节右移八位，在得到的地址上加time/2得到后面的数据
//            else
//                Send_buf[time + 3] = *(time / 2 + data_search(plc, begin_address)) & 0x00ff;        //字低字节 与上0x00ff即为低四位
//        }
        crc = crc16(Send_buf, legg + 3);                //校验
        Send_buf[legg + 3] = crc % 256;                //校验低位
        Send_buf[legg + 4] = crc / 256;                //校验高位
        u1SendChars(Send_buf, legg + 5);                    //返回屏
    }
}

void MODBUS_03_scada() {
    u16 legg, plc, receive_length, begin_address;
    u8 hi = 0, low = 0, i;
    u16 crc = crc16(receive_scada, 6);
    if (crc == (receive_scada[7] << 8 | receive_scada[6])){
        begin_address = receive_scada[2] << 8 | receive_scada[3];
        receive_length = receive_scada[4] << 8 | receive_scada[5];
        plc = (u16) receive_scada[0];
        legg = (u16) receive_length * 2;
        Send_buf[0] = receive_scada[0];
        Send_buf[1] = receive_scada[1];
        Send_buf[2] = receive_length * 2;
        while(i < legg) {
            u16 data = *(data_search_scada(plc, begin_address + i / 2));
            Send_buf[i + 3] = data >> 8;
            Send_buf[i + 4] = data & 0x00ff;
            i = i + 2;
        }
        crc = crc16(Send_buf, legg + 3);
        Send_buf[legg + 3] = crc % 256;
        Send_buf[legg + 4] = crc / 256;
        u1SendChars(Send_buf, legg + 5);
    }
}

void data_add(n, add, num) {  //Modbus_06 赋值函数
    if (n == 0x01) {
        ele[add] = num;
    } else if (n == 0x02) {
        if (add <= 112) ups1[add] = num;
        if (add >= 1001 && add <= 1033) ups1[add - 1001 + 113] = num;
        if (add >= 1101 && add <= 1244) ups1[add - 1101 + 146] = num;
        if (add >= 1501 && add <= 1644) ups1[add - 1501 + 290] = num;
        if (add >= 1901 && add <= 1984) ups1[add - 1901 + 434] = num;
        if (add >= 2301 && add <= 2303) ups1[add - 2301 + 518] = num;
    } else if (n == 0x03) {
        if (add <= 112) ups1[add] = num;
        if (add >= 1001 && add <= 1033) ups2[add - 1001 + 113] = num;
        if (add >= 1101 && add <= 1244) ups2[add - 1101 + 146] = num;
        if (add >= 1501 && add <= 1644) ups2[add - 1501 + 290] = num;
        if (add >= 1901 && add <= 1984) ups2[add - 1901 + 434] = num;
        if (add >= 2301 && add <= 2303) ups2[add - 2301 + 518] = num;
    }
}

void MODBUS_06() {  //MODBUS_06功能码
    u16 crc = 0;
    u16 num;
    u16 begin_address = 0;
    begin_address = use_str[2] << 8 | use_str[3]; //写入地址
    num = use_str[4] << 8 | use_str[5];    //寄存器值
    crc = crc = crc16(use_str, 6);                    //校验
    if (crc == (use_str[7] << 8 | use_str[6]))    //校验正确
    {
        data_add(begin_address, num);            //存入数组，给03功能码调用数值
        Send_buf[0] = use_str[0];                    //站号
        Send_buf[1] = use_str[1];                    //功能码
        Send_buf[2] = use_str[2];                    //开始高位
        Send_buf[3] = use_str[3];                    //开始低位
        Send_buf[4] = use_str[4];                    //字高位
        Send_buf[5] = use_str[5];                    //字低位
        crc = crc16(Send_buf, 6);                    //校验
        Send_buf[6] = crc % 256;                    //校验低位
        Send_buf[7] = crc / 256;                    //校验高位
        u2SendChars(Send_buf, 8);                        //返回屏
    }
}

void MODBUS_load(uart_num, func_code, add, q) {  //发送读取请求（站号,地址,数据个数）
    Send_buf[0] = (u16) uart_num;//站号
    Send_buf[1] = (u16) func_code;//功能码
    Send_buf[2] = add / 256;//起始地址高位
    Send_buf[3] = add % 256;//起始地址低位
    Send_buf[4] = q / 256;//总寄存器个数高位
    Send_buf[5] = q % 256;//总寄存器个数低位
    Send_buf[6] = crc16(Send_buf, 6) % 256;                    //校验低位
    Send_buf[7] = crc16(Send_buf, 6) / 256;                    //校验高位
    if (func_code == 3) {
        uart2_byte_count = 5 + 2 * q;
    } else {
        u16 num = 0;
        num = q / 8;
        if (q % 8) num++;
        uart2_byte_count = 5 + num;
    }
    flag_add = add;
    delay_ms(20);
    switch (uart_num) {
        case 1:
            timing_1 = 500;
            u2SendChars(Send_buf, 8);
            break;
        case 2:
            timing_2 = 500;
            u3SendChars(Send_buf, 8);
            break;
        case 3:
            timing_3 = 500;
            u6SendChars(Send_buf, 8);
            break;
    }
}

void load_next() {
    u8 num;
    if (location == LOAD_LENGTH) {
        location = 0;
    }
    num = load_uart_num[location];

    if (!((err_1 && num == 1) || (err_2 && num == 2) || (err_3 && num == 3))) {
        MODBUS_load(num, load_func_code[location], load_add[location], load_qua[location]);
        location++;
    }
}

void test() {
    u8 i = 0;
    for (i = 0; i < 3; i++) {
        ups1[i]++;
    }
}

/*************MODBUS协议程序***************/
void command(void) {
    if (use_str[1] == 0x01) MODBUS_01();                     //01功能码
    if (use_str[1] == 0x03) MODBUS_03();                     //03功能码
    if (use_str[1] == 0x06) MODBUS_06();                     //06功能码
    if (use_str[1] == 0x02) MODBUS_02();                     //02功能码
}
//todo scada: modbus_02
void command_scada(void) {
//    if (receive_scada[1] == 0x01) MODBUS_01_scada();                     //01功能码
    if (receive_scada[1] == 0x03) MODBUS_03_scada();                     //03功能码
//    if (receive_scada[1] == 0x02) MODBUS_02_scada();                     //02功能码
}

void setData(void) {
    u8 a = 0, i = 0;
    u16 add = 0;
    if (receive_str2[1] == 0x03) {
        a = receive_str2[2] / 2;
        add = flag_add;
        for (i = 0; i < a; i++) {
            data_add(receive_str2[0], add + i, receive_str2[3 + 2 * i] << 8 | receive_str2[4 + 2 * i]);
        }
    }
    if (receive_str2[1] == 0x02) {
        a = receive_str2[2];
        add = flag_add;
        for (i = 0; i < a; i++) {
            ele2[add / 8 + i] = receive_str2[3 + i];
        }
    }
    if (receive_str2[1] == 0x01) {
        a = receive_str2[2];
        add = flag_add;
        for (i = 0; i < a; i++) {
            ele3[add / 8 + i] = receive_str2[3 + i];
        }
    }
}

void GPRS_Send(u8 *str, u16 strlen) {
    u8 i;
    u8 http[] = "POST /api/abc HTTP/1.1\r\nHost:120.25.77.40:80\r\nContent-Type:application/x-www-form-urlencoded\r\nContent-Length:";
    u8 cr[] = "\r\n\r\nstr=";
    u16 location;
    u16 length;
    u16 body_length = strlen + 4;
    if (body_length < 10) {
        location = 110;
        http[location - 1] = body_length % 10 + '0';
    } else if (body_length < 100) {
        location = 111;
        http[location - 2] = body_length / 10 + '0';
        http[location - 1] = body_length % 10 + '0';
    } else {
        location = 112;
        http[location - 3] = body_length / 100 + '0';
        http[location - 2] = body_length / 10 + '0';
        http[location - 1] = body_length % 10 + '0';
    }
    strcat(http, cr);
    length = location + 8 + strlen;
    for (i = 0; i < strlen; i++) {
        http[location + 8 + i] = str[i];
    }
    delay_ms(30);
    u5SendChars(http, length);
}

void gprs_queue_load() {
    // 这里生成GPRS发送队列
    if (gprs_queue_ready == 0) {
        if (gprs_queue_location < LOAD_LENGTH) {
            receive_str2[uart2_byte_count - 2] = flag_add / 256;//地址高位
            receive_str2[uart2_byte_count - 1] = flag_add % 256;//地址低位
            memcpy(gprs_queue[location - 1], receive_str2, sizeof(receive_str2));
            gprs_count[location - 1] = uart2_byte_count;
            gprs_queue_location++;
        } else {
            LED2 = !LED2;
            gprs_queue_location = 0;
            gprs_queue_ready = 1;
        }
    }
}

void MODBUS_send(void) {
    if (flag_hmi_send) {
        command();
        flag_hmi_send = 0;
    }
//    if (flag_scada_send) {
//        //加入scada的处理函数
//        command_scada();
//        flag_scada_send = 0;
//    }
    if(flag_gprs && gprs_queue_ready) {
        if (gprs_location < LOAD_LENGTH) {
            GPRS_Send(gprs_queue[gprs_location], gprs_count[gprs_location]);
            gprs_location++;
            flag_gprs = 0;
        } else {
            LED2 = !LED2;
            gprs_location = 0;
            gprs_queue_ready = 0;
        }
    }
    //测试gprs连续性
//    if (flag_gprs) {
//        GPRS_Send(gprs_str_test, 15);
//        flag_gprs = 0;
//    }
    if (flag_finish) {
        setData();
        gprs_queue_load();
        flag_finish = 0;
        load_next();
    }
    if(err_1 && load_uart_num[location] == 1) {
        location = 3;
        load_next();
    }
    if(err_2 && load_uart_num[location] == 2) {
        location = 12;
        load_next();
    }
    if(err_3 && load_uart_num[location] == 3) {
        location = 0;
        load_next();
    }


}

//USART1中断服务程序
void USART1_IRQHandler(void) {
    u8 rec_data;
    u8 i = 0;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        // 接收中断
        // (USART1->DR) 读取接收到的数据
        rec_data = (u8) USART_ReceiveData(USART1);
        receive_str[flag_byte_count] = rec_data;
        flag_byte_count++;
        if (flag_byte_count == USART1_BYTE_COUNT) {
            // 到了uart2_byte_count位传递数据
            for (i = 0; i < 8; i++) {
                use_str[i] = receive_str[i];
            }
            flag_byte_count = 0;
            flag_hmi_send = 1;
        }
    }
}

//UART2中断服务函数
void USART2_IRQHandler(void) {
    u8 rec_data;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        rec_data = (u8) USART_ReceiveData(USART2);
        receive_str2[flag2_byte_count] = rec_data;
        flag2_byte_count++;
        if (flag2_byte_count == uart2_byte_count) {
            timing_1 = 0;
            flag2_byte_count = 0;
            flag_finish = 1;
        }
    }
}

void USART3_IRQHandler(void) {
    u8 rec_data;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        rec_data = (u8) USART_ReceiveData(USART3);
        receive_str2[flag2_byte_count] = rec_data;
        flag2_byte_count++;
        if (flag2_byte_count == uart2_byte_count) {
            timing_2 = 0;
            flag2_byte_count = 0;
            flag_finish = 1;
        }
    }
}

void USART6_IRQHandler(void) {
    u8 rec_data;
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
        rec_data = (u8) USART_ReceiveData(USART6);
        receive_str2[flag2_byte_count] = rec_data;
        flag2_byte_count++;
        if (flag2_byte_count == uart2_byte_count) {
            timing_3 = 0;
            flag2_byte_count = 0;
            flag_finish = 1;
        }
    }
}

void UART5_IRQHandler(void) {
    u8 rec_data;
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        rec_data = (u8) USART_ReceiveData(UART5);
//        receive_scada[scada_byte_count] = rec_data;
//        scada_byte_count++;
//        if (scada_byte_count == UART5_BYTE_COUNT) {
//            scada_byte_count = 0;
//            flag_scada_send = 1;
//        }
        //gprs
        if(!flag_gprs) {
            receive_gprs[0] = rec_data;
            gprs_byte_count++;
            u5Timeout = 5;
        }
    }
}
//定时器中断服务函数
void TIM2_IRQHandler(void) {
    //溢出中断
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        if (timing_1 > 0) {
            timing_1--;
            if (timing_1 == 0) {
                LED1 = !LED1;
                err_1 = 1;
            }
        }
        if (timing_2 > 0) {
            timing_2--;
            if (timing_2 == 0) {
                LED1 = !LED1;
                err_2 = 1;
            }
        }
        if (timing_3 > 0) {
            timing_3--;
            if (timing_3 == 0) {
                LED1 = !LED1;
                err_3 = 1;
            }
        }
        if(u5Timeout > 0) {
            u5Timeout--;
            if (u5Timeout == 0) {
                LED1 = !LED1;
//                xxx[0] = gprs_byte_count % 256;
//                xxx[1] = gprs_byte_count / 256;
                gprs_byte_count = 0;
                flag_gprs = 1;
            }
        }
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除中断标志位
}
