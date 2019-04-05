//===========================================================================//
//                                                                           //
// 文件：  pin_def.h                                                         //
// 说明：  管脚定义和常用的宏定义以及全局变量                                //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.5                        //
// 版本：  v1.0 
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUSR                                                             //
//                                                                           //
//===========================================================================//
#ifndef PIN_DEF_H

#define PIN_DEF_H



//***************************************************************************//
//                                                                           //
//                       常用的宏定义                                        //
//                                                                           //
//***************************************************************************//
/*****************************************************************************
函数名称：延时函数
功    能：进行精度为1us和1ms的延时
备    注：视MCLK频率而定，MCLK=16M则CPU_F=16000000，若MCLK=8M则CPU_F=8000000
******************************************************************************/
#define CPU_F ((double)8000000) 
#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0)) 
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0))


//***************************************************************************//
//                                                                           //
//                       引脚功能定义                                        //
//                                                                           //
//***************************************************************************//

// P1 引脚定义
#define	WSI_G   	       BIT3  // Working status indicator工作状态指示灯——绿
#define	WSI_Y   	       BIT4  // Working status indicator工作状态指示灯——黄 
#define	WSI_R   	       BIT5  // Working status indicator工作状态指示灯——红 
#define RELAY2                 BIT6  //relay继电器2 5V输出
#define RELAY1                 BIT7  //relay继电器1 12V输出

// P2 引脚定义
#define	FL_CTRL   	       BIT5  //fill light补光灯控制管脚 

// P3 引脚定义
#define	RXD_US   	       BIT4  //Ultrasonic sensor超声波避障传感器UART0接口
#define	TXD_US   	       BIT5  //Ultrasonic sensor超声波避障传感器UART0接口

// P4 引脚定义


// P5 引脚定义

#define	RXD_IPORT              BIT6  //IPORT口 UART1接口
#define	TXD_IPORT              BIT7  //IPORT口 UART1接口

// P6 引脚定义

// P7 引脚定义
#define COM_CFG                BIT2  //IPORT口 复位引脚
#define	LED1   	 	       BIT6  // 状态指示灯LED1
#define	LED2   	 	       BIT7  // 状态指示灯LED2 

// P8 引脚定义
#define USC1                   BIT5  // 超声波传感器选择1
#define USC2                   BIT6  // 超声波传感器选择2
#define USC3                   BIT7  // 超声波传感器选择3

// P9 引脚定义
#define TXD_JF                 BIT4  //局放传感器 UART2接口
#define RXD_JF                 BIT5  //局放传感器 UART2接口
#define RS485EN_JF             BIT6  //局放传感器 458使能

// P10 引脚定义
#define	SDA                    BIT1  // E2PROM  
#define	SCL                    BIT2  // E2PROM  
#define TXD485BUS 	       BIT4  //485总线
#define	RXD485BUS	       BIT5  //485总线
#define	RS485BUSEN             BIT6  // 串口3 485使能

// P11 引脚定义
#define TACK                   BIT0  // ACLK测试端
#define TMCK                   BIT1  // MCLK测试端
#define TSMCK                  BIT2  // SMCLK测试端

#endif


