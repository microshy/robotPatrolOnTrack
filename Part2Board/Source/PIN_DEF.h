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
#define	LED1   	 	       BIT5  // 状态指示灯LED1
#define	LED2   	 	       BIT6  // 状态指示灯LED2  
#define	FAN1_CTRL   	       BIT7  // 风扇1控制信号 

// P2 引脚定义

// P3 引脚定义
#define	RXD_O2   	       BIT4  //O2传感器UART0接口
#define	TXD_O2   	       BIT5  //O2传感器UART0接口

// P4 引脚定义

// P5 引脚定义

#define	RXD_SF6                BIT6  //SF6传感器 UART1接口
#define	TXD_SF6                BIT7  //SF6传感器 UART1接口

// P6 引脚定义

// P7 引脚定义

// P8 引脚定义

// P9 引脚定义

#define TXD485_VOLUME          BIT4  //声音传感器 UART2接口
#define RXD485_VOLUME          BIT5  //声音传感器 UART2接口
#define RS485EN_VOLUME         BIT6  //声音传感器 458使能

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


