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
#define	DCMSTOP2   	       BIT1  // 伸缩杆驱动停止位
#define	DCMEN2        	       BIT3  // 伸缩杆驱动使能位
#define	DCMPWM2       	       BIT2  // 伸缩杆驱动PWM位
#define	DCMSTOP1 	       BIT4  // 小车驱动停止位
#define	DCMEN1 	 	       BIT6  // 小车驱动使能位
#define	DCMPWM1       	       BIT5  // 小车驱动PWM位

// P2 引脚定义
#
#define	LED1       	       BIT4  // LED1
#define	LED2       	       BIT5  // LED2
//#define	JJ1       	       BIT6  // 小车驱动器就绪/接近开关1
#define	JJ1       	       BIT7  // 伸缩杆驱动器就绪/接近开关2

// P3 引脚定义
//#define	JJ1       	       BIT0  // 接近开关1
//#define JJ2       	       BIT1  // 接近开关2

// P4 引脚定义
#define AM                     BIT7  // 温湿度传感器

// P5 引脚定义
#define ENCODER_SET            BIT5  // 编码器配置管脚
#define	TXD485BUS              BIT6  // 485总线
#define	RXD485BUS              BIT7  // 485总线

// P6 引脚定义

// P7 引脚定义
#define	RS485BUSEN             BIT2  // 485总线使能

// P8 引脚定义

// P9 引脚定义

#define TXD485_ENCODER1        BIT4  // 小车编码器串口
#define	RXD485_ENCODER1	       BIT5  // 小车编码器串口
#define	RS485_ENCODER1EN       BIT6  // 小车编码器485使能

// P10 引脚定义
#define	SDA                    BIT1  // E2PROM  
#define	SCL                    BIT2  // E2PROM  
#define TXD485_ENCODER2        BIT4  // 伸缩杆编码器串口
#define	RXD485_ENCODER2	       BIT5  // 伸缩杆编码器串口
#define	RS485_ENCODER2EN       BIT6  // 伸缩杆编码器485使能


// P11 引脚定义
#define TACK                   BIT0  // ACLK测试端
#define TMCK                   BIT1  // MCLK测试端
#define TSMCK                  BIT2  // SMCLK测试端

#endif


