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
#define LA_IN1                 BIT1  //linear actuator电动推杆方向控制1
#define LA_IN2                 BIT2  //linear actuator电动推杆方向控制2
#define LA_PWM                 BIT3  //linear actuator电动推杆速度控制PWM
#define	LS3_LA   	       BIT4  // linear actuator电动推杆 限位3，（尾部限位）
#define	LS1_SERVO 	       BIT5  // limit switch 云台舵机限位1
#define	LS2_LA 	 	       BIT6  // linear actuator电动推杆 限位2，（头部限位）
#define	LS2_SERVO   	       BIT7  // limit switch 云台舵机限位2

// P2 引脚定义
#define	LS1_LA   	       BIT0  // linear actuator电动推杆 限位1，（头部限位）
#define SERVO_PWM1             BIT2  //云台舵机PWM1
#define SERVO_PWM2             BIT3  //云台舵机PWM2
#define DIR_ENCODER            BIT4  // encoder 编码器方向
#define COUNT_ENCODER          BIT5  // encoder 编码器计数

// P3 引脚定义
#define FAN2_CTRL              BIT1  // 风扇控制信号
#define	LED2       	       BIT2  // LED2
#define	LED1       	       BIT3  // LED1

// P4 引脚定义

// P5 引脚定义

#define	RXD_US                 BIT6  // Ultrasonic sensor超声波避障传感器 UART1接口
#define	TXD_US                 BIT7  // Ultrasonic sensor超声波避障传感器 UART1接口

// P6 引脚定义

// P7 引脚定义

// P8 引脚定义
#define USC1                   BIT5  // 超声波传感器选择1
#define USC2                   BIT6  // 超声波传感器选择2
#define USC3                   BIT7  // 超声波传感器选择3

// P9 引脚定义
#define TXD485BUS 	       BIT4  //485总线
#define	RXD485BUS	       BIT5  //485总线
#define	RS485BUSEN             BIT6  // 串口2 485使能

// P10 引脚定义
#define	SDA                    BIT1  // E2PROM  
#define	SCL                    BIT2  // E2PROM  

// P11 引脚定义
#define TACK                   BIT0  // ACLK测试端
#define TMCK                   BIT1  // MCLK测试端
#define TSMCK                  BIT2  // SMCLK测试端

#endif


