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

// P2 引脚定义

// P3 引脚定义
#define RELAY5                 BIT5  // 继电器5（5V输出）
#define RELAY4                 BIT6  // 继电器4（备用）
#define RELAY3                 BIT7  // 继电器3（运动控制）


// P4 引脚定义
#define RELAY2                 BIT0  // 继电器2（电源开关）
#define RELAY1                 BIT1  // 继电器1（充电）
#define LED1                   BIT4  // LED1
#define LED2                   BIT5  // LED2

// P5 引脚定义
#define	RS485BUSEN             BIT5  // 485总线使能
#define	TXD485BUS              BIT6  // 485总线
#define	RXD485BUS              BIT7  // 485总线

// P6 引脚定义
#define DCVOLTAGE1             BIT4  // 电压检测1
#define DCVOLTAGE2             BIT5  // 电压检测2
#define DCCURRENT              BIT6  // 电流检测

// P7 引脚定义

// P8 引脚定义

// P9 引脚定义

// P10 引脚定义
#define	SDA                    BIT1  // E2PROM  
#define	SCL                    BIT2  // E2PROM  

// P11 引脚定义
#define TACK                   BIT0  // ACLK测试端
#define TMCK                   BIT1  // MCLK测试端
#define TSMCK                  BIT2  // SMCLK测试端

#endif


