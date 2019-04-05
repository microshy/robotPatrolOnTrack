//===========================================================================//
//                                                                           //
// 文件：  SysInit.h                                                         //
// 说明：  系统硬件初始化                                                    //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/17                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//
#ifndef SysInit_H
#define SysInit_H

void Init_CLK(void);
void CLK_Test(void);

void Init_LED(void);
void FillLight(void);
void Init_WSI(void);
void Init_Relay(void);
void SelectUltrSens();
void Init_Timer0_A5(void);
//void Init_Timer0_A5_10Ms(unsigned int ms_10);//N*10ms定时器

void SysInit(void);

#endif