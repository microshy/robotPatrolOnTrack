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
void Init_FAN(void);
void Init_linearActuator(void);
void Init_Encoder(void);
void SelectUltrSens(void);
void Init_Timer0_A5(void);
void Init_Timer1_A3(void);
void Init_Timer0_B7(void);
void SysInit(void);
#endif