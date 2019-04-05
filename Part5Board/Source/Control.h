//===========================================================================//
//                                                                           //
// 文件：  Control.h                                                         //
// 说明：  功能函数，根据通信协议做相应动作                                  //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//
#ifndef Control_H
#define Control_H

extern unsigned char timer0Flag;//定时时间到标志
extern unsigned int timer0Count;//定时时间到标志

void DetectState();
void InstructProcess();
void DetectOwnState();

void DetectVol();
void DectctCur();
float detectVoltage();
float detectCurrent();

#endif

