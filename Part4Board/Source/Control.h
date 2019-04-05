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

extern long LencoderX;
extern long LencoderXOld;
extern long LencoderY;
extern long LencoderYOld;
extern unsigned char Uart1InstructFlag;//接收到指令标志
extern unsigned char Uart1InstructNum;//指令编号
extern unsigned char Uart2InstructFlag;//接收到指令标志
extern unsigned char Uart3InstructFlag;//接收到指令标志

void DetectState();//状态查询函数
void Move();//移动函数
void Stop();//停止函数
void MoveToX(void);
void MoveToX_END(void);
void MoveToY(void);
void MoveToY_END(void);

void InstructProcess();//指令处理
void DetectEncoder1();
void DetectEncoder2();
void DetectTAH();
void MinTimeslice();
void ReadEncoder1();
void ReadEncoder2();
unsigned char Read_AM2301(void);
void DetectAM();

void DetectSensor(void);

void Reset();

#endif

