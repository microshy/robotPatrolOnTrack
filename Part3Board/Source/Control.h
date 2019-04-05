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



void DetectState();
void Move_Z();
void Stop_Z();
void MoveToZ(void);
void MoveToZ_END(void);
void InfraPtzRoll();
void RollYunTai();
void InfraPtzStop ();
void InfraPtzRollToReq ();
void MinTimeslice();

void InstructProcess(void);

void DetectSensor(void);

#endif

