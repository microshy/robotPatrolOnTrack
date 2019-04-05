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

#define obstacleBLOW      BIT1
#define obstacleFRONT     BIT2
#define obstacleBEHiND    BIT3

void Reset();
void GetVersionReq();
void GetChrgStateReq();
void GetODOReq();
void GetStatusReq();
void GetBatteryReq();
void Move();
void Stop();
void EmergencyStop();
void GetPositionReq();
void InfraPtzRoll();
void RollYunTai();
void InfraPtzStop();
void InfraPtzRollToReq ();
void InfraPtzGetAngleReq ();
void DetectTEVReq();
void DetectUSReq();
void DetectO2Req();
void DetectSF6Req();
void DetectEnvParamReq();
void UpdateRomReq();

void MoveTo_Origin(void);
void MoveTo_Disribute(void);
void MoveToXYZ_RES(void);
void MoveToXYZ_END(void);
void MovetoXYZ(void);
void InstructProcess();//指令处理

void DetectState_02();
void DetectState_03();
void DetectState_04();
void DetectState_05();
void DetectPD();
unsigned char DetectObstacle();
void DetectState();

void UpdateProgram();

#endif

