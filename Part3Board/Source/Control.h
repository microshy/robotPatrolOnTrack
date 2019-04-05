//===========================================================================//
//                                                                           //
// �ļ���  Control.h                                                         //
// ˵����  ���ܺ���������ͨ��Э������Ӧ����                                  //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
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

