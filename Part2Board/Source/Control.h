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

void InstructProcess();//ָ���
void MinTimeslice();
void DetectO2();
void DetectVolume();
void DetectSensor();
#endif

