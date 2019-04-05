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

extern unsigned char timer0Flag;//��ʱʱ�䵽��־
extern unsigned int timer0Count;//��ʱʱ�䵽��־

void DetectState();
void InstructProcess();
void DetectOwnState();

void DetectVol();
void DectctCur();
float detectVoltage();
float detectCurrent();

#endif

