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

extern long LencoderX;
extern long LencoderXOld;
extern long LencoderY;
extern long LencoderYOld;
extern unsigned char Uart1InstructFlag;//���յ�ָ���־
extern unsigned char Uart1InstructNum;//ָ����
extern unsigned char Uart2InstructFlag;//���յ�ָ���־
extern unsigned char Uart3InstructFlag;//���յ�ָ���־

void DetectState();//״̬��ѯ����
void Move();//�ƶ�����
void Stop();//ֹͣ����
void MoveToX(void);
void MoveToX_END(void);
void MoveToY(void);
void MoveToY_END(void);

void InstructProcess();//ָ���
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

