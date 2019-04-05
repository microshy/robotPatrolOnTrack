//===========================================================================//
//                                                                           //
// �ļ���  pin_def.h                                                         //
// ˵����  �ܽŶ���ͳ��õĺ궨���Լ�ȫ�ֱ���                                //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.5                        //
// �汾��  v1.0 
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUSR                                                             //
//                                                                           //
//===========================================================================//
#ifndef PIN_DEF_H
#define PIN_DEF_H



//***************************************************************************//
//                                                                           //
//                       ���õĺ궨��                                        //
//                                                                           //
//***************************************************************************//
/*****************************************************************************
�������ƣ���ʱ����
��    �ܣ����о���Ϊ1us��1ms����ʱ
��    ע����MCLKƵ�ʶ�����MCLK=16M��CPU_F=16000000����MCLK=8M��CPU_F=8000000
******************************************************************************/
#define CPU_F ((double)8000000) 
#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0)) 
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0))


//***************************************************************************//
//                                                                           //
//                       ���Ź��ܶ���                                        //
//                                                                           //
//***************************************************************************//

// P1 ���Ŷ���
#define	DCMSTOP2   	       BIT1  // ����������ֹͣλ
#define	DCMEN2        	       BIT3  // ����������ʹ��λ
#define	DCMPWM2       	       BIT2  // ����������PWMλ
#define	DCMSTOP1 	       BIT4  // С������ֹͣλ
#define	DCMEN1 	 	       BIT6  // С������ʹ��λ
#define	DCMPWM1       	       BIT5  // С������PWMλ

// P2 ���Ŷ���
#
#define	LED1       	       BIT4  // LED1
#define	LED2       	       BIT5  // LED2
//#define	JJ1       	       BIT6  // С������������/�ӽ�����1
#define	JJ1       	       BIT7  // ����������������/�ӽ�����2

// P3 ���Ŷ���
//#define	JJ1       	       BIT0  // �ӽ�����1
//#define JJ2       	       BIT1  // �ӽ�����2

// P4 ���Ŷ���
#define AM                     BIT7  // ��ʪ�ȴ�����

// P5 ���Ŷ���
#define ENCODER_SET            BIT5  // ���������ùܽ�
#define	TXD485BUS              BIT6  // 485����
#define	RXD485BUS              BIT7  // 485����

// P6 ���Ŷ���

// P7 ���Ŷ���
#define	RS485BUSEN             BIT2  // 485����ʹ��

// P8 ���Ŷ���

// P9 ���Ŷ���

#define TXD485_ENCODER1        BIT4  // С������������
#define	RXD485_ENCODER1	       BIT5  // С������������
#define	RS485_ENCODER1EN       BIT6  // С��������485ʹ��

// P10 ���Ŷ���
#define	SDA                    BIT1  // E2PROM  
#define	SCL                    BIT2  // E2PROM  
#define TXD485_ENCODER2        BIT4  // �����˱���������
#define	RXD485_ENCODER2	       BIT5  // �����˱���������
#define	RS485_ENCODER2EN       BIT6  // �����˱�����485ʹ��


// P11 ���Ŷ���
#define TACK                   BIT0  // ACLK���Զ�
#define TMCK                   BIT1  // MCLK���Զ�
#define TSMCK                  BIT2  // SMCLK���Զ�

#endif


