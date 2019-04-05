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
#define LA_IN1                 BIT1  //linear actuator�綯�Ƹ˷������1
#define LA_IN2                 BIT2  //linear actuator�綯�Ƹ˷������2
#define LA_PWM                 BIT3  //linear actuator�綯�Ƹ��ٶȿ���PWM
#define	LS3_LA   	       BIT4  // linear actuator�綯�Ƹ� ��λ3����β����λ��
#define	LS1_SERVO 	       BIT5  // limit switch ��̨�����λ1
#define	LS2_LA 	 	       BIT6  // linear actuator�綯�Ƹ� ��λ2����ͷ����λ��
#define	LS2_SERVO   	       BIT7  // limit switch ��̨�����λ2

// P2 ���Ŷ���
#define	LS1_LA   	       BIT0  // linear actuator�綯�Ƹ� ��λ1����ͷ����λ��
#define SERVO_PWM1             BIT2  //��̨���PWM1
#define SERVO_PWM2             BIT3  //��̨���PWM2
#define DIR_ENCODER            BIT4  // encoder ����������
#define COUNT_ENCODER          BIT5  // encoder ����������

// P3 ���Ŷ���
#define FAN2_CTRL              BIT1  // ���ȿ����ź�
#define	LED2       	       BIT2  // LED2
#define	LED1       	       BIT3  // LED1

// P4 ���Ŷ���

// P5 ���Ŷ���

#define	RXD_US                 BIT6  // Ultrasonic sensor���������ϴ����� UART1�ӿ�
#define	TXD_US                 BIT7  // Ultrasonic sensor���������ϴ����� UART1�ӿ�

// P6 ���Ŷ���

// P7 ���Ŷ���

// P8 ���Ŷ���
#define USC1                   BIT5  // ������������ѡ��1
#define USC2                   BIT6  // ������������ѡ��2
#define USC3                   BIT7  // ������������ѡ��3

// P9 ���Ŷ���
#define TXD485BUS 	       BIT4  //485����
#define	RXD485BUS	       BIT5  //485����
#define	RS485BUSEN             BIT6  // ����2 485ʹ��

// P10 ���Ŷ���
#define	SDA                    BIT1  // E2PROM  
#define	SCL                    BIT2  // E2PROM  

// P11 ���Ŷ���
#define TACK                   BIT0  // ACLK���Զ�
#define TMCK                   BIT1  // MCLK���Զ�
#define TSMCK                  BIT2  // SMCLK���Զ�

#endif


