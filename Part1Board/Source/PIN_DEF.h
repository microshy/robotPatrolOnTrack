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
#define	WSI_G   	       BIT3  // Working status indicator����״ָ̬ʾ�ơ�����
#define	WSI_Y   	       BIT4  // Working status indicator����״ָ̬ʾ�ơ����� 
#define	WSI_R   	       BIT5  // Working status indicator����״ָ̬ʾ�ơ����� 
#define RELAY2                 BIT6  //relay�̵���2 5V���
#define RELAY1                 BIT7  //relay�̵���1 12V���

// P2 ���Ŷ���
#define	FL_CTRL   	       BIT5  //fill light����ƿ��ƹܽ� 

// P3 ���Ŷ���
#define	RXD_US   	       BIT4  //Ultrasonic sensor���������ϴ�����UART0�ӿ�
#define	TXD_US   	       BIT5  //Ultrasonic sensor���������ϴ�����UART0�ӿ�

// P4 ���Ŷ���


// P5 ���Ŷ���

#define	RXD_IPORT              BIT6  //IPORT�� UART1�ӿ�
#define	TXD_IPORT              BIT7  //IPORT�� UART1�ӿ�

// P6 ���Ŷ���

// P7 ���Ŷ���
#define COM_CFG                BIT2  //IPORT�� ��λ����
#define	LED1   	 	       BIT6  // ״ָ̬ʾ��LED1
#define	LED2   	 	       BIT7  // ״ָ̬ʾ��LED2 

// P8 ���Ŷ���
#define USC1                   BIT5  // ������������ѡ��1
#define USC2                   BIT6  // ������������ѡ��2
#define USC3                   BIT7  // ������������ѡ��3

// P9 ���Ŷ���
#define TXD_JF                 BIT4  //�ַŴ����� UART2�ӿ�
#define RXD_JF                 BIT5  //�ַŴ����� UART2�ӿ�
#define RS485EN_JF             BIT6  //�ַŴ����� 458ʹ��

// P10 ���Ŷ���
#define	SDA                    BIT1  // E2PROM  
#define	SCL                    BIT2  // E2PROM  
#define TXD485BUS 	       BIT4  //485����
#define	RXD485BUS	       BIT5  //485����
#define	RS485BUSEN             BIT6  // ����3 485ʹ��

// P11 ���Ŷ���
#define TACK                   BIT0  // ACLK���Զ�
#define TMCK                   BIT1  // MCLK���Զ�
#define TSMCK                  BIT2  // SMCLK���Զ�

#endif


