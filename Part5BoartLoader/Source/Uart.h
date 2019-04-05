//===========================================================================//
//                                                                           //
// �ļ���  Uart.h                                                            //
// ˵����  ����ģ��                                                          //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
//                                                                           //
//===========================================================================//
#ifndef Uart_H
#define Uart_H

/*****************************************************************************
�������ƣ���ʱ����
��    �ܣ����о���Ϊ1us��1ms����ʱ
��    ע����MCLKƵ�ʶ�����MCLK=16M��CPU_F=16000000����MCLK=8M��CPU_F=8000000
******************************************************************************/
#define CPU_F ((double)8000000) 
#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0)) 
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0))


//------------------------------- UART1-------------------------------//                            
#define UART1_OUT          P5DIR |=  BIT5;\
	                   P5OUT &=~  BIT5 ; \
	                   delay_us(5);
	                   
#define UART1_IN           P5DIR |=  BIT5;\
	                   P5OUT |=  BIT5 ; \
	                   delay_us(5);


void Init_UART1(void);
void Uart1_send(unsigned char *tx_buf,unsigned char length);

unsigned char CRC(unsigned char *buffer, int length);
unsigned char Uart1ExtrInstr(unsigned char *buffer ,int length);

#endif
                           