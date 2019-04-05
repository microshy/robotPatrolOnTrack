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
//------------------------------- UART2-------------------------------//                            
#define UART2_OUT          P9DIR |= RS485BUSEN;\
	                   P9OUT &=~ RS485BUSEN ; \
	                   delay_us(5);
	                   
#define UART2_IN           P9DIR |= RS485BUSEN;\
	                   P9OUT |= RS485BUSEN ; \
	                   delay_us(5);
                           

void Init_UART1(void);
void Init_UART2(void);
void Uart1_send(unsigned char *tx_buf,unsigned char length);
void Uart2_send(unsigned char *tx_buf,unsigned char length);

unsigned char CRC(unsigned char *buffer, int length);
unsigned int CRC16RTU( unsigned char * pszBuf, unsigned int unLength);
unsigned char Uart2ExtrInstr(unsigned char *buffer ,int length);

#endif
                           