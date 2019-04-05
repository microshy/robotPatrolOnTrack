//===========================================================================//
//                                                                           //
// 文件：  Uart.h                                                            //
// 说明：  串口模块                                                          //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//
#ifndef Uart_H
#define Uart_H

//------------------------------- UART1-------------------------------//                            
#define UART1_OUT          P7DIR |=  RS485BUSEN;\
	                   P7OUT &=~  RS485BUSEN ; \
	                   delay_us(5);
	                   
#define UART1_IN           P7DIR |=  RS485BUSEN;\
	                   P7OUT |=  RS485BUSEN ; \
	                   delay_us(5);

//------------------------------- UART2-------------------------------//                            
#define UART2_OUT          P9DIR |= RS485_ENCODER1EN;\
	                   P9OUT &=~ RS485_ENCODER1EN ; \
	                   delay_us(5);
	                   
#define UART2_IN           P9DIR |= RS485_ENCODER1EN;\
	                   P9OUT |= RS485_ENCODER1EN ; \
	                   delay_us(5);
                           
//------------------------------- UART3-------------------------------//                            
#define UART3_OUT          P10DIR |= RS485_ENCODER2EN;\
	                   P10OUT &=~ RS485_ENCODER2EN ; \
	                   delay_us(5);
	                   
#define UART3_IN           P10DIR |= RS485_ENCODER2EN;\
	                   P10OUT |= RS485_ENCODER2EN ; \
	                   delay_us(5);


void Init_UART1(void);
void Init_UART2(void);
void Init_UART3(void);
void Uart1_send(unsigned char *tx_buf,unsigned char length);
void Uart2_send(unsigned char *tx_buf,unsigned char length);
void Uart3_send(unsigned char *tx_buf,unsigned char length);

unsigned char CRC(unsigned char *buffer, int length);
unsigned int CRC16RTU( unsigned char * pszBuf, unsigned int unLength);
unsigned char Uart1ExtrInstr(unsigned char *buffer ,int length);

#endif
                           