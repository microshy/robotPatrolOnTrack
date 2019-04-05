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
//------------------------------- UART2-------------------------------//                            
#define UART2_OUT          P9DIR |= RS485EN_JF;\
	                   P9OUT &=~ RS485EN_JF ; \
	                   delay_us(5);
	                   
#define UART2_IN           P9DIR |= RS485EN_JF;\
	                   P9OUT |= RS485EN_JF ; \
	                   delay_us(5);
                           
//------------------------------- UART3-------------------------------//                            
#define UART3_OUT          P10DIR |= RS485BUSEN;\
	                   P10OUT &=~ RS485BUSEN ; \
	                   delay_us(5);
	                   
#define UART3_IN           P10DIR |= RS485BUSEN;\
	                   P10OUT |= RS485BUSEN ; \
	                   delay_us(5);


void Init_UART0(void);
void Init_UART1(void);
void Init_UART2(void);
void Init_UART3(void);
void Uart0_send(unsigned char *tx_buf,unsigned int length);
void Uart1_send(unsigned char *tx_buf,unsigned int length);
void Uart2_send(unsigned char *tx_buf,unsigned int length);
void Uart3_send(unsigned char *tx_buf,unsigned int length);

unsigned char CRC(unsigned char *buffer, int length);
unsigned int CRC16RTU( unsigned char * pszBuf, unsigned int unLength);
unsigned char Uart1ExtrInstr(unsigned char *buffer ,int length);
unsigned char Uart3ExtrInstr(unsigned char *buffer ,int length);

#endif
                           