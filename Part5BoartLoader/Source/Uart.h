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

/*****************************************************************************
函数名称：延时函数
功    能：进行精度为1us和1ms的延时
备    注：视MCLK频率而定，MCLK=16M则CPU_F=16000000，若MCLK=8M则CPU_F=8000000
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
                           