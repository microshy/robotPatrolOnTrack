//===========================================================================//
//                                                                           //
// 文件：  SysInit.c                                                         //
// 说明：  系统硬件初始化                                                    //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//

#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "SysInit.h"
#include "eeprom.h"
#include "Uart.h"

extern unsigned char timer0Flag=0;//定时时间到标志
extern int timer0Count=0;//定时时间到标志

//****************************************************************************
//                                                                           *
//              时钟初始化：MCLK=SMCLK=XT2=8MHZ,ACLK=XT1=32768hz             *
//                                                                           *
//****************************************************************************

void Init_CLK(void)
{
  WDTCTL     = WDTPW + WDTHOLD ; // 关看门狗
  P5SEL     |= 0x0C ; // P5.2 P5.3端口选择外部晶振XT2
  P7SEL     |= 0x03  ; // P7.0 P7.1端口选择外部低频晶振XT1
  UCSCTL6   &=~XT1OFF ; // 使能外部晶振 XT1
  UCSCTL6   |= XCAP_3 ; // 设置内部负载电容
  UCSCTL6   &= ~XT2OFF ; // XT2使能
  
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG); // 清除 XT2,XT1,DCO 错误标志                                                          
    SFRIFG1 &= ~OFIFG ; 
  }while(SFRIFG1&OFIFG); // 检测振荡器错误标志
  UCSCTL6   |=  XT2DRIVE1 ; // XT2 驱动模式 8~16MHz                                            
  UCSCTL4   |= SELS_5 + SELM_5 + SELA_0; // SMCLK = MCLK = XT2  ACLK = XT1
 }
void CLK_Test(void)
 {
	 Init_CLK() ;
	 P11DS  = TACK + TMCK + TSMCK ; // 选择驱动强度
	 P11SEL = TACK + TMCK + TSMCK ; // 选择引脚功能，ACK、MCK、SMCK输出至P11.0/1/2
	 P11DIR = TACK + TMCK + TSMCK ; // 设置引脚IO方向为输出
 }

//****************************************************************************
//                                                                           *
//                       LEDtest                                             *
//                                                                           *
//****************************************************************************
void Init_LED(void)
{
    P1DIR|=LED1;
    P1OUT &= ~LED1;
    //P1OUT |= LED1;
    
    P1DIR|=LED2;
    P1OUT &= ~LED2;
    //P1OUT |= LED2;
}
/*******************************************************************************
风扇初始化 
*******************************************************************************/
void Init_FAN(void)
{
    P1DIR |= FAN1_CTRL;
    //P1OUT |= FAN1_CTRL;
    P1OUT &= ~FAN1_CTRL;
}

/*******************************************************************************
定时器初始化
*******************************************************************************/

#pragma vector=TIMER0_A0_VECTOR                             
__interrupt void Timer0_A0 (void)
{
    timer0Flag=1;
    //TA0CCR0  = (650) - 1;
    TA0CCTL0 &= ~CCIE ;
}

void Init_Timer0_A5(void)
{
    TA0CTL = TASSEL_1 +TACLR + MC_1 ;
    TA0CCTL0 = CCIE ;
    TA0CCR0 = 650-1 ;
    timer0Flag=0;
}


/*******************************************************************************
系统初始化
*******************************************************************************/
void SysInit()
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    _DINT();//关总中断
    
    Init_CLK();
    Init_LED();
    Init_EEPROM();
    Init_LED();
    Init_FAN();
    //Init_Timer0_A5();
    Init_UART0();
    Init_UART1();
    Init_UART2();
    Init_UART3();
    timer0Count=0;
    
}



