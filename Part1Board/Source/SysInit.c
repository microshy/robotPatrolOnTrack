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
#include "Uart.h"
#include "PIN_DEF.h"
#include "SysInit.h"
#include "eeprom.h"

extern unsigned char timer0Flag=0;//定时时间到标志
extern unsigned char ChrgStateNow;
extern unsigned char timer0_B7_Flag=0;
extern unsigned char Electricity;

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
    P7DIR|=LED1;
    //P7OUT |= LED1;
    P7OUT &= ~LED1;
    P7DIR|=LED2;
    P7OUT |= LED2;
    //P7OUT &= ~LED2;
}
/*******************************************************************************
补光灯控制，常亮
*******************************************************************************/
void FillLight(void)
{
    P2DIR |= FL_CTRL;
    P2OUT |= FL_CTRL;
    //P2OUT &= ~FL_CTRL;
}

/*******************************************************************************
工作状态指示灯初始化 Working status indicator
*******************************************************************************/
void Init_WSI(void)
{
    P1DIR |= WSI_G;
    P1OUT |= WSI_G;
    //P1OUT &= ~WSI_G;
    
    P1DIR |= WSI_Y;
    //P1OUT |= WSI_Y;
    P1OUT &= ~WSI_Y;
    
    P1DIR |= WSI_R;
    //P1OUT |= WSI_R;
    P1OUT &= ~WSI_R;
}
/*******************************************************************************
继电器初始化 relay
*******************************************************************************/
void Init_Relay(void)
{    
    P1DIR |= RELAY2;
    //P1OUT |= RELAY2;//开
    P1OUT &= ~RELAY2;//关
    
    P1DIR |= RELAY1;
    //P1OUT |= RELAY1;//开
    P1OUT &= ~RELAY1;//关
}
/*******************************************************************************
超声波串口选择初始化
*******************************************************************************/
void SelectUltrSens()
{
    P8DIR |= USC1;
    P8OUT &= ~USC1;
    
    P8DIR |= USC2;
    P8OUT &= ~USC2;
    
    P8DIR |= USC3;
    P8OUT |= USC3;
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
    TA0CCR0 = 1635-1 ;
    timer0Flag=0;
}
/*******************************************************************************
定时中断
*******************************************************************************/
#pragma vector=TIMER0_B0_VECTOR                             
__interrupt void Timer0_B0 (void)
{
    if(ChrgStateNow==0x01)//充电状态
    {
        timer0_B7_Flag++;
        if(timer0_B7_Flag%2==0)
        {
            if(Electricity>=50)
            {
                P1OUT |= WSI_G;
                P1OUT &= ~WSI_R;
                P1OUT &= ~WSI_Y;
            }
            else if(Electricity>=30)
            {
                P1OUT &= ~WSI_G;
                P1OUT &= ~WSI_R;
                P1OUT |= WSI_Y;
            }
            else
            {
                P1OUT &= ~WSI_G;
                P1OUT |= WSI_R;
                P1OUT &= ~WSI_Y;
            }
            timer0_B7_Flag=0;
        }
        else 
        {
            if(Electricity>=50)
            {
                P1OUT &= ~WSI_G;
                P1OUT &= ~WSI_R;
                P1OUT &= ~WSI_Y;
            }
            else if(Electricity>=30)
            {
                P1OUT &= ~WSI_G;
                P1OUT &= ~WSI_R;
                P1OUT &= ~WSI_Y;
            }
            else
            {
                P1OUT &= ~WSI_G;
                P1OUT &= ~WSI_R;
                P1OUT &= ~WSI_Y;
            }
        }
    }
    //TB0CCTL0 &= ~CCIE ;
}
void Init_Timer0_B7(void)
{
    TB0CTL = TBSSEL_1 +TACLR + MC_1 ;
    TB0CCTL0 = CCIE ;
    TB0CCR0 = 16300-1 ;
    timer0_B7_Flag=0;
}
/*
void Init_Timer0_A5_10Ms(unsigned int ms_10)
{
    TA0CTL = TASSEL_1 +TACLR + MC_1 ;
    TA0CCTL0 = CCIE ;
    TA0CCR0 = 327*ms_10 ;
    timer0Flag=0;
}
*/
/*******************************************************************************
系统初始化
*******************************************************************************/
void SysInit()
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    _DINT();//关总中断
    Init_Relay();
    Init_CLK();
    Init_LED();
    FillLight();
    Init_EEPROM();
    Init_WSI();
    SelectUltrSens();
    //Init_Timer0_A5();
    Init_UART0();
    Init_UART1();
    Init_UART2();
    Init_UART3();
    Init_Timer0_B7();//初始化定时中断
    //TB0CCTL0 &= ~CCIE ;    
}



