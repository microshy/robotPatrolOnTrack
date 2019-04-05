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
extern unsigned int timer0Count=0;//定时时间到标志

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
    P4DIR|=LED1;
    P4OUT &= ~LED1;
    P4DIR|=LED2;
    //P4OUT |= LED2;
    P4OUT &= ~LED2;
}
/*******************************************************************************
继电器初始化
*******************************************************************************/
void Init_Relay()
{
    P4DIR |= RELAY1;
    //P4OUT |= RELAY1;//充电器
    P4OUT &= ~RELAY1;
    
    P4DIR |= RELAY2;
    P4OUT |= RELAY2;
    //P4OUT &= ~RELAY2;
    
    P3DIR |= RELAY3;
    P3OUT |= RELAY3;
    //P3OUT &= ~RELAY3;
    
    P3DIR |= RELAY4;
    P3OUT |= RELAY4;//给两个电极供电
    //P3OUT &= ~RELAY4;
    
    //P3DIR |= RELAY5;
    //P3OUT |= RELAY5;//给运动控制板供电
    //P3OUT &= ~RELAY5;
}
 //***************************************************************************
//                                                                           *
//                       ADC初始化                                      *
//                                                                           *
//****************************************************************************
void Init_ADC(void)
{
  /*
    // 只有在ADC12ENC复位的情况下才可以操作  
    // ADC12SHT1X ADC12SHT0X ADC12MSC ADC12REF2_5V ADC12REFON ADC12ON  
    ADC12CTL0 &= ~ADC12ENC;  
  
    // 设置采样保持时间，最大时间周期以提高转换精度  
    // 注意MSP430F5438没有REF模块，片内基准无效 
   REFCTL0 |= REFMSTR+REFVSEL_2+REFON+REFTCOFF; 
   REFCTL0|=REFMSTR+REFVSEL_2+REFON;
    // 操作ADC12REF2_5V ，ADC12REFON并无意义  
    //ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON;  
    ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON + ADC12REF2_5V + ADC12REFON;  
    // 采样保持脉冲来自采样定时器  
    ADC12CTL1 = ADC12SHP;  
    // 关闭内部内部温度检测以降低功耗，注意或操作否则修改转换精度  
    ADC12CTL2 |= ADC12TCOFF ;  
    // 基准电压选择AVCC，并选择1通道——(AVCC-AVSS)/2  
    //ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_2; 
    ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_2;
  
    __delay_cycles(75);  
    // ADC12使能  
    ADC12CTL0 |= ADC12ENC;  
  */
  P6SEL |= BIT4;
  P6SEL |= BIT5;
  P6SEL |= BIT6;
  ADC12CTL0 = ADC12ON; // 使能转换模块

REFCTL0|=REFMSTR+REFVSEL_2+REFON;

//使能REF管理，内部参考电压选择2.5v、打开内部参考电压

ADC12CTL1 = ADC12SHP;// // 选择脉冲触发模式、单通道单次次转换模式

ADC12MCTL0= ADC12SREF_1+ADC12INCH_1; //选择参考电压源、现在a5通道

ADC12CTL0 |= ADC12ENC; // 使能转换模块

}
/*******************************************************************************
定时器初始化
*******************************************************************************/
#pragma vector=TIMER0_B0_VECTOR                             
__interrupt void Timer0_B0 (void)
{
    timer0Flag=1;    
    TB0CCTL0 &= ~CCIE ;
}
void Init_Timer0_B7(void)
{
    TB0CTL = TBSSEL_1 +TACLR + MC_1 ;
    TB0CCTL0 = CCIE ;
    TB0CCR0 = 650-1 ;
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
    Init_UART1();
    Init_Relay();
    Init_ADC();
    //Init_Timer0_B7();
    
    
    //_EINT();         //开总中断
}

