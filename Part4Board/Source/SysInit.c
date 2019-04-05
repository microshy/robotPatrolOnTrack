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
extern unsigned char moveDir;
extern long LlocationX;
extern unsigned char coordFail=0x00;
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
    P2DIR|=LED1;
    P2OUT &= ~LED1;
    P2DIR|=LED2;
    P2OUT |= LED2;
    //P2OUT &= ~LED2;
}
/*******************************************************************************
接近开关初始化
*******************************************************************************/
void Init_JJSwitch()
{
    P2IES &= ~JJ1 ;  //p2.6 接近开关1，上升沿触发
    P2IE |= JJ1 ;
    P2IFG &= ~JJ1 ;
    
    //P3DIR &= ~JJ1;
    //P3DIR &= ~JJ2;
}
/*******************************************************************************
PORT2中断处理函数，2.6,7接近开关
*******************************************************************************/
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    if((P2IFG&JJ1)==JJ1)//编码器计数中断
    {
        //delay_ms(5);
        P2IFG &= ~JJ1;//清中断标志位
        if((P2IN&JJ1)!=JJ1)//低电平，即下降沿
        {
          /*
            //需要判断小车运动方向
            if(moveDir==0x01)//小车向前运动
            {
             // if((LlocationX-2003>10)||(LlocationX-2003<-10))
               // coordFail=0x01;
              P1OUT |= DCMSTOP1 ;//小车电机停止 
              TA0CCR4=4000;
              
            }
            if(moveDir==0x02)//小车向后运动
            {
              //if((LlocationX-2027>10)||(LlocationX-2027<-10))
                //coordFail=0x01;
              P1OUT |= DCMSTOP1 ;//小车电机停止 
              TA0CCR4=4000;
            }
          */
        }
        //delay_ms(20);
    }
    
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
小车驱动器初始化，PWM生成
*******************************************************************************/
void Init_Timer0_A5(void)
{  
    TA0CTL = TASSEL_2 + TACLR + MC_1 ; //时钟源为SMCLK=8M，增计数模式
    TA0CCR0 = 8000 - 1 ;//频率=8 000 000/8 000
    
    TA0CCTL1 = OUTMOD_6 ; // 伸缩杆电机驱动PWM
    TA0CCR1=4000 ;

    TA0CCTL4 = OUTMOD_6 ; // 小车电机驱动PWM
    TA0CCR4=4000 ;
    
    P1SEL |= DCMPWM2 ; //引脚定义为比较器，输出PWM信号给电机驱动
    P1DIR |= DCMPWM2 ; 
    
    P1SEL |= DCMPWM1 ; //引脚定义为比较器，输出PWM信号给电机驱动
    P1DIR |= DCMPWM1 ; 
    
    P1DIR |= DCMEN1 ;
    P1OUT &= ~DCMEN1 ;//电机使能
    P1DIR |= DCMEN2 ;
    P1OUT &= ~DCMEN2 ;//电机使能
    
    delay_ms(1000);
    
    P1OUT |= DCMEN1 ;//电机使能
    P1OUT |= DCMEN2 ;//电机使能
    
    P1DIR |= DCMSTOP1 ;
    P1OUT |= DCMSTOP1 ;//电机停止
    P1DIR |= DCMSTOP2 ;
    P1OUT |= DCMSTOP2 ;//电机停止
    
    //P2DIR &= ~DCMRE1 ;
    //P2DIR &= ~DCMRE2 ;
    
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
    Init_JJSwitch();
    Init_Timer0_A5();
    Init_UART1();
    Init_UART2();
    Init_UART3();
    
    //_EINT();         //开总中断
}

