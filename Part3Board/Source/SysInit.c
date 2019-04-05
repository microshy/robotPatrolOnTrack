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
#include "Uart.h"
#include "eeprom.h"

extern unsigned char timer0Flag=0;//定时时间到标志
extern unsigned int timer0Count=0;//定时时间到标志
extern int count_BM=0;//编码器计数
extern unsigned char encoder_status_Z=0x00;//编码器状态
extern long servo_X_PWM=7500;//X舵机的PWM给定值
extern long servo_Y_PWM=12000;//Y舵机的PWM给定值
extern unsigned char movetoZ_status;
extern unsigned char movetoZ_flag;//电动推杆移动到指令标志
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
    P3DIR|=LED1;
    P3OUT &= ~LED1;
    P3DIR|=LED2;
    //P3OUT |= LED2;
    P3OUT &= ~LED2;
}
/*******************************************************************************
风扇初始化 
*******************************************************************************/
void Init_FAN(void)
{
    P3DIR |= FAN2_CTRL;
    //P3OUT |= FAN2_CTRL;
    P3OUT &= ~FAN2_CTRL;
}
/*******************************************************************************
限位开关初始化，中断
*******************************************************************************/
void Init_linearActuator(void)
{
    P2IES &= ~LS1_LA ;  //p2.0 电缸限位1，上升沿触发
    P2IE |= LS1_LA ;
    P2IFG &= ~LS1_LA ;
    
    P1IES &= ~LS2_LA ;  //p1.6 电缸限位2，上升沿触发
    P1IE |= LS2_LA ;
    P1IFG &= ~LS2_LA ;
    
    P1IES &= ~LS3_LA ;  //p1.4 电缸限位3，上升沿触发
    P1IE |= LS3_LA ;
    P1IFG &= ~LS3_LA ;
    
    P1IES &= ~LS2_SERVO ;  //p1.7 云台限位2，上升沿触发
    P1IE |= LS2_SERVO ;
    P1IFG &= ~LS2_SERVO ;
    
    P1IES &= ~LS1_SERVO ;  //p1.5 云台限位1，上升沿触发
    P1IE |= LS1_SERVO ;
    P1IFG &= ~LS1_SERVO ;
}
void Init_Encoder(void)
{
  
    P2IES |= COUNT_ENCODER ;  //p2.5 编码器计数，上升沿触发
    P2IE |= COUNT_ENCODER ;
    P2IFG &= ~COUNT_ENCODER ;
    
    P2DIR &= ~DIR_ENCODER ;//P2.4 编码器方向
}

/*******************************************************************************
PORT1中断处理函数，1.4/5/6 电动推杆限位中断，1.7云台限位中断 
*******************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    //_EINT();         //开总中断，中断嵌套
    
    if((P1IFG&LS2_LA)==LS2_LA)//局放轴前限位
    {
        P1IFG &= ~LS2_LA ;
        if((P2IN&DIR_ENCODER)!=DIR_ENCODER)//判断方向，向前伸是低
        {
            TA0CCR2=0;//停
            P1OUT |= LA_IN1 ;//高
            P1OUT |= LA_IN2 ;//高
            movetoZ_flag = 0;
            movetoZ_status = 1;                                                //1，认为移动到位   ；
        }
        
    }
    if((P1IFG&LS3_LA)==LS3_LA)//局放轴后限位
    {
        P1IFG &= ~LS3_LA ;
        /*
        count_BM=0;
        P1OUT |= DCME ;//高
        P1OUT |= DCMS ;//高
        TA0CCR2=0 ;//占空比0%
        */
        TA0CCR2=0;//停
        P1OUT |= LA_IN1 ;//高
        P1OUT |= LA_IN2 ;//高
        
        count_BM=0;
    }
    if((P1IFG&LS2_SERVO)==LS2_SERVO)
    {
        P1IFG &= ~LS2_SERVO ;
        
    }
}
/*******************************************************************************
PORT2中断处理函数，2.0伸缩杆限位，2.4编码器计数中断
*******************************************************************************/
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    //_EINT();         //开总中断，中断嵌套
  
    if((P2IFG&COUNT_ENCODER)==COUNT_ENCODER)//编码器计数中断
    {
        P2IFG &= ~COUNT_ENCODER;//清中断标志位
        if((P2IN&DIR_ENCODER)==DIR_ENCODER)//判断方向
        {       
            count_BM--;
        }
        if((P2IN&DIR_ENCODER)!=DIR_ENCODER)//判断方向，向前伸是低
        {
            count_BM++;
            if(count_BM>1200)
            {
                TA0CCR2=0 ;//占空比0%
                P1OUT |= LA_IN1 ;//高
                P1OUT |= LA_IN2 ;//高
            }
        }
       // else{}
        
    }
    if((P2IFG&LS1_LA)==LS1_LA)//局放轴前限位
    {
        P2IFG &= ~LS1_LA;//清中断标志位
        if((P2IN&DIR_ENCODER)!=DIR_ENCODER)//判断方向，向前伸是低
        {
            TA0CCR2=0;//停
            P1OUT |= LA_IN1 ;//高
            P1OUT |= LA_IN2 ;//高
            movetoZ_flag = 0;
            movetoZ_status = 1;                                                //1，认为移动到位   ；
        }
        
    }

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
#pragma vector=TIMER0_B0_VECTOR                             
__interrupt void Timer0_B0 (void)
{
    timer0Flag=1;
    //TA0CCR0  = (650) - 1;
    
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
电缸PWM生成
*******************************************************************************/
void Init_Timer0_A5(void)
{
    TA0CTL = TASSEL_2 + TACLR + MC_1 ; //时钟源为SMCLK=8M，增计数模式
    TA0CCR0 = 4000 - 1 ;//频率=8 000 000/8 000
    
    TA0CCTL2 = OUTMOD_6 ; // left motor pwm
    TA0CCR2=0 ;
    
    P1SEL |= BIT3 ; //引脚定义为比较器，输出PWM信号给电缸驱动
    P1DIR |= BIT3 ; 
    
    P1DIR |= BIT1 ; //电缸方向控制管脚1
    P1OUT |= BIT1 ; 
    
    P1DIR |= BIT2 ; //电缸方向控制管脚2
    P1OUT |= BIT2 ; 
 }
/*******************************************************************************
云台PWM生成
*******************************************************************************/
void Init_Timer1_A3(void)
{
    TA1CTL = TASSEL_2 + TACLR + MC_1 ; //时钟源为SMCLK=8M，增计数模式
    TA1CCR0 = 32000 - 1 ;//频率=8 000 000/8 000
    
    TA1CCTL1 = OUTMOD_6 ; // 云台舵机 pwm
    TA1CCR1=servo_Y_PWM ;//减小向下，增大向上 12000
    TA1CCTL2 = OUTMOD_6 ; // 云台舵机 pwm
    TA1CCR2=servo_X_PWM ;//减小向右，增大向左  7500
    
    P2SEL |= BIT2 ; //引脚定义为比较器，输出PWM信号给电机驱动
    P2DIR |= BIT2 ; 
    P2SEL |= BIT3 ; //引脚定义为比较器，输出PWM信号给电机驱动
    P2DIR |= BIT3 ;
 }

void SysInit()
{
    servo_X_PWM=7500;
    servo_Y_PWM=12000;
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    _DINT();//关总中断
    Init_CLK();
    CLK_Test();
    Init_LED();
    Init_FAN();
    Init_linearActuator();
    Init_Encoder();
    SelectUltrSens();
    Init_Timer0_A5();
    Init_Timer1_A3();
    Init_UART1();
    Init_UART2();
    Init_EEPROM();
    //_EINT();         //开总中断
}

