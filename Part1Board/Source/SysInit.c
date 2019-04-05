//===========================================================================//
//                                                                           //
// �ļ���  SysInit.c                                                         //
// ˵����  ϵͳӲ����ʼ��                                                    //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
//                                                                           //
//===========================================================================//

#include "msp430f5438a.h"
#include "Uart.h"
#include "PIN_DEF.h"
#include "SysInit.h"
#include "eeprom.h"

extern unsigned char timer0Flag=0;//��ʱʱ�䵽��־
extern unsigned char ChrgStateNow;
extern unsigned char timer0_B7_Flag=0;
extern unsigned char Electricity;

//****************************************************************************
//                                                                           *
//              ʱ�ӳ�ʼ����MCLK=SMCLK=XT2=8MHZ,ACLK=XT1=32768hz             *
//                                                                           *
//****************************************************************************

void Init_CLK(void)
{
  WDTCTL     = WDTPW + WDTHOLD ; // �ؿ��Ź�
  P5SEL     |= 0x0C ; // P5.2 P5.3�˿�ѡ���ⲿ����XT2
  P7SEL     |= 0x03  ; // P7.0 P7.1�˿�ѡ���ⲿ��Ƶ����XT1
  UCSCTL6   &=~XT1OFF ; // ʹ���ⲿ���� XT1
  UCSCTL6   |= XCAP_3 ; // �����ڲ����ص���
  UCSCTL6   &= ~XT2OFF ; // XT2ʹ��
  
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG); // ��� XT2,XT1,DCO �����־                                                          
    SFRIFG1 &= ~OFIFG ; 
  }while(SFRIFG1&OFIFG); // ������������־
  UCSCTL6   |=  XT2DRIVE1 ; // XT2 ����ģʽ 8~16MHz                                            
  UCSCTL4   |= SELS_5 + SELM_5 + SELA_0; // SMCLK = MCLK = XT2  ACLK = XT1
 }
void CLK_Test(void)
 {
	 Init_CLK() ;
	 P11DS  = TACK + TMCK + TSMCK ; // ѡ������ǿ��
	 P11SEL = TACK + TMCK + TSMCK ; // ѡ�����Ź��ܣ�ACK��MCK��SMCK�����P11.0/1/2
	 P11DIR = TACK + TMCK + TSMCK ; // ��������IO����Ϊ���
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
����ƿ��ƣ�����
*******************************************************************************/
void FillLight(void)
{
    P2DIR |= FL_CTRL;
    P2OUT |= FL_CTRL;
    //P2OUT &= ~FL_CTRL;
}

/*******************************************************************************
����״ָ̬ʾ�Ƴ�ʼ�� Working status indicator
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
�̵�����ʼ�� relay
*******************************************************************************/
void Init_Relay(void)
{    
    P1DIR |= RELAY2;
    //P1OUT |= RELAY2;//��
    P1OUT &= ~RELAY2;//��
    
    P1DIR |= RELAY1;
    //P1OUT |= RELAY1;//��
    P1OUT &= ~RELAY1;//��
}
/*******************************************************************************
����������ѡ���ʼ��
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
��ʱ����ʼ��
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
��ʱ�ж�
*******************************************************************************/
#pragma vector=TIMER0_B0_VECTOR                             
__interrupt void Timer0_B0 (void)
{
    if(ChrgStateNow==0x01)//���״̬
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
ϵͳ��ʼ��
*******************************************************************************/
void SysInit()
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    _DINT();//�����ж�
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
    Init_Timer0_B7();//��ʼ����ʱ�ж�
    //TB0CCTL0 &= ~CCIE ;    
}



