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
#include "PIN_DEF.h"
#include "SysInit.h"
#include "eeprom.h"
#include "Uart.h"

extern unsigned char timer0Flag=0;//��ʱʱ�䵽��־
extern int timer0Count=0;//��ʱʱ�䵽��־

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
    P1DIR|=LED1;
    P1OUT &= ~LED1;
    //P1OUT |= LED1;
    
    P1DIR|=LED2;
    P1OUT &= ~LED2;
    //P1OUT |= LED2;
}
/*******************************************************************************
���ȳ�ʼ�� 
*******************************************************************************/
void Init_FAN(void)
{
    P1DIR |= FAN1_CTRL;
    //P1OUT |= FAN1_CTRL;
    P1OUT &= ~FAN1_CTRL;
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
    TA0CCR0 = 650-1 ;
    timer0Flag=0;
}


/*******************************************************************************
ϵͳ��ʼ��
*******************************************************************************/
void SysInit()
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    _DINT();//�����ж�
    
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



