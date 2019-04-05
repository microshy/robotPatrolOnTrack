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
extern unsigned int timer0Count=0;//��ʱʱ�䵽��־
extern unsigned char moveDir;
extern long LlocationX;
extern unsigned char coordFail=0x00;
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
    P2DIR|=LED1;
    P2OUT &= ~LED1;
    P2DIR|=LED2;
    P2OUT |= LED2;
    //P2OUT &= ~LED2;
}
/*******************************************************************************
�ӽ����س�ʼ��
*******************************************************************************/
void Init_JJSwitch()
{
    P2IES &= ~JJ1 ;  //p2.6 �ӽ�����1�������ش���
    P2IE |= JJ1 ;
    P2IFG &= ~JJ1 ;
    
    //P3DIR &= ~JJ1;
    //P3DIR &= ~JJ2;
}
/*******************************************************************************
PORT2�жϴ�������2.6,7�ӽ�����
*******************************************************************************/
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    if((P2IFG&JJ1)==JJ1)//�����������ж�
    {
        //delay_ms(5);
        P2IFG &= ~JJ1;//���жϱ�־λ
        if((P2IN&JJ1)!=JJ1)//�͵�ƽ�����½���
        {
          /*
            //��Ҫ�ж�С���˶�����
            if(moveDir==0x01)//С����ǰ�˶�
            {
             // if((LlocationX-2003>10)||(LlocationX-2003<-10))
               // coordFail=0x01;
              P1OUT |= DCMSTOP1 ;//С�����ֹͣ 
              TA0CCR4=4000;
              
            }
            if(moveDir==0x02)//С������˶�
            {
              //if((LlocationX-2027>10)||(LlocationX-2027<-10))
                //coordFail=0x01;
              P1OUT |= DCMSTOP1 ;//С�����ֹͣ 
              TA0CCR4=4000;
            }
          */
        }
        //delay_ms(20);
    }
    
}
/*******************************************************************************
��ʱ����ʼ��
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
С����������ʼ����PWM����
*******************************************************************************/
void Init_Timer0_A5(void)
{  
    TA0CTL = TASSEL_2 + TACLR + MC_1 ; //ʱ��ԴΪSMCLK=8M��������ģʽ
    TA0CCR0 = 8000 - 1 ;//Ƶ��=8 000 000/8 000
    
    TA0CCTL1 = OUTMOD_6 ; // �����˵������PWM
    TA0CCR1=4000 ;

    TA0CCTL4 = OUTMOD_6 ; // С���������PWM
    TA0CCR4=4000 ;
    
    P1SEL |= DCMPWM2 ; //���Ŷ���Ϊ�Ƚ��������PWM�źŸ��������
    P1DIR |= DCMPWM2 ; 
    
    P1SEL |= DCMPWM1 ; //���Ŷ���Ϊ�Ƚ��������PWM�źŸ��������
    P1DIR |= DCMPWM1 ; 
    
    P1DIR |= DCMEN1 ;
    P1OUT &= ~DCMEN1 ;//���ʹ��
    P1DIR |= DCMEN2 ;
    P1OUT &= ~DCMEN2 ;//���ʹ��
    
    delay_ms(1000);
    
    P1OUT |= DCMEN1 ;//���ʹ��
    P1OUT |= DCMEN2 ;//���ʹ��
    
    P1DIR |= DCMSTOP1 ;
    P1OUT |= DCMSTOP1 ;//���ֹͣ
    P1DIR |= DCMSTOP2 ;
    P1OUT |= DCMSTOP2 ;//���ֹͣ
    
    //P2DIR &= ~DCMRE1 ;
    //P2DIR &= ~DCMRE2 ;
    
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
    Init_JJSwitch();
    Init_Timer0_A5();
    Init_UART1();
    Init_UART2();
    Init_UART3();
    
    //_EINT();         //�����ж�
}

