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
    P4DIR|=LED1;
    P4OUT &= ~LED1;
    P4DIR|=LED2;
    //P4OUT |= LED2;
    P4OUT &= ~LED2;
}
/*******************************************************************************
�̵�����ʼ��
*******************************************************************************/
void Init_Relay()
{
    P4DIR |= RELAY1;
    //P4OUT |= RELAY1;//�����
    P4OUT &= ~RELAY1;
    
    P4DIR |= RELAY2;
    P4OUT |= RELAY2;
    //P4OUT &= ~RELAY2;
    
    P3DIR |= RELAY3;
    P3OUT |= RELAY3;
    //P3OUT &= ~RELAY3;
    
    P3DIR |= RELAY4;
    P3OUT |= RELAY4;//�������缫����
    //P3OUT &= ~RELAY4;
    
    //P3DIR |= RELAY5;
    //P3OUT |= RELAY5;//���˶����ư幩��
    //P3OUT &= ~RELAY5;
}
 //***************************************************************************
//                                                                           *
//                       ADC��ʼ��                                      *
//                                                                           *
//****************************************************************************
void Init_ADC(void)
{
  /*
    // ֻ����ADC12ENC��λ������²ſ��Բ���  
    // ADC12SHT1X ADC12SHT0X ADC12MSC ADC12REF2_5V ADC12REFON ADC12ON  
    ADC12CTL0 &= ~ADC12ENC;  
  
    // ���ò�������ʱ�䣬���ʱ�����������ת������  
    // ע��MSP430F5438û��REFģ�飬Ƭ�ڻ�׼��Ч 
   REFCTL0 |= REFMSTR+REFVSEL_2+REFON+REFTCOFF; 
   REFCTL0|=REFMSTR+REFVSEL_2+REFON;
    // ����ADC12REF2_5V ��ADC12REFON��������  
    //ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON;  
    ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON + ADC12REF2_5V + ADC12REFON;  
    // ���������������Բ�����ʱ��  
    ADC12CTL1 = ADC12SHP;  
    // �ر��ڲ��ڲ��¶ȼ���Խ��͹��ģ�ע�����������޸�ת������  
    ADC12CTL2 |= ADC12TCOFF ;  
    // ��׼��ѹѡ��AVCC����ѡ��1ͨ������(AVCC-AVSS)/2  
    //ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_2; 
    ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_2;
  
    __delay_cycles(75);  
    // ADC12ʹ��  
    ADC12CTL0 |= ADC12ENC;  
  */
  P6SEL |= BIT4;
  P6SEL |= BIT5;
  P6SEL |= BIT6;
  ADC12CTL0 = ADC12ON; // ʹ��ת��ģ��

REFCTL0|=REFMSTR+REFVSEL_2+REFON;

//ʹ��REF�����ڲ��ο���ѹѡ��2.5v�����ڲ��ο���ѹ

ADC12CTL1 = ADC12SHP;// // ѡ�����崥��ģʽ����ͨ�����δ�ת��ģʽ

ADC12MCTL0= ADC12SREF_1+ADC12INCH_1; //ѡ��ο���ѹԴ������a5ͨ��

ADC12CTL0 |= ADC12ENC; // ʹ��ת��ģ��

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
    Init_UART1();
    Init_Relay();
    Init_ADC();
    //Init_Timer0_B7();
    
    
    //_EINT();         //�����ж�
}

