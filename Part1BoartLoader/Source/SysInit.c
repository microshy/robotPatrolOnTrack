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
#include "SysInit.h"
#include "Uart.h"
#include "eeprom.h"

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

//****************************************************************************
//                                                                           *
//                       LEDtest                                             *
//                                                                           *
//****************************************************************************
void Init_LED(void)
{
    P7DIR|=BIT6;
    //P7OUT &= ~BIT6;
    P7OUT |= BIT6;
    P7DIR|=BIT7;
    //P7OUT &= ~BIT7;
    P7OUT |= BIT7;
}

/*******************************************************************************
ϵͳ��ʼ��
*******************************************************************************/
void SysInit()
{
  /*
    P1DIR |= BIT6;
    P1OUT |= BIT6;
    P1DIR |= BIT7;
    P1OUT |= BIT7;
  */
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    _DINT();//�����ж�
    
    Init_CLK();
    Init_LED();
    Init_EEPROM();
    Init_UART1();
    //_EINT();         //�����ж�
}



