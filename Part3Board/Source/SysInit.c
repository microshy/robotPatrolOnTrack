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
#include "Uart.h"
#include "eeprom.h"

extern unsigned char timer0Flag=0;//��ʱʱ�䵽��־
extern unsigned int timer0Count=0;//��ʱʱ�䵽��־
extern int count_BM=0;//����������
extern unsigned char encoder_status_Z=0x00;//������״̬
extern long servo_X_PWM=7500;//X�����PWM����ֵ
extern long servo_Y_PWM=12000;//Y�����PWM����ֵ
extern unsigned char movetoZ_status;
extern unsigned char movetoZ_flag;//�綯�Ƹ��ƶ���ָ���־
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
    P3DIR|=LED1;
    P3OUT &= ~LED1;
    P3DIR|=LED2;
    //P3OUT |= LED2;
    P3OUT &= ~LED2;
}
/*******************************************************************************
���ȳ�ʼ�� 
*******************************************************************************/
void Init_FAN(void)
{
    P3DIR |= FAN2_CTRL;
    //P3OUT |= FAN2_CTRL;
    P3OUT &= ~FAN2_CTRL;
}
/*******************************************************************************
��λ���س�ʼ�����ж�
*******************************************************************************/
void Init_linearActuator(void)
{
    P2IES &= ~LS1_LA ;  //p2.0 �����λ1�������ش���
    P2IE |= LS1_LA ;
    P2IFG &= ~LS1_LA ;
    
    P1IES &= ~LS2_LA ;  //p1.6 �����λ2�������ش���
    P1IE |= LS2_LA ;
    P1IFG &= ~LS2_LA ;
    
    P1IES &= ~LS3_LA ;  //p1.4 �����λ3�������ش���
    P1IE |= LS3_LA ;
    P1IFG &= ~LS3_LA ;
    
    P1IES &= ~LS2_SERVO ;  //p1.7 ��̨��λ2�������ش���
    P1IE |= LS2_SERVO ;
    P1IFG &= ~LS2_SERVO ;
    
    P1IES &= ~LS1_SERVO ;  //p1.5 ��̨��λ1�������ش���
    P1IE |= LS1_SERVO ;
    P1IFG &= ~LS1_SERVO ;
}
void Init_Encoder(void)
{
  
    P2IES |= COUNT_ENCODER ;  //p2.5 �����������������ش���
    P2IE |= COUNT_ENCODER ;
    P2IFG &= ~COUNT_ENCODER ;
    
    P2DIR &= ~DIR_ENCODER ;//P2.4 ����������
}

/*******************************************************************************
PORT1�жϴ�������1.4/5/6 �綯�Ƹ���λ�жϣ�1.7��̨��λ�ж� 
*******************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    //_EINT();         //�����жϣ��ж�Ƕ��
    
    if((P1IFG&LS2_LA)==LS2_LA)//�ַ���ǰ��λ
    {
        P1IFG &= ~LS2_LA ;
        if((P2IN&DIR_ENCODER)!=DIR_ENCODER)//�жϷ�����ǰ���ǵ�
        {
            TA0CCR2=0;//ͣ
            P1OUT |= LA_IN1 ;//��
            P1OUT |= LA_IN2 ;//��
            movetoZ_flag = 0;
            movetoZ_status = 1;                                                //1����Ϊ�ƶ���λ   ��
        }
        
    }
    if((P1IFG&LS3_LA)==LS3_LA)//�ַ������λ
    {
        P1IFG &= ~LS3_LA ;
        /*
        count_BM=0;
        P1OUT |= DCME ;//��
        P1OUT |= DCMS ;//��
        TA0CCR2=0 ;//ռ�ձ�0%
        */
        TA0CCR2=0;//ͣ
        P1OUT |= LA_IN1 ;//��
        P1OUT |= LA_IN2 ;//��
        
        count_BM=0;
    }
    if((P1IFG&LS2_SERVO)==LS2_SERVO)
    {
        P1IFG &= ~LS2_SERVO ;
        
    }
}
/*******************************************************************************
PORT2�жϴ�������2.0��������λ��2.4�����������ж�
*******************************************************************************/
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    //_EINT();         //�����жϣ��ж�Ƕ��
  
    if((P2IFG&COUNT_ENCODER)==COUNT_ENCODER)//�����������ж�
    {
        P2IFG &= ~COUNT_ENCODER;//���жϱ�־λ
        if((P2IN&DIR_ENCODER)==DIR_ENCODER)//�жϷ���
        {       
            count_BM--;
        }
        if((P2IN&DIR_ENCODER)!=DIR_ENCODER)//�жϷ�����ǰ���ǵ�
        {
            count_BM++;
            if(count_BM>1200)
            {
                TA0CCR2=0 ;//ռ�ձ�0%
                P1OUT |= LA_IN1 ;//��
                P1OUT |= LA_IN2 ;//��
            }
        }
       // else{}
        
    }
    if((P2IFG&LS1_LA)==LS1_LA)//�ַ���ǰ��λ
    {
        P2IFG &= ~LS1_LA;//���жϱ�־λ
        if((P2IN&DIR_ENCODER)!=DIR_ENCODER)//�жϷ�����ǰ���ǵ�
        {
            TA0CCR2=0;//ͣ
            P1OUT |= LA_IN1 ;//��
            P1OUT |= LA_IN2 ;//��
            movetoZ_flag = 0;
            movetoZ_status = 1;                                                //1����Ϊ�ƶ���λ   ��
        }
        
    }

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
���PWM����
*******************************************************************************/
void Init_Timer0_A5(void)
{
    TA0CTL = TASSEL_2 + TACLR + MC_1 ; //ʱ��ԴΪSMCLK=8M��������ģʽ
    TA0CCR0 = 4000 - 1 ;//Ƶ��=8 000 000/8 000
    
    TA0CCTL2 = OUTMOD_6 ; // left motor pwm
    TA0CCR2=0 ;
    
    P1SEL |= BIT3 ; //���Ŷ���Ϊ�Ƚ��������PWM�źŸ��������
    P1DIR |= BIT3 ; 
    
    P1DIR |= BIT1 ; //��׷�����ƹܽ�1
    P1OUT |= BIT1 ; 
    
    P1DIR |= BIT2 ; //��׷�����ƹܽ�2
    P1OUT |= BIT2 ; 
 }
/*******************************************************************************
��̨PWM����
*******************************************************************************/
void Init_Timer1_A3(void)
{
    TA1CTL = TASSEL_2 + TACLR + MC_1 ; //ʱ��ԴΪSMCLK=8M��������ģʽ
    TA1CCR0 = 32000 - 1 ;//Ƶ��=8 000 000/8 000
    
    TA1CCTL1 = OUTMOD_6 ; // ��̨��� pwm
    TA1CCR1=servo_Y_PWM ;//��С���£��������� 12000
    TA1CCTL2 = OUTMOD_6 ; // ��̨��� pwm
    TA1CCR2=servo_X_PWM ;//��С���ң���������  7500
    
    P2SEL |= BIT2 ; //���Ŷ���Ϊ�Ƚ��������PWM�źŸ��������
    P2DIR |= BIT2 ; 
    P2SEL |= BIT3 ; //���Ŷ���Ϊ�Ƚ��������PWM�źŸ��������
    P2DIR |= BIT3 ;
 }

void SysInit()
{
    servo_X_PWM=7500;
    servo_Y_PWM=12000;
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    _DINT();//�����ж�
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
    //_EINT();         //�����ж�
}

