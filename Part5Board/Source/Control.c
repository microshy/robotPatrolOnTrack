//===========================================================================//
//                                                                           //
// �ļ���  Control.c                                                         //
// ˵����  ���ܺ���������ͨ��Э������Ӧ����                                  //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "Control.h"
#include "SysInit.h"
#include "Uart.h"

extern unsigned char chrgstateNow=0x00;
extern unsigned char chrgstateOld=0x00;
int chrgFailCount=0;//�����ϴ���
unsigned char battery=0x00;
extern float voltage=0.0;
extern float current=0.0;
extern unsigned char Uart1InstructFlag;//���յ�ָ���־
extern unsigned char Uart1InstructNum;//ָ����
int timerCount=0;

/*******************************************************************************
��ȡ���ݲɼ������ݡ�״̬��Ϣ
*******************************************************************************/
void DetectState()
{
    if((chrgstateNow==0x01)&&(0x00==chrgstateOld))
    {
      P4OUT |= RELAY1;
    }
    else if((chrgstateNow==0x00)&&(0x01==chrgstateOld))
    {
      P4OUT &= ~RELAY1;
    }
    else
    {}
    unsigned char buffer_1[8];
    buffer_1[0]=0x08;//����λ
    buffer_1[1]=0x01;//Ŀ�ĵ�ַλ
    buffer_1[2]=0x05;//Դ��ַλ
    buffer_1[3]=0x01;//ָ����
    if(chrgstateNow==0x01)//�ڳ��״̬������£��жϳ���ǲ����й���
    {
        if((current-1.0<1.0)&&(voltage<27.5))//��ѹС��һ��ֵ������£�������С��һ��ֵ����Ϊû�г��ϵ�
        {
            chrgFailCount++;
            //
        }
        else 
            chrgFailCount=0;
            //
    }
    if(chrgFailCount>=50)
    {
      chrgFailCount=50;
      buffer_1[4]=0x01;
    }
    else 
      buffer_1[4]=0x00;
    
    
    if(voltage>28.6)
      battery=100;
    else if(voltage>28.0)
      battery=95;
    else if(voltage>27.6)
      battery=90;
    else if(voltage>27.2)
      battery=85;
    else if(voltage>26.8)
      battery=80;
    else if(voltage>26.5)
      battery=75;
    else if(voltage>26.0)
      battery=70;
    else if(voltage>25.8)
      battery=65;
    else if(voltage>25.4)
      battery=60;
    else if(voltage>25.2)
      battery=55;
    else if(voltage>24.9)
      battery=50;
    else if(voltage>24.7)
      battery=45;
    else if(voltage>24.6)
      battery=40;
    else if(voltage>24.4)
      battery=35;
    else if(voltage>24.3)
      battery=30;
    else if(voltage>24.1)
      battery=25;
    else if(voltage>24.)
      battery=20;
    else if(voltage>23.7)
      battery=15;
    else if(voltage>23.5)
      battery=10;
    else
      battery=5;
    
    buffer_1[5]=battery;
    
    int Icurrent = (int)(current*100);
    buffer_1[6]=(unsigned char)(Icurrent>>8);
    buffer_1[7]=(unsigned char)(Icurrent);
    
    //buffer_1[5]=100;
    unsigned char crc = CRC(buffer_1,8);
    
    unsigned char buffer[10];
    buffer[0]=0xfe;
    for(int i=0;i<8;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[9]=crc;
    Uart1_send(buffer,10);
  
}
/*******************************************************************************
ָ�����
*******************************************************************************/
void InstructProcess()
{
    if(Uart1InstructFlag==0x01)//������ָ��
    {
        Uart1InstructFlag=0x00;//���־
        switch (Uart1InstructNum)
        {
        case 0x01:
          DetectState();
          break;
          
        case 0x02:
          
          break;
        
     
        default :
          break;
          
          
        }
    }
}
/*******************************************************************************
����ѹ����
*******************************************************************************/
void DetectOwnState()
{
    if(Uart1InstructFlag != 0x01)
    {
        DectctCur();//�����
        timerCount++;
        if(timerCount==1)
        {
            DetectVol();//���ѹ
        }
        if(timerCount>3000)//һ����
        {
            timerCount=0;
        }
    }
}
/*******************************************************************************
��Сʱ��Ƭ
*******************************************************************************/
void MinTimeslice()
{
    Init_Timer0_B7();
    while(1)
    {
        if(timer0Flag==1)
        {
            timer0Count++;
            break;
        }
    }
}
/*******************************************************************************
����ѹʱ��Ƭ
*******************************************************************************/
void DetectVol()
{       //���ѹ
    Init_Timer0_B7();
    voltage=detectVoltage();
    while(1)
    {
        if(timer0Flag==1)
        {
            break;
        }
        
    }
}
/*******************************************************************************
������ʱ��Ƭ
*******************************************************************************/
void DectctCur()
{
    //�����
    Init_Timer0_B7();
    current=detectCurrent();
    while(1)
    {
        if(timer0Flag==1)
        {
            break;
        }
        
    }
}
/*******************************************************************************
��ѹ��⣬��ѹ���
*******************************************************************************/
float detectVoltage()
{
 
    P6SEL |= BIT5; // ѡ��P6.5��Ϊģ���źŵ������

    ADC12CTL0 = ADC12ON; // ʹ��ת��ģ��

    REFCTL0|=REFMSTR+REFVSEL_2+REFON;

    //ʹ��REF�����ڲ��ο���ѹѡ��2.5v�����ڲ��ο���ѹ

    ADC12CTL1 = ADC12SHP;// // ѡ�����崥��ģʽ����ͨ�����δ�ת��ģʽ

    ADC12MCTL0= ADC12SREF_1+ADC12INCH_5; //ѡ��ο���ѹԴ������a5ͨ��

    ADC12CTL0 |= ADC12ENC; // ʹ��ת��ģ��

    for ( unsigned char i=0; i<0x30; i++);

    ADC12CTL0 |= ADC12SC; // ��ʼת��

    while ((ADC12CTL1 & ADC12BUSY));

    float voltage= (((float)ADC12MEM0/4096)*2.5)*24.6605-9.4441; //�۲�Average
  
  /*
    // ֻ����ADC12ENC��λ������²ſ��Բ���  
    // ADC12SHT1X ADC12SHT0X ADC12MSC ADC12REF2_5V ADC12REFON ADC12ON  
    ADC12CTL0 &= ~ADC12ENC;  
  
    // ���ò�������ʱ�䣬���ʱ�����������ת������  
    // ע��MSP430F5438û��REFģ�飬Ƭ�ڻ�׼��Ч  
    // ����ADC12REF2_5V ��ADC12REFON��������  
    //ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON;  
//    ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON +  
//                ADC12REF2_5V + ADC12REFON;  
    // ���������������Բ�����ʱ��  
    REFCTL0|=REFMSTR+REFVSEL_2+REFON;
    ADC12CTL1 = ADC12SHP;  
    // �ر��ڲ��ڲ��¶ȼ���Խ��͹��ģ�ע�����������޸�ת������  
    ADC12CTL2 |= ADC12TCOFF ;  
    // ��׼��ѹѡ��AVCC����ѡ��1ͨ������(AVCC-AVSS)/2  
    ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_5;  
  
    __delay_cycles(75);  
    // ADC12ʹ��  
    ADC12CTL0 |= ADC12ENC; 
    
    
    ADC12CTL0 |= ADC12SC;                   // ����ת��  
    while ( !(ADC12IFG & BIT0) );           // �ȴ�ת�����  
  
            // ��ת����ͨ��Ϊͨ��11 (AVCC-AVSS)/2;  
            // ��ʱת���ľ���Ϊ12λ����4096  
            // AVCCͨ��һ����к�LDO�����������  
            // ��ӡLDO�����ѹ������3λ����  
    float voltage = (ADC12MEM0  / 4096.0) * 33 * 2.4/2.5; 
     */
    return voltage;
   
}

/*******************************************************************************
�������
*******************************************************************************/
float detectCurrent()
{
    P6SEL |= BIT6; // ѡ��P6.5��Ϊģ���źŵ������

    ADC12CTL0 = ADC12ON; // ʹ��ת��ģ��

    REFCTL0|=REFMSTR+REFVSEL_2+REFON;

    //ʹ��REF�����ڲ��ο���ѹѡ��2.5v�����ڲ��ο���ѹ

    ADC12CTL1 = ADC12SHP;// // ѡ�����崥��ģʽ����ͨ�����δ�ת��ģʽ

    ADC12MCTL0= ADC12SREF_1+ADC12INCH_6; //ѡ��ο���ѹԴ������a5ͨ��

    ADC12CTL0 |= ADC12ENC; // ʹ��ת��ģ��

    for ( unsigned char i=0; i<0x30; i++);

    ADC12CTL0 |= ADC12SC; // ��ʼת��

    while ((ADC12CTL1 & ADC12BUSY));
    float electric = ( 2.5-(ADC12MEM0  / 4096.0 * 2.5) )*10;
  /*
    // ֻ����ADC12ENC��λ������²ſ��Բ���  
    // ADC12SHT1X ADC12SHT0X ADC12MSC ADC12REF2_5V ADC12REFON ADC12ON  
    ADC12CTL0 &= ~ADC12ENC;  
  
    // ���ò�������ʱ�䣬���ʱ�����������ת������  
    // ע��MSP430F5438û��REFģ�飬Ƭ�ڻ�׼��Ч  
    // ����ADC12REF2_5V ��ADC12REFON��������  
    ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON;  
//    ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON +  
//                ADC12REF2_5V + ADC12REFON;  
    // ���������������Բ�����ʱ��  
    ADC12CTL1 = ADC12SHP;  
    // �ر��ڲ��ڲ��¶ȼ���Խ��͹��ģ�ע�����������޸�ת������  
    ADC12CTL2 |= ADC12TCOFF ;  
    // ��׼��ѹѡ��AVCC����ѡ��1ͨ������(AVCC-AVSS)/2  
    ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_6;  
  
    __delay_cycles(75);  
    // ADC12ʹ��  
    ADC12CTL0 |= ADC12ENC; 
    
    
    ADC12CTL0 |= ADC12SC;                   // ����ת��  
    while ( !(ADC12IFG & BIT0) );           // �ȴ�ת�����  
  
            // ��ת����ͨ��Ϊͨ��11 (AVCC-AVSS)/2;  
            // ��ʱת���ľ���Ϊ12λ����4096  
            // AVCCͨ��һ����к�LDO�����������  
            // ��ӡLDO�����ѹ������3λ����  
    float electric = ADC12MEM0  / 4096.0 * 3.3; 
  */
    return electric;
    
}



