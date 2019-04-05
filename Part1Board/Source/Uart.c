//===========================================================================//
//                                                                           //
// �ļ���  Uart.c                                                            //
// ˵����  ����ģ��                                                          //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
//                                                                           //
//===========================================================================//

#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "Uart.h"
#include "eeprom.h"
#include "Control.h"

unsigned char CRC8Table[] =
{ 
    //0
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E, 
    //1
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    //2
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    //3
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    //4 
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    //5
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    //6
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    //7
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    //8
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    //9
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    //A
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    //B
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    //C
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    //D
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xFE, 0x79, 0x48, 0x1B, 0x2A,
    //E
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    //F
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC

};
/*******************************************************************************
Uart0��ص����ݱ��棬����������������ֵ
*******************************************************************************/
unsigned char Uart0RXBuffer[1];
extern unsigned char distData[2]={0x00};
unsigned char Uart0RXFlag=0x00;//���ݽ��ձ�־
extern unsigned char Uart0InstructFlag=0x00;//���յ�ָ���־
extern unsigned char isObstacle=0x00;//���ϰ���
/*******************************************************************************
Uart1��ص����ݱ��棬��λ��ָ��ƶ����ƶ���ָ����Ҫ�·������Ϣ
*******************************************************************************/
unsigned char Uart1RXBuffer[1];
unsigned char Uart1RXFlag=0X00;//���ݽ��ձ�־
extern unsigned char Uart1DataBuffer[256]={0x00};//���ݱ�������
extern unsigned char Uart1DataBfLength=0;//���ݱ��泤��
unsigned char Uart1DataNum=0;//���ݱ������
extern unsigned char Uart1InstructFlag=0x00;//���յ�ָ���־
extern unsigned char Uart1InstructNum=0x00;//ָ����

extern unsigned char Uart1moveRes[4]={0x00};//��λ���ƶ�ָ��
extern unsigned char Uart1movetoRes[8]={0x00};//��λ���ƶ���ָ��
extern unsigned char Uart1ptzRollRes[3]={0x00};//��λ��������̨�ƶ�ָ��
extern unsigned char Uart1ctrlInfraPTZRes[4]={0x00};//��λ��������̨�ƶ���ָ��
extern unsigned char Uart1UpdateID=0x00;//rom�����豸ID
extern unsigned char Uart1UpdateDataNum=0x00;//rom�����������
extern unsigned char UpdateDataNum=0x00;
extern unsigned char Uart1UpdateStatus=0x00;//rom����״̬λ
extern unsigned int addrEeprom=0x0000;//
extern unsigned int Uart1UpdateDataCount=0;
extern unsigned char updateState=0x00;
extern unsigned char updateReady=0x00;
unsigned char dataLost=0x00;


/*******************************************************************************
Uart2��ص����ݱ��棬�ַŴ����� ���ַ�ֵ
*******************************************************************************/
unsigned char Uart2RXBuffer[1];
unsigned char Uart2RXFlag=0X00;//���ݽ��ձ�־
extern unsigned char Uart2DataBuffer[20]={0x00};//���ݱ�������
extern unsigned char Uart2DataBfLength=0;//���ݱ��泤��
unsigned char Uart2DataNum=0;//���ݱ������
extern unsigned char Uart2InstructFlag=0x00;//���յ�ָ���־
extern unsigned char TEV[2];//TEV����ֵ
extern unsigned char US[2];//�ַų�����

/*******************************************************************************
Uart3��ص����ݱ���,485����
*******************************************************************************/
unsigned char Uart3RXBuffer[1];
unsigned char Uart3RXFlag=0X00;//���ݽ��ձ�־
unsigned char Uart3DataBuffer[256];//���ݱ�������
unsigned char Uart3DataBfLength=0;//���ݱ��泤��
unsigned char Uart3DataNum=0;//���ݱ������
extern unsigned char Uart3InstructFlag=0x00;//���յ�ָ���־
extern unsigned char Uart3InstructSource=0x00;//ָ���Դ��ַ
unsigned char Uart3InstructAim;//ָ���Ŀ�ĵ�ַ
unsigned char Uart3InstructNum;//ָ����


//extern unsigned char ChrgState;//���״̬
extern unsigned char totalMile[32];//�������Ϣ
extern long totalMileBMQ;
extern long LtotalMile;
extern unsigned char Electricity;//����
extern unsigned char current[2];//����
extern unsigned char positionX[4];//X������
extern unsigned char positionY[2];//Y������
extern unsigned char positionZ[2];//Z������
extern unsigned char CyuntaiX[2];//��̨���λ��
extern unsigned char CyuntaiY[2];//��̨���λ��
extern unsigned char O2[2];//����Ũ��
extern unsigned char SF6[2];//SF6����Ũ��
extern unsigned char temperature[2];//�¶�
extern unsigned char humidity[2];//ʪ��
extern unsigned char volume[2];//����ֵ

extern unsigned char motor_status_X;//С�����״̬
extern unsigned char motor_status_Y;//�������״̬
extern unsigned char motor_status_Z;//�ַŵ��״̬
extern unsigned char encoder_status_X;//С�����ӱ�����
extern unsigned char encoder_status_Y;//�����˰��ӱ�����
extern unsigned char encoder_status_Z;//�ַŰ��ӱ�����
extern unsigned char xianwei_status_X;//С����λ״̬
extern unsigned char xianwei_status_Y;//������λ״̬
extern unsigned char xianwei_status_Z;//�ַ���λ״̬
extern unsigned char TEV_status;//tev����
extern unsigned char US_status;//����������
extern unsigned char SF6_status;//SF6����������
extern unsigned char O2_status;//O2����������
extern unsigned char TaH_status;//��ʪ�ȴ���������
extern unsigned char Volume_status;//��������������
extern unsigned char Inf_status;//�����������
extern unsigned char yunatai_status;//��̨����
extern unsigned char camare_status;//����ͷ����
extern unsigned char charge_status;//������

extern unsigned char movetoX_status;//X���ƶ�����
extern unsigned char movetoY_status;//y���ƶ�����
extern unsigned char movetoZ_status;//z���ƶ�����

extern unsigned char isObstacleUart3=0x00;//�ַſ��ư���ϰ�����Ϣ
//****************************************************************************
//                                                                           *
//                       ���ڳ�ʼ��                                          *
//                                                                           *
//****************************************************************************
//---------------------------����0���ӳ�����������---------------------------//
void Init_UART0(void)
{
    UCA0CTL1  = UCSWRST ; // ״̬����λ
    P3SEL |= TXD_US + RXD_US ;  //���Ź���ѡ��
    P3DIR |= TXD_US ;  //���÷������
    P3DIR &= ~ RXD_US ;  //���÷�������
	
    UCA0CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA0BR0   = 0x41                                           ; // 8MHz/9600=833 
    UCA0BR1   = 0x03                                           ; 
    UCA0MCTL  = UCBRS_2 + UCBRF_0                              ; // UCBRSx=2, UCBRFx=0
    
    UCA0CTL1  &= ~UCSWRST                                       ; // ����״̬��
    UCA0IE    |= UCRXIE                                         ; // ���������ж�
}
//---------------------------����1 IPORT��-115200------------------------------//
void Init_UART1(void)
{
    UCA1CTL1  = UCSWRST                                        ; // ״̬����λ
    P5SEL |= TXD_IPORT + RXD_IPORT                                ;  //���Ź���ѡ��
    P5DIR |= TXD_IPORT                                           ;  //���÷������
    P5DIR &= ~ RXD_IPORT                                        ;  //���÷�������
    
    UCA1CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA1BR0   = 0x45                                           ; // 8MHz/115200=59 
    UCA1BR1   = 0x00                                           ; 
    UCA1MCTL  = UCBRS_4 + UCBRF_0                              ; // UCBRSx=4, UCBRFx=0
    /*
    UCA1BR0   = 0x8A                                           ; // 8MHz/57600=138 
    UCA1BR1   = 0x00                                           ; 
    UCA1MCTL  = UCBRS_7 + UCBRF_0                              ; // UCBRSx=7, UCBRFx=0
    */
    UCA1CTL1  &= ~UCSWRST                                       ; // ����״̬��
    UCA1IE    |= UCRXIE                                         ; // ���������ж�

}
//----------------------------����2 �ַŴ�����--9600--------------------------//
void Init_UART2(void)
{
    UCA2CTL1  = UCSWRST                                        ; // ״̬����λ
    P9SEL |= TXD_JF  + RXD_JF                               ;  //���Ź���ѡ��
    P9DIR |= TXD_JF                                           ;  //���÷������
    P9DIR &= ~ RXD_JF                                         ;  //���÷�������
	
    UCA2CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA2BR0   = 0x41                                           ; // 8MHz/9600=833 
    UCA2BR1   = 0x03                                           ; 
    UCA2MCTL  = UCBRS_2 + UCBRF_0                              ; // UCBRSx=2, UCBRFx=0
    
    UCA2CTL1  &= ~UCSWRST                                       ; // ����״̬��
    UCA2IE    |= UCRXIE                                         ; // ���������ж�
		
    UART2_IN;
}
//-----------------------------����3 485����--115200----------  --------------//
void Init_UART3(void)
{
    UCA3CTL1  = UCSWRST                                        ; // ״̬����λ
    P10SEL |= TXD485BUS  + RXD485BUS                               ;  //���Ź���ѡ��
    P10DIR |= TXD485BUS                                           ;  //���÷������
    P10DIR &= ~ RXD485BUS                                         ;  //���÷�������

    UCA3CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA3BR0   = 0x45                                           ; // 8MHz/115200=69 
    UCA3BR1   = 0x00                                           ; 
    UCA3MCTL  = UCBRS_4 + UCBRF_0                              ; // UCBRSx=4, UCBRFx=0
    
    UCA3CTL1  &= ~UCSWRST                                       ; // ����״̬��
    UCA3IE    |= UCRXIE                                         ; // ���������ж�
    
    UART3_IN;
}
/*******************************************************************************
UART0ģ������ݷ���
*******************************************************************************/
void Uart0_send(unsigned char *tx_buf,unsigned int length)
{
  unsigned int i ;
  for(i=0;i<length;i++)
  {
    UCA0TXBUF = *tx_buf++ ; 
    while (!(UCA0IFG&UCTXIFG)) ; 
  }
}
/*******************************************************************************
UART1ģ������ݷ���
*******************************************************************************/
void Uart1_send(unsigned char *tx_buf,unsigned int length)
{
  unsigned int i;
  for(i=0;i<length;i++)
  {
    UCA1TXBUF = *tx_buf++ ; 
    while (!(UCA1IFG&UCTXIFG)) ; 
  }
}
/*******************************************************************************
UART2ģ������ݷ��ͣ��ַŴ����� 485
*******************************************************************************/
void Uart2_send(unsigned char *tx_buf,unsigned int length)
{
  unsigned int i ;
  UART2_OUT;
  for(i=0;i<length;i++)
  {
    UCA2TXBUF = *tx_buf++ ; 
    while (!(UCA2IFG&UCTXIFG)) ; 
  }
  __delay_cycles(8000) ;
  UART2_IN ;
}
/*******************************************************************************
UART3ģ������ݷ��� 485����
*******************************************************************************/
void Uart3_send(unsigned char *tx_buf,unsigned int length)
{
  UART3_OUT;
  unsigned int i ;
  for(i=0;i<length;i++)
  {
    UCA3TXBUF = *tx_buf++ ; 
    while (!(UCA3IFG&UCTXIFG)) ; 
  }
  __delay_cycles(800) ;
  UART3_IN;
}

/*******************************************************************************
UART0ģ������ݽ��� ������������
*******************************************************************************/
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV,4))
    {
        case 0:break                                                     ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart0RXBuffer[0] = UCA0RXBUF ;
            switch(Uart0RXFlag)
            {
            case 0x00:
              if(Uart0RXBuffer[0] == 0xFF)//����ͷ
              {
                  Uart0RXFlag = 0x01;//�ҵ�����ͷ
              }
              break;
            case 0x01:
              distData[0] = Uart0RXBuffer[0];//�������ݸ�λ
              Uart0RXFlag = 0x02;
              break;
            case 0x02:
              distData[1] = Uart0RXBuffer[0];//�������ݵ�λ
              Uart0RXFlag = 0x03;
              break;
            case 0x03:
              if(((distData[0]+distData[1])&0xFF) == Uart0RXBuffer[0])//У��λ
              {
                Uart0InstructFlag=0x01;
                /*
                unsigned int dist=distData[0]*256+distData[1];
                if((dist>250)&&(dist<500))
                {
                  isObstacle=0x01;//���ϰ���
                }
                else 
                {
                  isObstacle=0x00;//���ϰ���
                }
                */
              }
              Uart0RXFlag = 0x00;
              break;
            default :
              Uart0RXFlag = 0x00;
              break;
            }
            break ;
        case 4:break                                                     ;  // Vector 4 - TXIFG
        default: break                                                   ;  
    }  
}
/*******************************************************************************
UART1ģ������ݽ��� IPORT�� ������λ������
*******************************************************************************/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV,4))
    {
        case 0:break                                                     ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart1RXBuffer[0] = UCA1RXBUF ;
            //Uart1_send(Uart1RXBuffer,1);
            switch (Uart1RXFlag)
            {
            case 0x00:
              if(Uart1RXBuffer[0]==0xFE)
              {
                Uart1RXFlag=0x01;//�ҵ�����ͷ
                //Uart1_send(Uart1RXBuffer,1);
              }
              break;
            case 0x01://���ݳ���
              
              Uart1DataBfLength=Uart1RXBuffer[0];//��¼���ݳ���
              Uart1DataNum=0;//���ڼ���
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//�������ݳ���
              Uart1RXFlag=0x02;//�ҵ����ݳ���λ
              //Uart1_send(Uart1RXBuffer,1);
              break;
            case 0x02://ָ����
              Uart1DataNum++;//���ڼ���
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//����ָ����
              //Uart1InstructNum=Uart1RXBuffer[0];//����ָ����
              Uart1RXFlag=0x03;//
              //Uart1_send(Uart1RXBuffer,1);
              break;
            case 0x03://����λ��CRCУ��λ
              
              Uart1DataNum++;//���ڼ���
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//����λ��CRCУ��λ
              //Uart1_send(Uart1RXBuffer,1);
              if(Uart1DataBfLength<=Uart1DataNum)//����Ԥ������������
              {
                unsigned char crc=CRC(Uart1DataBuffer,Uart1DataBfLength);//����CRCλ
                //Uart1_send(&crc,1);
                //Uart1_send(Uart1DataBuffer,Uart1DataBfLength+1);
                if(crc == Uart1RXBuffer[0])
                {
                  Uart1InstructFlag=0x01;
                  Uart1InstructNum=Uart1ExtrInstr(Uart1DataBuffer ,Uart1DataBfLength);
                  
                  if(Uart1InstructNum==0x18)//��������ָ��
                  {
                      if(Uart1UpdateDataNum==0x00)//��һ֡
                      {
                          updateState=0x01;//��������״̬
                          UpdateDataNum=Uart1UpdateDataNum;
                          addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart1UpdateDataNum*250),Uart1DataBuffer+5 ,(unsigned char)(Uart1DataBfLength-5));
                          dataLost=0x00;
                      }
                      if(Uart1UpdateDataNum!=0x00)//���ǵ�һ֡
                      {
                          if(UpdateDataNum+1==Uart1UpdateDataNum)//����֡
                          {
                            UpdateDataNum=Uart1UpdateDataNum;
                            addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart1UpdateDataNum*250),Uart1DataBuffer+5 ,(unsigned char)(Uart1DataBfLength-5));
                          }
                          else if(UpdateDataNum==Uart1UpdateDataNum)//�ظ�֡
                          {
                          }
                          else
                            dataLost=0x01;//��������ݶ�ʧ
                          
                      }  
                      __delay_cycles(8000);
                      unsigned char buffer_1[3]={0x03 , 0x18, Uart1UpdateDataNum};
                      unsigned char crc=CRC(buffer_1,3);
                      unsigned char buffer[5]={0xFE , 0x03 , 0x18 , Uart1UpdateDataNum , crc};
                      Uart1_send(buffer,5);
                      
                      if(Uart1UpdateStatus==0x01)//���һ֡
                      {
                          if(dataLost==0x00)//û�ж���������ӽ�������
                          {
                              if(Uart1UpdateID==0x01)//���ذ壬���жϽ���Bootloader����
                              {
                                  delay_ms(5);
                                  unsigned char c=0x55;
                                  eeprom_writepage(0xFFFF,&c,1);//д��־�����������������ֳɵĳ��򣬿���ֱ�Ӹ���
                                  
                                  UCA0IE &= ~UCRXIE ; // �رս����ж�
                                  UCA1IE &= ~UCRXIE ; // �رս����ж�
                                  UCA2IE &= ~UCRXIE ; // �رս����ж�
                                  UCA3IE &= ~UCRXIE ; // �رս����ж�
                                  TA0CCTL0 &= ~CCIE ; // �ر�ʱ���ж�
                                  TB0CCTL0 &= ~CCIE ; //
                                  WDTCTL = 0;
                              }
                              else//�ӿذ�
                              {
                                  //����һ��Ҫ���µ�������С
                                  Uart1UpdateDataCount = Uart1UpdateDataNum*250+(unsigned int)(Uart1DataBfLength-5);
                                  updateReady=0x01;
                                  //UpdateProgram();
                              }
                          }
                          else 
                            updateState=0x00;//���ݰ���ʧ�������к������£�����ִ����������
                      }
                  }
                }
                else
                {
                  Uart1InstructFlag=0x00;
                }
                Uart1RXFlag=0x00;
              }
              break;
            default :
              Uart1RXFlag=0x00;
              break;
            }
            
            break ;
        case 4:break                                                     ;  // Vector 4 - TXIFG
        default: break                                                   ;  
    }  
}

/*******************************************************************************
����2���ݽ���ģ�� �ַŴ�����
*******************************************************************************/
#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
    switch(__even_in_range(UCA2IV,4))
    {
        case 0:break ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart2RXBuffer[0] = UCA2RXBUF ;
            switch(Uart2RXFlag)
            {
            case 0x00:
            {
                if(Uart2RXBuffer[0]==0x01)//�豸��ַ��01
                {
                    Uart2RXFlag=0x01;
                    Uart2DataNum=0;
                    Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//�����豸��ַλ
                }
                break;
            }
            case 0x01:
            {
                if(Uart2RXBuffer[0]==0x03)//������
                {
                    Uart2RXFlag=0x02;
                    Uart2DataNum++;
                    Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//���湦����λ
                }
                else 
                    Uart2RXFlag=0x00;
                break;
            }
            case 0x02:
            {
                Uart2DataBfLength=Uart2RXBuffer[0];//��¼���ݳ���
                Uart2DataNum++;
                Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//�������ݳ���
                Uart2RXFlag=0x03;//�ҵ����ݳ���λ
                break;
            }
            case 0x03:
            {
                Uart2DataNum++;//����
                Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//�������ݳ���
                if(Uart2DataNum-2==Uart2DataBfLength)
                  Uart2RXFlag=0x04;//����λ�������
                break;
            }
            case 0x04:
            {
                Uart2DataNum++;
                Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//����CRC16У���λ
                Uart2RXFlag=0x05;
                break;
            }
            case 0x05:
            {
                Uart2DataNum++;
                Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//����CRC16У���λ
                Uart2RXFlag=0x00;
                
                unsigned int crc=Uart2DataBuffer[Uart2DataNum-1]*256+Uart2DataBuffer[Uart2DataNum];
                if(crc==CRC16RTU(Uart2DataBuffer,Uart2DataBfLength+3))
                {
                    Uart2InstructFlag=0x01;//���ճɹ�
                    //��ȡ������ֵ
                    TEV[0]=Uart2DataBuffer[3];
                    TEV[1]=Uart2DataBuffer[4];
                    US[0]=Uart2DataBuffer[7];
                    US[1]=Uart2DataBuffer[8];
                }
                else
                {
                    Uart2InstructFlag=0x00;//����ʧ��
                }
                break;
            }
            default :
              Uart2RXFlag=0x00;
              break;
            }
            break ;
        case 4:break ;  // Vector 4 - TXIFG
        default: break ;  
    }
}
/*******************************************************************************
����3���ݽ���ģ�� 485����
*******************************************************************************/
#pragma vector=USCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void)
{
    
    Uart3RXBuffer[0]=0x00;
    switch(__even_in_range(UCA3IV,4))
    {
        case 0:break ; // Vector 0 - no interrupt
        case 2:   // Vector 2 - RXIFG
            Uart3RXBuffer[0] = UCA3RXBUF ;
            switch (Uart3RXFlag)
            {
            case 0x00://����ͷ
              if(Uart3RXBuffer[0]==0xFE)
              {
                  Uart3RXFlag=0x01;
              }
              break;
            case 0x01://���ݳ���λ
              Uart3DataBfLength=Uart3RXBuffer[0];//��¼���ݳ���
              Uart3DataNum=0;//���ڼ���
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//�������ݳ���
              Uart3RXFlag=0x02;//�ҵ����ݳ���λ
              break;
            case 0x02://Ŀ�ĵ�ַ
              Uart3DataNum++;//���ڼ���
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//����Ŀ�ĵ�ַ
              Uart3InstructAim=Uart3RXBuffer[0];
              Uart3RXFlag=0x03;//�ҵ����ݳ���λ
              break;
            case 0x03://Դ��ַ
              Uart3DataNum++;//���ڼ���
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//����Դ��ַ
              Uart3InstructSource=Uart3RXBuffer[0];
              Uart3RXFlag=0x04;//�ҵ����ݳ���λ
              break;
            case 0x04://ָ����
              Uart3DataNum++;//���ڼ���
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//����ָ����
              Uart3InstructNum=Uart3RXBuffer[0];
              Uart3RXFlag=0x05;//�ҵ����ݳ���λ
              break;
            case 0x05://����λ��CRCλ
              Uart3DataNum++;//���ڼ���
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//����λ
              if(Uart3DataBfLength<=Uart3DataNum)//����Ԥ������������
              {
                unsigned char crc=Uart3RXBuffer[0];//����CRCλ
                if(crc == CRC(Uart3DataBuffer,Uart3DataBfLength))
                {
                  Uart3InstructFlag=0x01;
                  Uart3InstructNum=Uart3ExtrInstr(Uart3DataBuffer ,Uart3DataBfLength);//��ȡָ������������
                }
                else
                {
                  Uart3InstructFlag=0x00;
                }
                Uart3RXFlag=0x00;
              }
              break;
            default :
              Uart3RXFlag=0x00;
              break;
              
            }
            break ;
        case 4:break ;  // Vector 4 - TXIFG
        default: break ;  
    }  
}

/*******************************************************************************
����CRCУ��λ

*******************************************************************************/
unsigned char CRC(unsigned char *buffer, int length)
{
    unsigned char crc = 0x00;
    for (int i = 0; i < length; i++)
    {
        crc = CRC8Table[crc ^ buffer[i]];
    }
    return crc;
}
/******************************************************
*��������:CRC16RTU
*��   ��:pszBuf  ҪУ�������
        unLength У�����ݵĳ�
*��   ��:У��ֵ
*��   ��:ѭ������У��-16
         ��RTU��׼-0xA001��
*******************************************************/
unsigned int CRC16RTU( unsigned char * pszBuf, unsigned int unLength)
{
    unsigned int CRC=0XFFFF;
    unsigned long CRC_count;
    for(CRC_count=0;CRC_count<unLength;CRC_count++)
    {
        int i;
        CRC=CRC^*(pszBuf+CRC_count);

        for(i=0;i<8;i++)
        {
            if(CRC&1)
            {
                CRC>>=1;
                CRC^=0xA001;
            }
            else
            { 
                CRC>>=1;
            }
        }
    }
    return CRC;
}

/*******************************************************************************
����λ��������ָ�������ȡ
*******************************************************************************/
unsigned char Uart1ExtrInstr(unsigned char *buffer ,int length)
{
    switch (buffer[1])
    {

        case 0x04://��λ����ָ��
            return 0x04;

        case 0x05://�����˰汾��Ϣ����ָ��
            return 0x05;

        case 0x06://�����˳��״̬����ָ��
            return 0x06;

        case 0x07://�������˶����������ָ��
            return 0x07;

        case 0x08://������״̬����ָ��
            return 0x08;
            
        case 0x09://������Ϣ����ָ��
            return 0x09;
            
        case 0x0A://�ƶ�����������ָ������˶��ᡢ�˶������˶��ٶ���Ϣ
            for (int i=0;i<4;i++)
            {
                Uart1moveRes[i] = buffer[i+2];
            }            
            return 0x0A;
            
        case 0x0B://ֹͣ�ƶ�����������ָ��
            return 0x0B;
            
        case 0x0C://����ֹͣ�ƶ�����������ָ��
            return 0x0C;
            
        case 0x0D://��ȡ������Ϣָ��
            return 0x0D;
            
        case 0x0E://�ƶ������˵�ָ����������ָ�������ά����
            for(int i=0;i<8;i++)
            {
                Uart1movetoRes[i] = buffer[i+2];
            }
            
            return 0x0E;

        case 0x0f://������̨ת������ָ����淽����ٶ�
            for (int i=0;i<3;i++)
            {
                Uart1ptzRollRes[i] = buffer[2+i];
            }
            return 0x0f;
                    
        case 0X10://ֹͣ������̨ת������ָ��
            return 0x10;

        case 0x11://��̨��������ָ������ά����
            for (int i=0;i<4;i++)
            {
                Uart1ctrlInfraPTZRes[i] = buffer[2+i];
            }
            return 0x11;

        case 0x12://��ȡ������̨λ������ָ��
            return 0x12;

        case 0x13://�ַţ�TEV���������ָ��
            return 0x13;
            
        case 0x14://�ַţ����������������ָ��
            return 0x14;
            
        case 0x15://O2Ũ�ȼ������ָ��
            return 0x15;
            
        case 0x16://SF6Ũ�ȼ������ָ��
            return 0x16;
            
        case 0x17://���������������ָ��
            return 0x17;
        case 0x18://Update����ָ��
            Uart1UpdateID=buffer[2];//�豸ID
            Uart1UpdateDataNum=buffer[3];//�������
            Uart1UpdateStatus=buffer[4];//״̬λ
            
            return 0x18;
        default:
            return 0x00;//��ʾ��ȡ����ʧ��
    }
}

/*******************************************************************************
���ӿ��ư崫����ָ�������ȡ
*******************************************************************************/
unsigned char Uart3ExtrInstr(unsigned char *buffer ,int length)
{
    switch (buffer[3])//ָ����
    {
        case 0x01://״̬��ѯָ��
            if(buffer[2]==0x02)//���ݲɼ���ظ�
            {
                O2[0]=buffer[4];
                O2[1]=buffer[5];
                SF6[0]=buffer[6];
                SF6[1]=buffer[7];
                volume[0]=buffer[8];
                volume[1]=buffer[9];
                O2_status=buffer[10];
                SF6_status=buffer[11];
                Volume_status=buffer[12];
            }
            else if(buffer[2]==0x03)//�ַ��˶����ư����ݻظ�
            {
                positionZ[0]=buffer[4];//�綯�Ƹ�λ��
                positionZ[1]=buffer[5];
                motor_status_Z=buffer[6];//Z���״̬
                encoder_status_Z=buffer[7];//Z�������״̬
                movetoZ_status=buffer[8];//
                CyuntaiX[0]=buffer[9];//��̨λ��
                CyuntaiX[1]=buffer[10];
                CyuntaiY[0]=buffer[11];//��̨λ��
                CyuntaiY[1]=buffer[12];
                yunatai_status=buffer[13];//��̨״̬
                isObstacleUart3=buffer[14];//�ϰ�����Ϣ
            }
            else if(buffer[2]==0x04)//С�����������˶����ư����ݻظ�
            {
                positionX[0]=buffer[4];//С��λ��
                positionX[1]=buffer[5];
                positionX[2]=buffer[6];
                positionX[3]=buffer[7];
                motor_status_X=buffer[8];//X���״̬
                encoder_status_X=buffer[9];//С��������״̬
                movetoX_status=buffer[10];
                
                positionY[0]=buffer[11];//������λ��
                positionY[1]=buffer[12];
                motor_status_Y=buffer[13];//Y���״̬
                encoder_status_Y=buffer[14];//�����˱�����״̬
                movetoY_status=buffer[15];
                
                humidity[0]=buffer[16];//ʪ��
                humidity[1]=buffer[17];//ʪ��
                temperature[0]=buffer[18];//�¶�
                temperature[1]=buffer[19];//�¶�
                
                TaH_status=buffer[20];//��ʪ�ȴ�����״̬
                
                //�����
                for(int i=0;i<10;i++)
                {
                    totalMile[i]=buffer[21+i];
                }
            }
            else if(buffer[2]==0x05)//��Դ���������ݻظ�
            {
                charge_status=buffer[4];
                Electricity=buffer[5];
                current[0]=buffer[6];
                current[1]=buffer[7];
            }
            else{}
              
            return 0x01;
            
        case 0x02://
            return 0x02;
                    
        case 0x03://
            return 0x03;

        case 0x04://
            return 0x04;

        case 0x05://
            return 0x05;

        case 0x06://
            return 0x06;

        case 0x07://
            return 0x07;

        case 0x08://
            return 0x08;
            
        case 0x09://
            return 0x09;

        default:
            return 0x00;//
    }
}




