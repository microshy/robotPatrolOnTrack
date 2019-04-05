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
Uart1��ص����ݱ��棬����������������ֵ
*******************************************************************************/
unsigned char Uart1RXBuffer[1];
extern unsigned char distData[2]={0x00};
unsigned char Uart1RXFlag=0x00;//���ݽ��ձ�־
extern unsigned char Uart1InstructFlag=0x00;//���յ�ָ���־
extern unsigned char isObstacle=0x00;//���ϰ���
/*******************************************************************************
Uart2��ص����ݱ���,485����
*******************************************************************************/
unsigned char Uart2RXBuffer[1];
unsigned char Uart2RXFlag=0X00;//���ݽ��ձ�־
unsigned char Uart2DataBuffer[256]={0x00};//���ݱ�������
int Uart2DataBfLength=0;//���ݱ��泤��
int Uart2DataNum=0;//���ݱ������
extern unsigned char Uart2InstructFlag=0x00;//���յ�ָ���־
unsigned char Uart2InstructSource=0x00;//ָ���Դ��ַ
unsigned char Uart2InstructAim=0x00;//ָ���Ŀ�ĵ�ַ
extern unsigned char Uart2InstructNum=0x00;//ָ����

extern unsigned char Uart2UpdateID=0x00;//rom�����豸ID
extern unsigned char Uart2UpdateDataNum=0x00;//rom�����������
unsigned char UpdateDataNum;
extern unsigned char Uart2UpdateStatus=0x00;//rom����״̬λ
extern unsigned int addrEeprom=0x0000;//
extern unsigned char updateState=0x00;//��������״̬
unsigned char dataLost=0x00;

extern unsigned char moveDir=0x00;//0x00��ʾֹͣ��0x01��ʾǰ��0x02��ʾ��0x03��ʾ��
extern unsigned char Uart2moveRes[3]={0x00};//���ذ��ƶ��綯�Ƹ�ָ��
extern unsigned char Uart2movetoRes[2]={0x00};//���ذ�綯�Ƹ��ƶ���ָ��
extern unsigned char Uart2ptzRollRes[3]={0x00};//���ذ��ƶ���ָ̨��
extern unsigned char Uart2ptzRollToRes[4]={0x00};//���ذ���̨�ƶ���ָ��
//****************************************************************************
//                                                                           *
//                       ���ڳ�ʼ��                                          *
//                                                                           *
//****************************************************************************
//---------------------------����1 ������������-9600--------------------------//
void Init_UART1(void)
{
    UCA1CTL1  = UCSWRST                                        ; // ״̬����λ
    P5SEL |= TXD_US + RXD_US                                ;  //���Ź���ѡ��
    P5DIR |= TXD_US                                           ;  //���÷������
    P5DIR &= ~ RXD_US                                        ;  //���÷�������
    
    UCA1CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA1BR0   = 0x41                                           ; // 8MHz/9600=833 
    UCA1BR1   = 0x03                                           ; 
    UCA1MCTL  = UCBRS_2 + UCBRF_0                              ; // UCBRSx=2, UCBRFx=0
        
    UCA1CTL1  &= ~UCSWRST                                       ; // ����״̬��
    UCA1IE    |= UCRXIE                                         ; // ���������ж�

}
//----------------------------����2 485����--115200----- ---------------------//
void Init_UART2(void)
{
    UCA2CTL1  = UCSWRST                                        ; // ״̬����λ
    P9SEL |= TXD485BUS  + RXD485BUS                               ;  //���Ź���ѡ��
    P9DIR |= TXD485BUS                                           ;  //���÷������
    P9DIR &= ~ RXD485BUS                                         ;  //���÷�������
	
    UCA2CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA2BR0   = 0x45                                           ; // 8MHz/115200=69
    UCA2BR1   = 0x00                                           ; 
    UCA2MCTL  = UCBRS_4 + UCBRF_0                              ; // UCBRSx=4, UCBRFx=0
    
    UCA2CTL1  &= ~UCSWRST                                       ; // ����״̬��
    UCA2IE    |= UCRXIE                                         ; // ���������ж�
		
    UART2_IN;
}

/*******************************************************************************
UART1ģ������ݷ���
*******************************************************************************/
void Uart1_send(unsigned char *tx_buf,unsigned char length)
{
  unsigned char i;
  for(i=0;i<length;i++)
  {
    UCA1TXBUF = *tx_buf++ ; 
    while (!(UCA1IFG&UCTXIFG)) ; 
  }
}
/*******************************************************************************
UART2ģ������ݷ��� 485
*******************************************************************************/
void Uart2_send(unsigned char *tx_buf,unsigned char length)
{
  unsigned char i ;
  UART2_OUT;
  for(i=0;i<length;i++)
  {
    UCA2TXBUF = *tx_buf++ ; 
    while (!(UCA2IFG&UCTXIFG)) ; 
  }
  __delay_cycles(800) ;
  UART2_IN ;
}

/*******************************************************************************
UART1ģ������ݽ��� ��������ഫ����
*******************************************************************************/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV,4))
    {
        case 0:break                                                     ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart1RXBuffer[0] = UCA1RXBUF ;
            switch(Uart1RXFlag)
            {
            case 0x00:
              if(Uart1RXBuffer[0] == 0xFF)//����ͷ
              {
                  Uart1RXFlag = 0x01;//�ҵ�����ͷ
              }
              break;
            case 0x01:
              distData[0] = Uart1RXBuffer[0];//�������ݸ�λ
              Uart1RXFlag = 0x02;
              break;
            case 0x02:
              distData[1] = Uart1RXBuffer[0];//�������ݵ�λ
              Uart1RXFlag = 0x03;
              break;
            case 0x03:
              if(((distData[0]+distData[1])&0xFF) == Uart1RXBuffer[0])//У��λ
              {
                Uart1InstructFlag=0x01;
                /*
                unsigned int dist=distData[0]*256+distData[1];
                if((dist>250)&&(dist<500))
                {
                  isObstacle=0x01;//���ϰ���
                }
                */
              }
              Uart1RXFlag = 0x00;
              break;
            default :
              Uart1RXFlag = 0x00;
              break;
            }
            break ;
        case 4:break                                                     ;  // Vector 4 - TXIFG
        default: break                                                   ;  
    }  
}

/*******************************************************************************
����2���ݽ���ģ�� 485����
*******************************************************************************/
#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
    Uart2RXBuffer[0]=0x00;
    switch(__even_in_range(UCA2IV,4))
    {
        case 0:break ; // Vector 0 - no interrupt
        case 2:   // Vector 2 - RXIFG
            Uart2RXBuffer[0] = UCA2RXBUF ;
            switch (Uart2RXFlag)
            {
            case 0x00://����ͷ
              if(Uart2RXBuffer[0]==0xFE)
              {
                  Uart2RXFlag=0x01;
              }
              break;
            case 0x01://���ݳ���λ
              Uart2DataBfLength=Uart2RXBuffer[0];//��¼���ݳ���
              Uart2DataNum=0;//���ڼ���
              Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//�������ݳ���
              Uart2RXFlag=0x02;//�ҵ����ݳ���λ
              break;
            case 0x02://Ŀ�ĵ�ַ
              Uart2DataNum++;//���ڼ���
              Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//����Ŀ�ĵ�ַ
              Uart2InstructAim=Uart2RXBuffer[0];
              Uart2RXFlag=0x03;//�ҵ����ݳ���λ
              break;
            case 0x03://Դ��ַ
              Uart2DataNum++;//���ڼ���
              Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//����Դ��ַ
              Uart2InstructSource=Uart2RXBuffer[0];
              Uart2RXFlag=0x04;//�ҵ����ݳ���λ
              break;
            case 0x04://ָ����
              Uart2DataNum++;//���ڼ���
              Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//����ָ����
              Uart2InstructNum=Uart2RXBuffer[0];
              Uart2RXFlag=0x05;//�ҵ����ݳ���λ
              break;
            case 0x05://����λ��CRCλ
              Uart2DataNum++;//���ڼ���
              Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//����λ
              if(Uart2DataBfLength<=Uart2DataNum)//����Ԥ������������
              {
                unsigned char crc=Uart2RXBuffer[0];//����CRCλ
                if((Uart2InstructAim==0x03) &&(crc == CRC(Uart2DataBuffer,Uart2DataBfLength)))//Ŀ�ĵ�ַ��ȷ��У����ȷ
                {
                  Uart2InstructFlag=0x01;
                  Uart2InstructNum=Uart2ExtrInstr(Uart2DataBuffer ,Uart2DataBfLength);
                  
                  if(Uart2InstructNum==0x18)//rom����
                  {
                      if(Uart2UpdateDataNum==0x00)//��һ֡
                      {
                          updateState=0x01;
                          dataLost=0x00;//
                          UpdateDataNum=Uart2UpdateDataNum;
                          addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart2UpdateDataNum*249),Uart2DataBuffer+6 ,(unsigned char)(Uart2DataBfLength-6));
                      }
                      if(Uart2UpdateDataNum!=0x00)//���ǵ�һ֡
                      {
                          if(UpdateDataNum+1==Uart2UpdateDataNum)//����֡
                          {
                              UpdateDataNum=Uart2UpdateDataNum;
                              addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart2UpdateDataNum*249),Uart2DataBuffer+6 ,(unsigned char)(Uart2DataBfLength-6));
                          }
                          else if(UpdateDataNum==Uart2UpdateDataNum)//�ظ�֡
                          {}
                          else
                          {
                            dataLost=0x01;//���ݶ�ʧ
                          }
                      }  
                      __delay_cycles(8000);
                      unsigned char buffer_1[5]={0x05 ,0x01 ,0x03, 0x18, Uart2UpdateDataNum};
                      unsigned char crc=CRC(buffer_1,5);
                      unsigned char buffer[7]={0xFE , 0x05 ,0x01,0x03, 0x18 , Uart2UpdateDataNum , crc};
                      Uart2_send(buffer,7);
                      
                      
                      if(Uart2UpdateStatus==0x01)//���һ֡
                      {
                          if(dataLost==0x00)
                          {
                              delay_ms(5);
                              unsigned char c=0x55;
                              eeprom_writepage(0xFFFF,&c,1);//д��־�����������������ֳɵĳ��򣬿���ֱ�Ӹ���
                              
                              UCA1IE &= ~UCRXIE ; // �رս����ж�
                              UCA2IE &= ~UCRXIE ; // �رս����ж�
                              TB0CCTL0 &= ~CCIE;
                              P1IE = 0x00;
                              P2IE = 0x00;
                              WDTCTL = 0;
                              //((void (*)())0xFFFE)();
                          }
                          else
                            updateState=0x00;//��������
                      }
                  }
                }
                else
                {
                  Uart2InstructFlag=0x00;
                }
                Uart2RXFlag=0x00;
              }
              break;
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
8λCRC
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
���ӿ��ư崫����ָ�������ȡ
*******************************************************************************/

unsigned char Uart2ExtrInstr(unsigned char *buffer ,int length)
{
    switch (buffer[3])//ָ����
    {
        case 0x01://״̬��ѯָ��
            moveDir=buffer[4];
            return 0x01;
            
        case 0x02://�綯�Ƹ��˶�ָ��
            for(int i=0;i<3;i++)
                Uart2moveRes[i]=buffer[4+i];
            return 0x02;
                    
        case 0x03://�綯�Ƹ�ֹͣ�˶�ָ��
          
            return 0x03;

        case 0x04://�綯�Ƹ��ƶ���ָ��λ��ָ��
            for(int i=0;i<3;i++)
                Uart2movetoRes[i]=buffer[4+i];
            return 0x04;

        case 0x05://�綯�Ƹ��ƶ���λ��ָ��������ָ��
            
            return 0x05;

        case 0x06://�ƶ���ָ̨��
            for(int i=0;i<3;i++)
                Uart2ptzRollRes[i]=buffer[4+i];
            
            return 0x06;

        case 0x07://ֹͣ�ƶ���ָ̨��
            return 0x07;

        case 0x08://�ƶ���̨��λָ��
            for(int i=0;i<4;i++)
                Uart2ptzRollToRes[i]=buffer[4+i];
            return 0x08;
            
        case 0x09://������Ϣ����ָ��
            return 0x09;

        case 0x18:
          //Uart3UpdateID=buffer[2];//�豸ID
          Uart2UpdateDataNum=buffer[4];//�������
          Uart2UpdateStatus=buffer[5];//״̬λ
          
            return 0x18;
        default:
            return 0x00;//��ʾ��ȡ����ʧ��
    }
}




