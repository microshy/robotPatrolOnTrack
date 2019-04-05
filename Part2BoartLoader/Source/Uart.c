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
Uart3��ص����ݱ��棬��λ��ָ��ƶ����ƶ���ָ����Ҫ�·������Ϣ
*******************************************************************************/
unsigned char Uart3RXBuffer[1];
unsigned char Uart3RXFlag=0X00;//���ݽ��ձ�־
unsigned char Uart3DataBuffer[256]={0x00};//���ݱ�������
int Uart3DataBfLength=0;//���ݱ��泤��
int Uart3DataNum=0;//���ݱ������
extern unsigned char Uart3InstructFlag=0x00;//���յ�ָ���־
unsigned char Uart3InstructSource=0x00;//ָ���Դ��ַ
unsigned char Uart3InstructAim=0x00;//ָ���Ŀ�ĵ�ַ
extern unsigned char Uart3InstructNum=0x00;//ָ����

extern unsigned char Uart3UpdateID=0x00;//rom�����豸ID
extern unsigned char Uart3UpdateDataNum=0x00;//rom�����������
unsigned char UpdateDataNum;
extern unsigned char Uart3UpdateStatus=0x00;//rom����״̬λ
extern unsigned int addrEeprom=0x0000;//
extern unsigned char updateStatus;//��������״̬
unsigned char dataLost=0x00;
//****************************************************************************
//                                                                           *
//                       ���ڳ�ʼ��                                          *
//                                                                           *
//****************************************************************************
//---------------------------����1 IPORT��-115200------------------------------//
void Init_UART3(void)
{
    UCA3CTL1  = UCSWRST                                        ; // ״̬����λ
    P10SEL |= BIT4 + BIT5                                ;  //���Ź���ѡ��
    P10DIR |= BIT4                                           ;  //���÷������
    P10DIR &= ~ BIT5                                        ;  //���÷�������
    
    UCA3CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA3BR0   = 0x45                                           ; // 8MHz/115200=59 
    UCA3BR1   = 0x00                                           ; 
    UCA3MCTL  = UCBRS_4 + UCBRF_0                              ; // UCBRSx=4, UCBRFx=0
        
    UCA3CTL1  &= ~UCSWRST                                       ; // ����״̬��
    UCA3IE    |= UCRXIE                                         ; // ���������ж�
    
    UART3_IN;
}
/*******************************************************************************
UART1ģ������ݷ���
*******************************************************************************/
void Uart3_send(unsigned char *tx_buf,unsigned char length)
{
    UART3_OUT;
    unsigned char i ;
    for(i=0;i<length;i++)
    {
      UCA3TXBUF = *tx_buf++ ; 
      while (!(UCA3IFG&UCTXIFG)) ; 
    }
    __delay_cycles(8000) ;
    UART3_IN;
}
/*******************************************************************************
����3���ݽ���ģ�� 485����
*******************************************************************************/
#pragma location="SCIINTSEGMENT"				// SEGMENT at 0xFA00
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
                if((Uart3InstructAim==0x02) &&(crc == CRC(Uart3DataBuffer,Uart3DataBfLength)))//Ŀ�ĵ�ַ��ȷ��У����ȷ
                {
                  Uart3InstructFlag=0x01;
                  Uart3InstructNum=Uart3ExtrInstr(Uart3DataBuffer ,Uart3DataBfLength);
                  
                  if(Uart3InstructNum==0x18)//rom����
                  {
                      updateStatus=0x01;
                      if(Uart3UpdateDataNum==0x00)//��һ֡
                      {
                          dataLost=0x00;//
                          UpdateDataNum=Uart3UpdateDataNum;
                          addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart3UpdateDataNum*249),Uart3DataBuffer+6 ,(unsigned char)(Uart3DataBfLength-6));
                      }
                      if(Uart3UpdateDataNum!=0x00)//���ǵ�һ֡
                      {
                          if(UpdateDataNum+1==Uart3UpdateDataNum)//����֡
                          {
                              UpdateDataNum=Uart3UpdateDataNum;
                              addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart3UpdateDataNum*249),Uart3DataBuffer+6 ,(unsigned char)(Uart3DataBfLength-6));
                          }
                          else if(UpdateDataNum==Uart3UpdateDataNum)//�ظ�֡
                          {}
                          else
                          {
                            dataLost=0x01;//���ݶ�ʧ
                          }
                      }  
                      if(Uart3UpdateStatus==0x01)//���һ֡
                      {
                          if(dataLost==0x00)
                              updateStatus=0x02;
                            else
                              updateStatus=0x04;//��������
                      }
                      __delay_cycles(8000);
                      unsigned char buffer_1[5]={0x05 ,0x01 ,0x02, 0x18, Uart3UpdateDataNum};
                      unsigned char crc=CRC(buffer_1,5);
                      unsigned char buffer[7]={0xFE , 0x05 ,0x01,0x02, 0x18 , Uart3UpdateDataNum , crc};
                      Uart3_send(buffer,7);
                  }
                  else//����ִ��������
                  {
                      //updateStatus=0x04;
                  } 
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


/*******************************************************************************
����λ��������ָ�������ȡ
*******************************************************************************/
unsigned char Uart3ExtrInstr(unsigned char *buffer ,int length)
{
    switch (buffer[3])
    {
    case 0x18:
      //Uart3UpdateID=buffer[2];//�豸ID
      Uart3UpdateDataNum=buffer[4];//�������
      Uart3UpdateStatus=buffer[5];//״̬λ
      return 0x18;
      break;
                
    default:
      return 0x00;//��ʾ��ȡ����ʧ��
    }
}