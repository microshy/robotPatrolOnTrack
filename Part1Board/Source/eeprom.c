//===========================================================================//
//                                                                           //
// �ļ���  eeprom.c                                                          //
// ˵����  AT24C256��д����                                                  //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.5                        //
// �汾��  v1.1 
// ʱ�䣺  2017/03/03                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUSR                                                             //
//                                                                           //
//===========================================================================//
/******************************************************************************
��ע��
�������������ҵģ�ֱ���õ�ʱ���ֶ���ʱ������0XFF��
��������AT24C256�Ķ�д��ַ�������⣺AT24C256�Ķ�д��ַ�������ֽڡ�
���ֽ���ǰ�����ֽ��ں�
The 256K is internally organized as 512 pages of 64-bytes each. 
Random word addressing requires a 15-bit data word address.
��д��ַ��15λ�ģ���0x0004��0x8004�޲��
512ҳ��ÿҳ64�ֽڣ��������0x003f������д�����ֽڣ����λ����ʵ��0x003f,0x0000;
��AT24C256�����ҳ������д��
���ǿ��Կ�ҳ����
����д֮����Ҫһ��ʱ����

�ο�AT24C256�ļ����ֲ�
******************************************************************************/


#include "msp430f5438a.h"
#include "eeprom.h"


//ʵ�����
/*
    unsigned char test_byte1=0xcc;
    unsigned char test_byte2=0x01;
    unsigned int test_addr=0x0004;
    unsigned char a1 = eeprom_writebyte( test_addr , test_byte1 );//д��һ���ֽ�
    if(a1==0)
    {
        a1=0;
    }
    delay_ms(10);
    unsigned char a2 = eeprom_readbyte( test_addr , &test_byte2 );//��һ���ֽ�
    if(a2==0)
    {
        a2=0;
    }
    
    if( test_byte1 == test_byte2 )
    {
      int a=1;
    }
    */
    /*
    unsigned char test_buf1[8]= {1,2,3,4,5,6,7,8};  
    unsigned char test_buf2[8]= {0,0,0,0,0,0,0,0};  
   //while(1)
   {
    eeprom_writepage(0x0008 , test_buf1, 8);  
    delay_ms(10);  
   }
    eeprom_readpage(0x0008 , test_buf2, 8);  
    
    if(test_buf1[0]==test_buf1[0])
    {
      int a=1;
    }
*/
    /*
    unsigned char test_buf1[128];  
    for(unsigned char i=0;i<128;i++)
    {
        test_buf1[i]=i;
    }
    unsigned char test_buf2[128];  
    for(unsigned char i=0;i<128;i++)
    {
        test_buf2[i]=i+128;
    }
    unsigned int addr=0x0004;
    addr=eeprom_write_page(addr , test_buf1, 128);  
    addr=eeprom_write_page(addr , test_buf2, 128);  
    
    unsigned char buffer[256]={0x00};
    //for(unsigned int i=0;i<4;i++)
    {
        eeprom_readpage(0x0000,buffer,255);
    }
    
    if(test_buf1[0]==test_buf1[0])
    {
      int a=1;
    }
*/

/*******************************************************************************
��ʼ��EEPROM
*******************************************************************************/
void Init_EEPROM()
{
    P10SEL &= ~BIT2;                         // P10.2@UCB0SCL
    P10DIR |= BIT2;
    P10OUT |= BIT2;
    // ���9��ʱ���Իָ�I2C����״̬
    
    for( int i = 0 ; i < 9 ; i++ )
    {
      P10OUT |= BIT2;
      __delay_cycles(8000);
      P10OUT &= ~BIT2;
      __delay_cycles(8000);
    }

    P10SEL |= (BIT1 + BIT2);                 // P3.1@UCB0SDAP3.2@UCB0SCL
    // P3.1@ISP.1 P3.2@ISP.5

    UCB3CTL1 |= UCSWRST;
    UCB3CTL0 = UCMST + UCMODE_3 + UCSYNC ;  // I2C����ģʽ
    UCB3CTL1 |= UCSSEL_2;                   // ѡ��SMCLK
    UCB3BR0 = 80;
    UCB3BR1 = 0;
    UCB3CTL0 &= ~UCSLA10;                   // 7λ��ַģʽ
    UCB3I2CSA = 0x50;            // EEPROM��ַ
    UCB3CTL1 &= ~UCSWRST;
    
}
/*******************************************************************************
д�����ֽ�
*******************************************************************************/
unsigned char eeprom_writebyte( unsigned int Iword_addr , unsigned char word_value )
{
  //_DINT();//�����ж�
  while( UCB3CTL1 & UCTXSTP );
  UCB3CTL1 |= UCTR;                 // дģʽ
  UCB3CTL1 |= UCTXSTT;              // ��������λ

  unsigned char Cword_addr;
  Cword_addr=Iword_addr>>8;
  UCB3TXBUF = Cword_addr;            // �����ֽڵ�ַ
  // �ȴ�UCTXIFG=1 ��UCTXSTT=0 ͬʱ�仯�ȴ�һ����־λ����
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // ����Ӧ�� UCNACKIFG=1
    {
      return 1;
    }
  }  
  Cword_addr=Iword_addr;
  UCB3TXBUF = Cword_addr;            // �����ֽڵ�ַ
  // �ȴ�UCTXIFG=1 ��UCTXSTT=0 ͬʱ�仯�ȴ�һ����־λ����
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // ����Ӧ�� UCNACKIFG=1
    {
      return 1;
    }
  }  

  UCB3TXBUF = word_value;           // �����ֽ�����
  while(!(UCB3IFG & UCTXIFG));      // �ȴ�UCTXIFG=1

  UCB3CTL1 |= UCTXSTP;
  while(UCB3CTL1 & UCTXSTP);        // �ȴ��������

  //_EINT();         //�����ж�
  return 0;
}

/*******************************************************************************
д����ֽڣ����ܿ�ҳд
*******************************************************************************/
unsigned char eeprom_writepage( unsigned int Iword_addr , unsigned char *pword_buf , unsigned char len)
{
  //_DINT();//�����ж�
  while( UCB3CTL1 & UCTXSTP );
  UCB3CTL1 |= UCTR;                 // дģʽ
  UCB3CTL1 |= UCTXSTT;              // ��������λ

  unsigned char Cword_addr;
  Cword_addr=Iword_addr>>8;
  UCB3TXBUF = Cword_addr;            // �����ֽڵ�ַ
  // �ȴ�UCTXIFG=1 ��UCTXSTT=0 ͬʱ�仯�ȴ�һ����־λ����
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // ����Ӧ�� UCNACKIFG=1
    {
      return 1;
    }
  }  
  Cword_addr=Iword_addr;
  UCB3TXBUF = Cword_addr;            // �����ֽڵ�ַ
  // �ȴ�UCTXIFG=1 ��UCTXSTT=0 ͬʱ�仯�ȴ�һ����־λ����
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // ����Ӧ�� UCNACKIFG=1
    {
      return 1;
    }
  }  

  for( unsigned char i = 0 ; i < len ; i++ )
  {
    UCB3TXBUF = *pword_buf++;       // ���ͼĴ�������
    while(!(UCB3IFG & UCTXIFG));    // �ȴ�UCTXIFG=1   
  }

  UCB3CTL1 |= UCTXSTP;
  while(UCB3CTL1 & UCTXSTP);        // �ȴ��������

  //_EINT();         //�����ж�
  return 0;
}
/*******************************************************************************
д����ֽڣ����Կ�ҳд����������ַ
*******************************************************************************/
unsigned int eeprom_write_page( unsigned int Iword_addr , unsigned char *pword_buf , unsigned char len)
{
    if(len<=64-Iword_addr%64)//һҳ��д��
    {
          while(1)
          {
              unsigned char a=eeprom_writepage(Iword_addr,pword_buf,len);
              if(a==0)
                break;
          }
    }
    else//һҳ��д����
    {   
        unsigned int i;
        while(1)
        {
            unsigned char a=eeprom_writepage(Iword_addr,pword_buf,64-Iword_addr%64);
            __delay_cycles(8000);
            if(a==0)
              break;
        }
        for(i=0;i<(len-(64-Iword_addr%64))/64;i++)
        {
            while(1)
            {
                unsigned char a = eeprom_writepage(Iword_addr+64-Iword_addr%64+i*64,pword_buf+64-Iword_addr%64+i*64,64);
                __delay_cycles(8000);
                if(a==0)
                  break;
            }
        }
        while(1)
        {
            unsigned char a = eeprom_writepage(Iword_addr+(64-Iword_addr%64)+i*64,pword_buf+64-Iword_addr%64+i*64,len-(64-Iword_addr%64)-i*64);
            __delay_cycles(8000);
            if(a==0)
              break;
        }
    }
    return (Iword_addr+len);
}
/*******************************************************************************
�������ֽ�
*******************************************************************************/
unsigned char eeprom_readbyte( unsigned int Iword_addr , unsigned char *pword_value )
{
  //_DINT();//�����ж�
  UCB3CTL1 |= UCTR;                 // дģʽ
  UCB3CTL1 |= UCTXSTT;              // ��������λ��д�����ֽ�

  unsigned char Cword_addr;
  Cword_addr=Iword_addr>>8;
  UCB3TXBUF = Cword_addr;            // �����ֽڵ�ַ������Ҫ�����TXBUF
  // �ȴ�UCTXIFG=1 ��UCTXSTT=0 ͬʱ�仯�ȴ�һ����־λ����
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // ����Ӧ�� UCNACKIFG=1
    {
      return 1;
    }
  }                       
  Cword_addr=Iword_addr;
  UCB3TXBUF = Cword_addr;            // �����ֽڵ�ַ������Ҫ�����TXBUF
  // �ȴ�UCTXIFG=1 ��UCTXSTT=0 ͬʱ�仯�ȴ�һ����־λ����
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // ����Ӧ�� UCNACKIFG=1
    {
      return 1;
    }
  }                       

  UCB3CTL1 &= ~UCTR;                // ��ģʽ
  UCB3CTL1 |= UCTXSTT;              // ��������λ�Ͷ������ֽ�

  while(UCB3CTL1 & UCTXSTT);        // �ȴ�UCTXSTT=0
  // ����Ӧ�� UCNACKIFG = 1
  UCB3CTL1 |= UCTXSTP;              // �ȷ���ֹͣλ

  while(!(UCB3IFG & UCRXIFG));      // ��ȡ�ֽ�����
  *pword_value = UCB3RXBUF;         // ��ȡBUF�Ĵ����ڷ���ֹͣλ֮��

  while( UCB3CTL1 & UCTXSTP );

  //_EINT();         //�����ж�
  return 0;
}

/*******************************************************************************
������ֽ�
*******************************************************************************/
unsigned char eeprom_readpage(unsigned int Iword_addr , unsigned char *pword_buf , unsigned char len )
{
  //_DINT();//�����ж�
  while( UCB3CTL1 & UCTXSTP );
  UCB3CTL1 |= UCTR;                 // дģʽ
  UCB3CTL1 |= UCTXSTT;              // ��������λ��д�����ֽ�

  unsigned char Cword_addr;
  Cword_addr=Iword_addr>>8;
  UCB3TXBUF = Cword_addr;            // �����ֽڵ�ַ
  // �ȴ�UCTXIFG=1 ��UCTXSTT=0 ͬʱ�仯�ȴ�һ����־λ����
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // ����Ӧ�� UCNACKIFG=1
    {
      return 1;
    }
  }  
  Cword_addr=Iword_addr;
  UCB3TXBUF = Cword_addr;            // �����ֽڵ�ַ
  // �ȴ�UCTXIFG=1 ��UCTXSTT=0 ͬʱ�仯�ȴ�һ����־λ����
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // ����Ӧ�� UCNACKIFG=1
    {
      return 1;
    }
  }  

  UCB3CTL1 &= ~UCTR;                // ��ģʽ
  UCB3CTL1 |= UCTXSTT;              // ��������λ�Ͷ������ֽ�

  while(UCB3CTL1 & UCTXSTT);        // �ȴ�UCTXSTT=0
  // ����Ӧ�� UCNACKIFG = 1
  for( unsigned char i = 0; i < len - 1 ; i++ )
  {
    while(!(UCB3IFG & UCRXIFG));    // ��ȡ�ֽ����ݣ����������һ���ֽ�����
    *pword_buf++ = UCB3RXBUF;
  }

  UCB3CTL1 |= UCTXSTP;              // �ڽ������һ���ֽ�֮ǰ����ֹͣλ

  while(!(UCB3IFG & UCRXIFG));      // ��ȡ���һ���ֽ�����
  *pword_buf = UCB3RXBUF;

  while( UCB3CTL1 & UCTXSTP );

  //_EINT();         //�����ж�
  return 0;
}