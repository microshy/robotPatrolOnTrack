//===========================================================================//
//                                                                           //
// 文件：  eeprom.c                                                          //
// 说明：  AT24C256读写程序                                                  //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.5                        //
// 版本：  v1.1 
// 时间：  2017/03/03                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUSR                                                             //
//                                                                           //
//===========================================================================//
/******************************************************************************
备注：
程序是在网上找的，直接用的时候发现读的时候老是0XFF，
后来发现AT24C256的读写地址出现问题：AT24C256的读写地址是两个字节。
高字节在前，低字节在后；
The 256K is internally organized as 512 pages of 64-bytes each. 
Random word addressing requires a 15-bit data word address.
读写地址是15位的，即0x0004和0x8004无差别；
512页，每页64字节；即如果在0x003f处连续写两个字节，存的位置其实是0x003f,0x0000;
即AT24C256不会跨页的连续写。
但是可以跨页读。
两次写之间需要一定时间间隔

参考AT24C256的技术手册
******************************************************************************/


#include "msp430f5438a.h"
#include "eeprom.h"


//实验程序
/*
    unsigned char test_byte1=0xcc;
    unsigned char test_byte2=0x01;
    unsigned int test_addr=0x0004;
    unsigned char a1 = eeprom_writebyte( test_addr , test_byte1 );//写入一个字节
    if(a1==0)
    {
        a1=0;
    }
    delay_ms(10);
    unsigned char a2 = eeprom_readbyte( test_addr , &test_byte2 );//读一个字节
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
初始化EEPROM
*******************************************************************************/
void Init_EEPROM()
{
    P10SEL &= ~BIT2;                         // P10.2@UCB0SCL
    P10DIR |= BIT2;
    P10OUT |= BIT2;
    // 输出9个时钟以恢复I2C总线状态
    
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
    UCB3CTL0 = UCMST + UCMODE_3 + UCSYNC ;  // I2C主机模式
    UCB3CTL1 |= UCSSEL_2;                   // 选择SMCLK
    UCB3BR0 = 80;
    UCB3BR1 = 0;
    UCB3CTL0 &= ~UCSLA10;                   // 7位地址模式
    UCB3I2CSA = 0x50;            // EEPROM地址
    UCB3CTL1 &= ~UCSWRST;
    
}
/*******************************************************************************
写单个字节
*******************************************************************************/
unsigned char eeprom_writebyte( unsigned int Iword_addr , unsigned char word_value )
{
  //_DINT();//关总中断
  while( UCB3CTL1 & UCTXSTP );
  UCB3CTL1 |= UCTR;                 // 写模式
  UCB3CTL1 |= UCTXSTT;              // 发送启动位

  unsigned char Cword_addr;
  Cword_addr=Iword_addr>>8;
  UCB3TXBUF = Cword_addr;            // 发送字节地址
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化等待一个标志位即可
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }  
  Cword_addr=Iword_addr;
  UCB3TXBUF = Cword_addr;            // 发送字节地址
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化等待一个标志位即可
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }  

  UCB3TXBUF = word_value;           // 发送字节内容
  while(!(UCB3IFG & UCTXIFG));      // 等待UCTXIFG=1

  UCB3CTL1 |= UCTXSTP;
  while(UCB3CTL1 & UCTXSTP);        // 等待发送完成

  //_EINT();         //开总中断
  return 0;
}

/*******************************************************************************
写多个字节，不能跨页写
*******************************************************************************/
unsigned char eeprom_writepage( unsigned int Iword_addr , unsigned char *pword_buf , unsigned char len)
{
  //_DINT();//关总中断
  while( UCB3CTL1 & UCTXSTP );
  UCB3CTL1 |= UCTR;                 // 写模式
  UCB3CTL1 |= UCTXSTT;              // 发送启动位

  unsigned char Cword_addr;
  Cword_addr=Iword_addr>>8;
  UCB3TXBUF = Cword_addr;            // 发送字节地址
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化等待一个标志位即可
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }  
  Cword_addr=Iword_addr;
  UCB3TXBUF = Cword_addr;            // 发送字节地址
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化等待一个标志位即可
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }  

  for( unsigned char i = 0 ; i < len ; i++ )
  {
    UCB3TXBUF = *pword_buf++;       // 发送寄存器内容
    while(!(UCB3IFG & UCTXIFG));    // 等待UCTXIFG=1   
  }

  UCB3CTL1 |= UCTXSTP;
  while(UCB3CTL1 & UCTXSTP);        // 等待发送完成

  //_EINT();         //开总中断
  return 0;
}
/*******************************************************************************
写多个字节，可以跨页写，返回最后地址
*******************************************************************************/
unsigned int eeprom_write_page( unsigned int Iword_addr , unsigned char *pword_buf , unsigned char len)
{
    if(len<=64-Iword_addr%64)//一页内写完
    {
          while(1)
          {
              unsigned char a=eeprom_writepage(Iword_addr,pword_buf,len);
              if(a==0)
                break;
          }
    }
    else//一页内写不完
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
读单个字节
*******************************************************************************/
unsigned char eeprom_readbyte( unsigned int Iword_addr , unsigned char *pword_value )
{
  //_DINT();//关总中断
  UCB3CTL1 |= UCTR;                 // 写模式
  UCB3CTL1 |= UCTXSTT;              // 发送启动位和写控制字节

  unsigned char Cword_addr;
  Cword_addr=Iword_addr>>8;
  UCB3TXBUF = Cword_addr;            // 发送字节地址，必须要先填充TXBUF
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化等待一个标志位即可
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }                       
  Cword_addr=Iword_addr;
  UCB3TXBUF = Cword_addr;            // 发送字节地址，必须要先填充TXBUF
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化等待一个标志位即可
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }                       

  UCB3CTL1 &= ~UCTR;                // 读模式
  UCB3CTL1 |= UCTXSTT;              // 发送启动位和读控制字节

  while(UCB3CTL1 & UCTXSTT);        // 等待UCTXSTT=0
  // 若无应答 UCNACKIFG = 1
  UCB3CTL1 |= UCTXSTP;              // 先发送停止位

  while(!(UCB3IFG & UCRXIFG));      // 读取字节内容
  *pword_value = UCB3RXBUF;         // 读取BUF寄存器在发送停止位之后

  while( UCB3CTL1 & UCTXSTP );

  //_EINT();         //开总中断
  return 0;
}

/*******************************************************************************
读多个字节
*******************************************************************************/
unsigned char eeprom_readpage(unsigned int Iword_addr , unsigned char *pword_buf , unsigned char len )
{
  //_DINT();//关总中断
  while( UCB3CTL1 & UCTXSTP );
  UCB3CTL1 |= UCTR;                 // 写模式
  UCB3CTL1 |= UCTXSTT;              // 发送启动位和写控制字节

  unsigned char Cword_addr;
  Cword_addr=Iword_addr>>8;
  UCB3TXBUF = Cword_addr;            // 发送字节地址
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化等待一个标志位即可
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }  
  Cword_addr=Iword_addr;
  UCB3TXBUF = Cword_addr;            // 发送字节地址
  // 等待UCTXIFG=1 与UCTXSTT=0 同时变化等待一个标志位即可
  while(!(UCB3IFG & UCTXIFG))
  {
    if( UCB3IFG & UCNACKIFG )       // 若无应答 UCNACKIFG=1
    {
      return 1;
    }
  }  

  UCB3CTL1 &= ~UCTR;                // 读模式
  UCB3CTL1 |= UCTXSTT;              // 发送启动位和读控制字节

  while(UCB3CTL1 & UCTXSTT);        // 等待UCTXSTT=0
  // 若无应答 UCNACKIFG = 1
  for( unsigned char i = 0; i < len - 1 ; i++ )
  {
    while(!(UCB3IFG & UCRXIFG));    // 读取字节内容，不包括最后一个字节内容
    *pword_buf++ = UCB3RXBUF;
  }

  UCB3CTL1 |= UCTXSTP;              // 在接收最后一个字节之前发送停止位

  while(!(UCB3IFG & UCRXIFG));      // 读取最后一个字节内容
  *pword_buf = UCB3RXBUF;

  while( UCB3CTL1 & UCTXSTP );

  //_EINT();         //开总中断
  return 0;
}