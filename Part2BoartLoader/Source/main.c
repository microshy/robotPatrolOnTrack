/*******************************************************************************
远程更新引导程序。
包括时钟初始化，一个串口接收机制，EEPROM擦写程序
*******************************************************************************/
#include "msp430f5438a.h"
#include "SysInit.h"
#include "Uart.h"
#include "eeprom.h"
#include "FlashDrv.h"

/*** Variable Declarm ***/

unsigned int updateDone=0;//
extern unsigned char updateStatus=0x00;//软件更新状态

void USERS_PROGRAM(void);

/*******************************************************************************
// Main Function
//
//
//
*******************************************************************************/
void main(void)
{
	SysInit();     
        unsigned int Waiting=0;
        updateStatus=0x00;
        updateDone=0;
         /*
        for(unsigned int i=0;i<512;i++)//清eeprom
        {
            unsigned char buffer[128]={0x00};            
            eeprom_write_page( 0x0000+i*128 , buffer , 128);//写eeprom
        }
      
        for(unsigned int i=0;i<256;i++)
        {
            unsigned char buffer[250]={0x00};
            eeprom_readpage( 0x0000+i*250 , buffer , 250);//读eeprom
            buffer[0]=0;
        }
        */
        //判断eeprom里有没有完整的待更新的程序
        unsigned char userUpdtPrgm=0x00;
        while(1)
        {
            unsigned char a2 = eeprom_readbyte( 0xFFFF , &userUpdtPrgm );//读一个字节
            if(a2==0)//读失败
                break;
        }
        if(userUpdtPrgm==0x55)//有完整的待更新的user的程序
        {
            updateStatus=0x02;//不用等待接受，直接更新
            unsigned char c=0x00;
            eeprom_writepage(0xFFFF,&c,1);//清楚标志
            //Uart1_send(&c,1);
        }
        //写回bootloader中断向量表/
        FLASH_EraseSEG((unsigned int *)0xFE00);		// Flash Interrupt Vector擦除现有的中断向量表
        
	FLASH_Writew((unsigned int *)0xFFFE,0xF000);// 复位向量
        
	FLASH_Writew((unsigned int *)0xFFD8,0xFA00);// 串口3接收中断
        delay_ms(20);
        _EINT();         //开总中断
        
        
        //等待接收
        while( Waiting<200 )
        {
            P1OUT &= ~BIT6;						// LED blink
            delay_ms(100);
            P1OUT |= BIT6;						// LED blink
            delay_ms(100);
            if (updateStatus==0x00)//没有软件要更新，等待状态
            {
              Waiting++;
              continue;
            }
            if (updateStatus==0x01)//正在接受上位机发送的程序
            {}
            if (updateStatus==0x02)//程序接受完成，已保存到EEPROM，准备擦除Flash
            {
              //_DINT();//关总中断
              UCA3IE &= ~UCRXIE;
              FLASH_EraseBOK();
              delay_ms(20);
              //_EINT();         //开总中断
              continue;
            }
            if (updateStatus==0x03)//FALSH擦除完成，准备写入程序
            {
                FlashUpdate();
                updateDone=1;
            }
            if (updateStatus==0x04)//快速进入用户程序
              Waiting=5000;
        }
        //等待或者更新完成，进入user程序，可能需要从新写中断向量表
        //_DINT();//关总中断
        UCA3IE &= ~UCRXIE;
        if(updateDone==0)//没有进行软件更新，需要再重写回user的中断向量表
        {
            unsigned char userIntVec=0x00;
            while(1)
            {
                unsigned char a2 = eeprom_readbyte( 0xFFFE , &userIntVec );//读一个字节
                if(a2==0)//读失败
                    break;
            }
            if(userIntVec==0x55)//有完整的user的中断向量表
            {
                
                FLASH_EraseSEG((unsigned int *)0xFE00);		// Flash Interrupt Vector擦除现有的中断向量表
                
                for(unsigned char i=0;i<4;i++)
                {
                    unsigned char buffer[128]={0x00};
                    eeprom_readpage( 0xFA00+i*128 , buffer , 128);//读eeprom
                    for(unsigned int j=0;j<128;j+=2)
                    {
                        unsigned int b=0x0000;
                        b = buffer[j+1];
                        b = b<<8 ;
                        b += buffer[j];
                        if(i*128+j+0xFE00 != 0xFFFE)//复位中断不变
                        {
                          FLASH_Writew((unsigned int *)(0xFE00+i*128+j),b);//写flash
                        }
                    }
                }
                FLASH_Writew((unsigned int *)0xFFFE,0xF000);//复位中断
            }
        }
        else 
        {
            //unsigned char buffer[6]={0xFE , 0x04 , 0x01 ,0x02 , 0x19, 0x5A};
            //Uart3_send(buffer,6);
            unsigned char c=0x00;
            eeprom_writepage(0xFFFE,&c,1);//需要告诉user程序刷新eeprom里的中断向量表
        }
        delay_ms(20);
        //_EINT();         //开总中断
        
	/*** Timeout or Quit, Deinit SCI, and Run User's Program @ 0x5C00 ***/
	//UCA3IE  &= ~UCRXIE;
	USERS_PROGRAM();						// Run User's Program @ 0x5C00
}
/*******************************************************************************
// User's Function @ 5C00
*******************************************************************************/
#pragma location="USERSEGMENT"
void USERS_PROGRAM(void)
{
    while(1)
    {
            P1OUT &= ~BIT6 ;	//
            delay_ms(500);
            P1OUT |= BIT6 ;	//
            delay_ms(500);
    }
}
