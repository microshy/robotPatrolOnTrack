//===========================================================================//
//                                                                           //
// 文件：  main.c                                                            //
// 说明：  主函数                                                            //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.5                        //
// 版本：  v1.3 
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUSR                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "SysInit.h"
#include "Uart.h"
#include "Control.h"
#include "eeprom.h"
#include "FlashDrv.h"

extern unsigned char updateState;//软件更新状态

void main( void )
{
    SysInit();
    
    unsigned char userIntVec=0x00;
    while(1)
    {
        unsigned char a2 = eeprom_readbyte( 0xFFFE , &userIntVec );//读一个字节
        if(a2==0)//读失败
            break;
    }
    if(userIntVec!=0x55)//没有，初次运行，eeprom里没有中断向量表
    {
        for(unsigned char i=0;i<4;i++)
        {
            unsigned char buffer[128]={0x00};
            FLASH_ReadPage(0xFE00+i*128,buffer,128);//读flash
            eeprom_write_page( 0xFA00+i*128 , buffer , 128);//写eeprom
        }
        eeprom_writebyte(0xFFFE,0x55);//标志有完整的user的中断向量表
        unsigned char buffer[6]={0xFE , 0x04 , 0x01 ,0x03 , 0x19, 0xAE};
        Uart2_send(buffer,6);
    }
    else
    {
    }
    delay_ms(1000);
    
    _EINT();         //开总中断
    
    while((P1IN&LS3_LA)!=LS3_LA)//缩回来，利用限位开关限制，防止伸缩杆在限位0点时仍能向里缩
    {
        P1OUT |= LA_IN2 ;//低
        P1OUT &= ~LA_IN1 ;//高
        
        TA0CCR2=1000;
    }
    TA0CCR2=0;
    P1OUT |= LA_IN2 ;//低
    P1OUT |= LA_IN1 ;//高
    
    while(1)
    {/*
        P3OUT &= ~LED2;
        delay_ms(200);
        P3OUT |= LED2;
        delay_ms(200);
      */
        InstructProcess();//处理主控的命令
        if(updateState==0x00)
        {
            DetectSensor();//循环进行超声波避障传感器的检测
        }
        MoveToZ();
    }
}
