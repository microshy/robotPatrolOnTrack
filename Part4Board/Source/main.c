//===========================================================================//
//                                                                           //
// 文件：  main.c                                                            //
// 说明：  主函数                                                      //
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

extern unsigned long Ltotalmile;
extern unsigned char updateState;
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
        unsigned char buffer[6]={0xFE , 0x04 , 0x01 ,0x04 , 0x19, 0x00};
        Uart1_send(buffer,6);
    }
    else
    {
    }
    delay_ms(1000);
    
    unsigned char Ctotalmile[4]={0x00};
    eeprom_readpage(0xFC00 , Ctotalmile, 4); 
    Ltotalmile=(long)(Ctotalmile[0]*256*256*256)+(long)(Ctotalmile[1]*256*256)+(long)(Ctotalmile[2]*256)+(long)(Ctotalmile[3]);
    _EINT();         //开总中断
    
    //Reset();
    //delay_ms(1000);
    while(1)
    {
        InstructProcess();
        if(updateState==0x00)
        {
            DetectSensor();
        }
        MoveToX();
        MoveToY();
    }
}
