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

extern unsigned char updateReady;
extern unsigned char updateState;
void main( void )
{
    SysInit();//系统初始化
    
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
        //软件更新完成
        unsigned char buffer[6]={0xFE , 0x03 , 0x19 , 0x01 , 0x56};//
        Uart1_send(buffer,5);
    }
    else
    {
    }
    delay_ms(20);
    
    _EINT();         //开总中断
    delay_ms(1000);
    
    while(1)
    {
        InstructProcess();
        if(updateState==0x00)
        {
            DetectState();
            MovetoXYZ();
        }
        if(updateReady==0x01)//软件更新
        {
            updateReady=0x00;
            updateState=0x00;
            UpdateProgram();
        }
        
        
    }
}
