//===========================================================================//
//                                                                           //
// �ļ���  main.c                                                            //
// ˵����  ������                                                      //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.5                        //
// �汾��  v1.3 
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUSR                                                             //
//                                                                           //
//===========================================================================//



#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "SysInit.h"
#include "Uart.h"
#include "Control.h"
#include "eeprom.h"
#include "FlashDrv.h"
extern unsigned char updateState;

void main( void )
{
    SysInit();//ϵͳ��ʼ��
  
    //unsigned char a1[22]={"\r\nStart User Program\r\n"};
    //Uart3_send(a1,22);
    unsigned char userIntVec=0x00;
    while(1)
    {
        unsigned char a2 = eeprom_readbyte( 0xFFFE , &userIntVec );//��һ���ֽ�
        if(a2==0)//��ʧ��
            break;
    }
    if(userIntVec!=0x55)//û�У��������У�eeprom��û���ж�������
    {
        for(unsigned char i=0;i<4;i++)
        {
            unsigned char buffer[128]={0x00};
            FLASH_ReadPage(0xFE00+i*128,buffer,128);//��flash
            eeprom_write_page( 0xFA00+i*128 , buffer , 128);//дeeprom
        }
        eeprom_writebyte(0xFFFE,0x55);//��־��������user���ж�������
        unsigned char buffer[6]={0xFE , 0x04 , 0x01 ,0x02 , 0x19, 0x5A};
        Uart3_send(buffer,6);
    }
    delay_ms(20);
    
    _EINT();         //�����ж�
    
    while(1)
    {
        InstructProcess();//ָ���
        if(updateState==0x00)
        {
            DetectSensor();
        }
    }
}
