/*******************************************************************************
Զ�̸�����������
����ʱ�ӳ�ʼ����һ�����ڽ��ջ��ƣ�EEPROM��д����
*******************************************************************************/
#include "msp430f5438a.h"
#include "SysInit.h"
#include "Uart.h"
#include "eeprom.h"
#include "FlashDrv.h"

/*** Variable Declarm ***/

unsigned int updateDone=0;//
extern unsigned char updateStatus=0x00;//�������״̬

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
        for(unsigned int i=0;i<512;i++)//��eeprom
        {
            unsigned char buffer[128]={0x00};            
            eeprom_write_page( 0x0000+i*128 , buffer , 128);//дeeprom
        }
      
        for(unsigned int i=0;i<256;i++)
        {
            unsigned char buffer[250]={0x00};
            eeprom_readpage( 0x0000+i*250 , buffer , 250);//��eeprom
            buffer[0]=0;
        }
        */
        //�ж�eeprom����û�������Ĵ����µĳ���
        unsigned char userUpdtPrgm=0x00;
        while(1)
        {
            unsigned char a2 = eeprom_readbyte( 0xFFFF , &userUpdtPrgm );//��һ���ֽ�
            if(a2==0)//��ʧ��
                break;
        }
        if(userUpdtPrgm==0x55)//�������Ĵ����µ�user�ĳ���
        {
            updateStatus=0x02;//���õȴ����ܣ�ֱ�Ӹ���
            unsigned char c=0x00;
            eeprom_writepage(0xFFFF,&c,1);//�����־
            //Uart1_send(&c,1);
        }
        //д��bootloader�ж�������/
        FLASH_EraseSEG((unsigned int *)0xFE00);		// Flash Interrupt Vector�������е��ж�������
        
	FLASH_Writew((unsigned int *)0xFFFE,0xF000);// ��λ����
        
	FLASH_Writew((unsigned int *)0xFFD8,0xFA00);// ����3�����ж�
        delay_ms(20);
        _EINT();         //�����ж�
        
        
        //�ȴ�����
        while( Waiting<200 )
        {
            P1OUT &= ~BIT6;						// LED blink
            delay_ms(100);
            P1OUT |= BIT6;						// LED blink
            delay_ms(100);
            if (updateStatus==0x00)//û�����Ҫ���£��ȴ�״̬
            {
              Waiting++;
              continue;
            }
            if (updateStatus==0x01)//���ڽ�����λ�����͵ĳ���
            {}
            if (updateStatus==0x02)//���������ɣ��ѱ��浽EEPROM��׼������Flash
            {
              //_DINT();//�����ж�
              UCA3IE &= ~UCRXIE;
              FLASH_EraseBOK();
              delay_ms(20);
              //_EINT();         //�����ж�
              continue;
            }
            if (updateStatus==0x03)//FALSH������ɣ�׼��д�����
            {
                FlashUpdate();
                updateDone=1;
            }
            if (updateStatus==0x04)//���ٽ����û�����
              Waiting=5000;
        }
        //�ȴ����߸�����ɣ�����user���򣬿�����Ҫ����д�ж�������
        //_DINT();//�����ж�
        UCA3IE &= ~UCRXIE;
        if(updateDone==0)//û�н���������£���Ҫ����д��user���ж�������
        {
            unsigned char userIntVec=0x00;
            while(1)
            {
                unsigned char a2 = eeprom_readbyte( 0xFFFE , &userIntVec );//��һ���ֽ�
                if(a2==0)//��ʧ��
                    break;
            }
            if(userIntVec==0x55)//��������user���ж�������
            {
                
                FLASH_EraseSEG((unsigned int *)0xFE00);		// Flash Interrupt Vector�������е��ж�������
                
                for(unsigned char i=0;i<4;i++)
                {
                    unsigned char buffer[128]={0x00};
                    eeprom_readpage( 0xFA00+i*128 , buffer , 128);//��eeprom
                    for(unsigned int j=0;j<128;j+=2)
                    {
                        unsigned int b=0x0000;
                        b = buffer[j+1];
                        b = b<<8 ;
                        b += buffer[j];
                        if(i*128+j+0xFE00 != 0xFFFE)//��λ�жϲ���
                        {
                          FLASH_Writew((unsigned int *)(0xFE00+i*128+j),b);//дflash
                        }
                    }
                }
                FLASH_Writew((unsigned int *)0xFFFE,0xF000);//��λ�ж�
            }
        }
        else 
        {
            //unsigned char buffer[6]={0xFE , 0x04 , 0x01 ,0x02 , 0x19, 0x5A};
            //Uart3_send(buffer,6);
            unsigned char c=0x00;
            eeprom_writepage(0xFFFE,&c,1);//��Ҫ����user����ˢ��eeprom����ж�������
        }
        delay_ms(20);
        //_EINT();         //�����ж�
        
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
