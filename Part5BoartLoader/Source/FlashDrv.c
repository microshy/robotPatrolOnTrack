//===========================================================================//
//                                                                           //
// 文件：  FlashDrv.c                                                        //
// 说明：  flash驱动                                                         //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "FlashDrv.h"
#include "eeprom.h"

unsigned char RxASCII[4]={0,0,0,0};				// Program Rx Buffer
unsigned int FlxDat=0;							// Program HEX buffer
unsigned int *FlxAdr=
(unsigned int *)0x5C00;							// Program Address
unsigned long int FlxCnt=0;						// Program byte Counter
extern unsigned char updateStatus;//软件更新状态
unsigned char flashDataType=0x00;//数据类型，0x00表示地址数据，0x01表示程序数据
/*******************************************************************************
// ASCII to Hex - transform ASCII at RxASCII (char) to Hex at FlxDat (uint)
*******************************************************************************/
void ASCII2HEX(void)
{
	unsigned char i;
	FlxDat=0;
	for(i=0;i<4;i++)
	{
		if(RxASCII[i]>='A' && RxASCII[i]<='F')
			FlxDat = (FlxDat<<4) | (RxASCII[i]-55);
		if(RxASCII[i]>='0' && RxASCII[i]<='9')
			FlxDat = (FlxDat<<4) | (RxASCII[i]-48);
	}

}

/*******************************************************************************
// Do Data Exchange - LSB first in FLASH, But SCI Tramsmit by MSB first, Do it
*******************************************************************************/
void DoDataExg(void)
{
	unsigned char Ex;
	Ex=(unsigned char)FlxDat;
	FlxDat=(FlxDat>>8)+(Ex<<8);
}


/*******************************************************************************
// Erase one Segment - one SEG is 0x200 (512 Bytes)
*******************************************************************************/
void FLASH_EraseSEG(unsigned int *Address)
{
	//_DINT();
	FCTL3=FWKEY;								// LOCK = 0
	while((FCTL3&BUSY)==BUSY);					// Waitint for FLASH
	FCTL1=FWKEY+ERASE;							// ERASE=1
	*Address=0;									// Write to the SEGment
	FCTL1=FWKEY;
	FCTL3=FWKEY+LOCK;
	while((FCTL3&BUSY)==BUSY);					// Waitint for FLASH
	//_EINT();
}

/*******************************************************************************
// Write a word (nValue) to Address
*******************************************************************************/
void FLASH_Writew(unsigned int *Address,unsigned int nValue)
{
	FCTL1=FWKEY+WRT;							// WRT = 1
	FCTL3=FWKEY;								// LOCK = 0
	while((FCTL3&BUSY)==BUSY);					// Waitint for FLASH
	*Address=nValue;
	FCTL1=FWKEY;								// WRT = 0
	FCTL3=FWKEY+LOCK;							// LOCK = 1
	while((FCTL3&BUSY)==BUSY);					// Waitint for FLASH
}

/*******************************************************************************
// 描述: 读FLASH操作
// 输入: unsigned int waddr 16位地址
// 输出: unsigned char 返回一个字节数据
*******************************************************************************/
unsigned char FLASH_Read(unsigned int waddr)
{
  unsigned char value;
  while(FCTL3 & BUSY);
  value = *(unsigned char *)waddr;
  return value;
}
/*******************************************************************************
// 描述: 读FLASH操作
// 输入: unsigned int waddr 16位地址
         unsigned char *pword_buff 存放地址
         unsigned char len 读取长度
// 输出: unsigned char 返回一个字节数据
*******************************************************************************/
void FLASH_ReadPage(unsigned int waddr,unsigned char *pword_buff,unsigned char len)
{
  while(FCTL3 & BUSY);
  while(len--)
  {
    *pword_buff = *(unsigned char *)waddr;
  }
}
/*******************************************************************************
// Erase the Block - Erase from 0x5C00 to 0xF000
*******************************************************************************/
void FLASH_EraseBOK(void)
{
	unsigned int Adr;
	Adr=0x5C00;
	while(Adr<0xF000)
	{
		FLASH_EraseSEG((unsigned int *)Adr);
		Adr+=0x0200;
	}
	FLASH_EraseSEG((unsigned int *)0xFE00);		// Flash Interrupt Vector
	FlxAdr=(unsigned int *)0xFFFE;				// Reverve Reset Vector
	FlxDat=0xF000;								// Interrupt @ 0xF000
	FLASH_Writew(FlxAdr,FlxDat);
	/*
        FlxAdr=(unsigned int *)0xFFDC;				// Reverve Uart1 Vector
	FlxDat=0xFA00;								// Interrupt @ 0xFA00
	FLASH_Writew(FlxAdr,FlxDat);
        */
        updateStatus=0x03;
}


/*******************************************************************************
// Flash RxASCII to FLASH in Order
*******************************************************************************/
void Flash_Program(void)
{
	unsigned char check=0;
	ASCII2HEX();								// ASCII to HEX
	if(flashDataType==0x00)//地址数据
	{
		FlxAdr=(unsigned int *)FlxDat;			// Load Address on FlxDat
		flashDataType=0x01;//程序数据
	}
	else
	{
		check |= (FlxAdr>=(unsigned int *)0x5C00) && (FlxAdr<(unsigned int *)0xF000);
		check |= (FlxAdr>=(unsigned int *)0xFD00) && (FlxAdr<(unsigned int *)0xFFFE);
		check &= /*(FlxAdr!=(unsigned int *)0xFFDC) && */(FlxAdr!=(unsigned int *)0xFFFE);//串口中断和复位向量
		if(check)
		{
			DoDataExg();						// Exchange data
			FLASH_Writew(FlxAdr,FlxDat);		// Flash word
			FlxAdr++;							// Address increase
		}
	}
	// Sendback
	//SCI_Tx(FlxDat/256);
	//SCI_Tx(FlxDat);
}

/*******************************************************************************
// Flash Master in Programing
*******************************************************************************/
void FlashMaster(unsigned char temp)
{
	if(temp==' ')
		return;
	else if(temp=='@')
		flashDataType=0x00;//地址数据
	else if(temp=='q')
	{
		if(FlxCnt%4>0)
		{
			Flash_Program();					// Flash the last words
		}
                updateStatus=0x04;//更新完成
	}
	else if((temp>='A' && temp<='F') | (temp>='0' && temp<='9'))
	{
		RxASCII[FlxCnt%4]=temp;
		FlxCnt++;
		if(FlxCnt !=0 && FlxCnt%4==0)
		{
			Flash_Program();
		}
	}
        else return;
	// Ignored Space
	// Ignored \r 0x0D
	// Ignored \n 0x0A
}
void FlashUpdate()
{
    unsigned char test_buf1[128]= {0x00};   
    for(unsigned int i=0;i<512;i++)
    {
        unsigned char a=0;
        while(1)
        {
            a=eeprom_readpage(i*128 , test_buf1, 128);
            if(a==0)
              break;
        }
          
        for(unsigned int j=0;j<128;j++)
        {
            FlashMaster(test_buf1[j]);
            if(updateStatus==0x04)
              break;
        }
        if(updateStatus==0x04)
              break;        
    }
    
}





