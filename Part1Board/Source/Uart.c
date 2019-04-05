//===========================================================================//
//                                                                           //
// 文件：  Uart.c                                                            //
// 说明：  串口模块                                                          //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//

#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "Uart.h"
#include "eeprom.h"
#include "Control.h"

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
Uart0相关的数据保存，超声波传感器返回值
*******************************************************************************/
unsigned char Uart0RXBuffer[1];
extern unsigned char distData[2]={0x00};
unsigned char Uart0RXFlag=0x00;//数据接收标志
extern unsigned char Uart0InstructFlag=0x00;//接收到指令标志
extern unsigned char isObstacle=0x00;//有障碍物
/*******************************************************************************
Uart1相关的数据保存，上位机指令，移动和移动到指令需要下发相关信息
*******************************************************************************/
unsigned char Uart1RXBuffer[1];
unsigned char Uart1RXFlag=0X00;//数据接收标志
extern unsigned char Uart1DataBuffer[256]={0x00};//数据保存数组
extern unsigned char Uart1DataBfLength=0;//数据保存长度
unsigned char Uart1DataNum=0;//数据保存计数
extern unsigned char Uart1InstructFlag=0x00;//接收到指令标志
extern unsigned char Uart1InstructNum=0x00;//指令编号

extern unsigned char Uart1moveRes[4]={0x00};//上位机移动指令
extern unsigned char Uart1movetoRes[8]={0x00};//上位机移动到指令
extern unsigned char Uart1ptzRollRes[3]={0x00};//上位机红外云台移动指令
extern unsigned char Uart1ctrlInfraPTZRes[4]={0x00};//上位机红外云台移动到指令
extern unsigned char Uart1UpdateID=0x00;//rom更新设备ID
extern unsigned char Uart1UpdateDataNum=0x00;//rom更新数据序号
extern unsigned char UpdateDataNum=0x00;
extern unsigned char Uart1UpdateStatus=0x00;//rom更新状态位
extern unsigned int addrEeprom=0x0000;//
extern unsigned int Uart1UpdateDataCount=0;
extern unsigned char updateState=0x00;
extern unsigned char updateReady=0x00;
unsigned char dataLost=0x00;


/*******************************************************************************
Uart2相关的数据保存，局放传感器 ，局放值
*******************************************************************************/
unsigned char Uart2RXBuffer[1];
unsigned char Uart2RXFlag=0X00;//数据接收标志
extern unsigned char Uart2DataBuffer[20]={0x00};//数据保存数组
extern unsigned char Uart2DataBfLength=0;//数据保存长度
unsigned char Uart2DataNum=0;//数据保存计数
extern unsigned char Uart2InstructFlag=0x00;//接收到指令标志
extern unsigned char TEV[2];//TEV测量值
extern unsigned char US[2];//局放超声波

/*******************************************************************************
Uart3相关的数据保存,485总线
*******************************************************************************/
unsigned char Uart3RXBuffer[1];
unsigned char Uart3RXFlag=0X00;//数据接收标志
unsigned char Uart3DataBuffer[256];//数据保存数组
unsigned char Uart3DataBfLength=0;//数据保存长度
unsigned char Uart3DataNum=0;//数据保存计数
extern unsigned char Uart3InstructFlag=0x00;//接收到指令标志
extern unsigned char Uart3InstructSource=0x00;//指令发送源地址
unsigned char Uart3InstructAim;//指令发送目的地址
unsigned char Uart3InstructNum;//指令编号


//extern unsigned char ChrgState;//充电状态
extern unsigned char totalMile[32];//总里程信息
extern long totalMileBMQ;
extern long LtotalMile;
extern unsigned char Electricity;//电量
extern unsigned char current[2];//电流
extern unsigned char positionX[4];//X轴坐标
extern unsigned char positionY[2];//Y轴坐标
extern unsigned char positionZ[2];//Z轴坐标
extern unsigned char CyuntaiX[2];//云台舵机位置
extern unsigned char CyuntaiY[2];//云台舵机位置
extern unsigned char O2[2];//氧气浓度
extern unsigned char SF6[2];//SF6气体浓度
extern unsigned char temperature[2];//温度
extern unsigned char humidity[2];//湿度
extern unsigned char volume[2];//噪音值

extern unsigned char motor_status_X;//小车电机状态
extern unsigned char motor_status_Y;//伸缩电机状态
extern unsigned char motor_status_Z;//局放电机状态
extern unsigned char encoder_status_X;//小车板子编码器
extern unsigned char encoder_status_Y;//伸缩杆板子编码器
extern unsigned char encoder_status_Z;//局放板子编码器
extern unsigned char xianwei_status_X;//小车限位状态
extern unsigned char xianwei_status_Y;//伸缩限位状态
extern unsigned char xianwei_status_Z;//局放限位状态
extern unsigned char TEV_status;//tev故障
extern unsigned char US_status;//超声波故障
extern unsigned char SF6_status;//SF6传感器故障
extern unsigned char O2_status;//O2传感器故障
extern unsigned char TaH_status;//温湿度传感器故障
extern unsigned char Volume_status;//噪音传感器故障
extern unsigned char Inf_status;//红外热像故障
extern unsigned char yunatai_status;//云台故障
extern unsigned char camare_status;//摄像头故障
extern unsigned char charge_status;//充电故障

extern unsigned char movetoX_status;//X轴移动到达
extern unsigned char movetoY_status;//y轴移动到达
extern unsigned char movetoZ_status;//z轴移动到达

extern unsigned char isObstacleUart3=0x00;//局放控制板的障碍物信息
//****************************************************************************
//                                                                           *
//                       串口初始化                                          *
//                                                                           *
//****************************************************************************
//---------------------------串口0连接超声波传感器---------------------------//
void Init_UART0(void)
{
    UCA0CTL1  = UCSWRST ; // 状态机复位
    P3SEL |= TXD_US + RXD_US ;  //引脚功能选择
    P3DIR |= TXD_US ;  //设置方向输出
    P3DIR &= ~ RXD_US ;  //设置方向输入
	
    UCA0CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA0BR0   = 0x41                                           ; // 8MHz/9600=833 
    UCA0BR1   = 0x03                                           ; 
    UCA0MCTL  = UCBRS_2 + UCBRF_0                              ; // UCBRSx=2, UCBRFx=0
    
    UCA0CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA0IE    |= UCRXIE                                         ; // 允许接收中断
}
//---------------------------串口1 IPORT口-115200------------------------------//
void Init_UART1(void)
{
    UCA1CTL1  = UCSWRST                                        ; // 状态机复位
    P5SEL |= TXD_IPORT + RXD_IPORT                                ;  //引脚功能选择
    P5DIR |= TXD_IPORT                                           ;  //设置方向输出
    P5DIR &= ~ RXD_IPORT                                        ;  //设置方向输入
    
    UCA1CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA1BR0   = 0x45                                           ; // 8MHz/115200=59 
    UCA1BR1   = 0x00                                           ; 
    UCA1MCTL  = UCBRS_4 + UCBRF_0                              ; // UCBRSx=4, UCBRFx=0
    /*
    UCA1BR0   = 0x8A                                           ; // 8MHz/57600=138 
    UCA1BR1   = 0x00                                           ; 
    UCA1MCTL  = UCBRS_7 + UCBRF_0                              ; // UCBRSx=7, UCBRFx=0
    */
    UCA1CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA1IE    |= UCRXIE                                         ; // 允许接收中断

}
//----------------------------串口2 局放传感器--9600--------------------------//
void Init_UART2(void)
{
    UCA2CTL1  = UCSWRST                                        ; // 状态机复位
    P9SEL |= TXD_JF  + RXD_JF                               ;  //引脚功能选择
    P9DIR |= TXD_JF                                           ;  //设置方向输出
    P9DIR &= ~ RXD_JF                                         ;  //设置方向输入
	
    UCA2CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA2BR0   = 0x41                                           ; // 8MHz/9600=833 
    UCA2BR1   = 0x03                                           ; 
    UCA2MCTL  = UCBRS_2 + UCBRF_0                              ; // UCBRSx=2, UCBRFx=0
    
    UCA2CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA2IE    |= UCRXIE                                         ; // 允许接收中断
		
    UART2_IN;
}
//-----------------------------串口3 485总线--115200----------  --------------//
void Init_UART3(void)
{
    UCA3CTL1  = UCSWRST                                        ; // 状态机复位
    P10SEL |= TXD485BUS  + RXD485BUS                               ;  //引脚功能选择
    P10DIR |= TXD485BUS                                           ;  //设置方向输出
    P10DIR &= ~ RXD485BUS                                         ;  //设置方向输入

    UCA3CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA3BR0   = 0x45                                           ; // 8MHz/115200=69 
    UCA3BR1   = 0x00                                           ; 
    UCA3MCTL  = UCBRS_4 + UCBRF_0                              ; // UCBRSx=4, UCBRFx=0
    
    UCA3CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA3IE    |= UCRXIE                                         ; // 允许接收中断
    
    UART3_IN;
}
/*******************************************************************************
UART0模块的数据发送
*******************************************************************************/
void Uart0_send(unsigned char *tx_buf,unsigned int length)
{
  unsigned int i ;
  for(i=0;i<length;i++)
  {
    UCA0TXBUF = *tx_buf++ ; 
    while (!(UCA0IFG&UCTXIFG)) ; 
  }
}
/*******************************************************************************
UART1模块的数据发送
*******************************************************************************/
void Uart1_send(unsigned char *tx_buf,unsigned int length)
{
  unsigned int i;
  for(i=0;i<length;i++)
  {
    UCA1TXBUF = *tx_buf++ ; 
    while (!(UCA1IFG&UCTXIFG)) ; 
  }
}
/*******************************************************************************
UART2模块的数据发送，局放传感器 485
*******************************************************************************/
void Uart2_send(unsigned char *tx_buf,unsigned int length)
{
  unsigned int i ;
  UART2_OUT;
  for(i=0;i<length;i++)
  {
    UCA2TXBUF = *tx_buf++ ; 
    while (!(UCA2IFG&UCTXIFG)) ; 
  }
  __delay_cycles(8000) ;
  UART2_IN ;
}
/*******************************************************************************
UART3模块的数据发送 485总线
*******************************************************************************/
void Uart3_send(unsigned char *tx_buf,unsigned int length)
{
  UART3_OUT;
  unsigned int i ;
  for(i=0;i<length;i++)
  {
    UCA3TXBUF = *tx_buf++ ; 
    while (!(UCA3IFG&UCTXIFG)) ; 
  }
  __delay_cycles(800) ;
  UART3_IN;
}

/*******************************************************************************
UART0模块的数据接收 超声波传感器
*******************************************************************************/
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV,4))
    {
        case 0:break                                                     ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart0RXBuffer[0] = UCA0RXBUF ;
            switch(Uart0RXFlag)
            {
            case 0x00:
              if(Uart0RXBuffer[0] == 0xFF)//数据头
              {
                  Uart0RXFlag = 0x01;//找到数据头
              }
              break;
            case 0x01:
              distData[0] = Uart0RXBuffer[0];//距离数据高位
              Uart0RXFlag = 0x02;
              break;
            case 0x02:
              distData[1] = Uart0RXBuffer[0];//距离数据低位
              Uart0RXFlag = 0x03;
              break;
            case 0x03:
              if(((distData[0]+distData[1])&0xFF) == Uart0RXBuffer[0])//校验位
              {
                Uart0InstructFlag=0x01;
                /*
                unsigned int dist=distData[0]*256+distData[1];
                if((dist>250)&&(dist<500))
                {
                  isObstacle=0x01;//有障碍物
                }
                else 
                {
                  isObstacle=0x00;//无障碍物
                }
                */
              }
              Uart0RXFlag = 0x00;
              break;
            default :
              Uart0RXFlag = 0x00;
              break;
            }
            break ;
        case 4:break                                                     ;  // Vector 4 - TXIFG
        default: break                                                   ;  
    }  
}
/*******************************************************************************
UART1模块的数据接收 IPORT口 接收上位机命令
*******************************************************************************/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV,4))
    {
        case 0:break                                                     ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart1RXBuffer[0] = UCA1RXBUF ;
            //Uart1_send(Uart1RXBuffer,1);
            switch (Uart1RXFlag)
            {
            case 0x00:
              if(Uart1RXBuffer[0]==0xFE)
              {
                Uart1RXFlag=0x01;//找到数据头
                //Uart1_send(Uart1RXBuffer,1);
              }
              break;
            case 0x01://数据长度
              
              Uart1DataBfLength=Uart1RXBuffer[0];//记录数据长度
              Uart1DataNum=0;//用于计数
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//保存数据长度
              Uart1RXFlag=0x02;//找到数据长度位
              //Uart1_send(Uart1RXBuffer,1);
              break;
            case 0x02://指令编号
              Uart1DataNum++;//用于计数
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//保存指令编号
              //Uart1InstructNum=Uart1RXBuffer[0];//保存指令编号
              Uart1RXFlag=0x03;//
              //Uart1_send(Uart1RXBuffer,1);
              break;
            case 0x03://数据位和CRC校验位
              
              Uart1DataNum++;//用于计数
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//数据位或CRC校验位
              //Uart1_send(Uart1RXBuffer,1);
              if(Uart1DataBfLength<=Uart1DataNum)//接收预计数量的数据
              {
                unsigned char crc=CRC(Uart1DataBuffer,Uart1DataBfLength);//保存CRC位
                //Uart1_send(&crc,1);
                //Uart1_send(Uart1DataBuffer,Uart1DataBfLength+1);
                if(crc == Uart1RXBuffer[0])
                {
                  Uart1InstructFlag=0x01;
                  Uart1InstructNum=Uart1ExtrInstr(Uart1DataBuffer ,Uart1DataBfLength);
                  
                  if(Uart1InstructNum==0x18)//软件更新指令
                  {
                      if(Uart1UpdateDataNum==0x00)//第一帧
                      {
                          updateState=0x01;//软件更新状态
                          UpdateDataNum=Uart1UpdateDataNum;
                          addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart1UpdateDataNum*250),Uart1DataBuffer+5 ,(unsigned char)(Uart1DataBfLength-5));
                          dataLost=0x00;
                      }
                      if(Uart1UpdateDataNum!=0x00)//不是第一帧
                      {
                          if(UpdateDataNum+1==Uart1UpdateDataNum)//连续帧
                          {
                            UpdateDataNum=Uart1UpdateDataNum;
                            addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart1UpdateDataNum*250),Uart1DataBuffer+5 ,(unsigned char)(Uart1DataBfLength-5));
                          }
                          else if(UpdateDataNum==Uart1UpdateDataNum)//重复帧
                          {
                          }
                          else
                            dataLost=0x01;//程序包数据丢失
                          
                      }  
                      __delay_cycles(8000);
                      unsigned char buffer_1[3]={0x03 , 0x18, Uart1UpdateDataNum};
                      unsigned char crc=CRC(buffer_1,3);
                      unsigned char buffer[5]={0xFE , 0x03 , 0x18 , Uart1UpdateDataNum , crc};
                      Uart1_send(buffer,5);
                      
                      if(Uart1UpdateStatus==0x01)//最后一帧
                      {
                          if(dataLost==0x00)//没有丢包，程序接接受完整
                          {
                              if(Uart1UpdateID==0x01)//主控板，关中断进入Bootloader函数
                              {
                                  delay_ms(5);
                                  unsigned char c=0x55;
                                  eeprom_writepage(0xFFFF,&c,1);//写标志，告诉引导代码有现成的程序，可以直接更新
                                  
                                  UCA0IE &= ~UCRXIE ; // 关闭接收中断
                                  UCA1IE &= ~UCRXIE ; // 关闭接收中断
                                  UCA2IE &= ~UCRXIE ; // 关闭接收中断
                                  UCA3IE &= ~UCRXIE ; // 关闭接收中断
                                  TA0CCTL0 &= ~CCIE ; // 关闭时钟中断
                                  TB0CCTL0 &= ~CCIE ; //
                                  WDTCTL = 0;
                              }
                              else//子控版
                              {
                                  //计算一下要更新的软件大小
                                  Uart1UpdateDataCount = Uart1UpdateDataNum*250+(unsigned int)(Uart1DataBfLength-5);
                                  updateReady=0x01;
                                  //UpdateProgram();
                              }
                          }
                          else 
                            updateState=0x00;//数据包丢失，不进行后续更新，正常执行自身程序
                      }
                  }
                }
                else
                {
                  Uart1InstructFlag=0x00;
                }
                Uart1RXFlag=0x00;
              }
              break;
            default :
              Uart1RXFlag=0x00;
              break;
            }
            
            break ;
        case 4:break                                                     ;  // Vector 4 - TXIFG
        default: break                                                   ;  
    }  
}

/*******************************************************************************
串口2数据接收模块 局放传感器
*******************************************************************************/
#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
    switch(__even_in_range(UCA2IV,4))
    {
        case 0:break ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart2RXBuffer[0] = UCA2RXBUF ;
            switch(Uart2RXFlag)
            {
            case 0x00:
            {
                if(Uart2RXBuffer[0]==0x01)//设备地址是01
                {
                    Uart2RXFlag=0x01;
                    Uart2DataNum=0;
                    Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//保存设备地址位
                }
                break;
            }
            case 0x01:
            {
                if(Uart2RXBuffer[0]==0x03)//功能码
                {
                    Uart2RXFlag=0x02;
                    Uart2DataNum++;
                    Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//保存功能码位
                }
                else 
                    Uart2RXFlag=0x00;
                break;
            }
            case 0x02:
            {
                Uart2DataBfLength=Uart2RXBuffer[0];//记录数据长度
                Uart2DataNum++;
                Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//保存数据长度
                Uart2RXFlag=0x03;//找到数据长度位
                break;
            }
            case 0x03:
            {
                Uart2DataNum++;//计数
                Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//保存数据长度
                if(Uart2DataNum-2==Uart2DataBfLength)
                  Uart2RXFlag=0x04;//数据位保存完毕
                break;
            }
            case 0x04:
            {
                Uart2DataNum++;
                Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//保存CRC16校验高位
                Uart2RXFlag=0x05;
                break;
            }
            case 0x05:
            {
                Uart2DataNum++;
                Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];//保存CRC16校验低位
                Uart2RXFlag=0x00;
                
                unsigned int crc=Uart2DataBuffer[Uart2DataNum-1]*256+Uart2DataBuffer[Uart2DataNum];
                if(crc==CRC16RTU(Uart2DataBuffer,Uart2DataBfLength+3))
                {
                    Uart2InstructFlag=0x01;//接收成功
                    //提取传感器值
                    TEV[0]=Uart2DataBuffer[3];
                    TEV[1]=Uart2DataBuffer[4];
                    US[0]=Uart2DataBuffer[7];
                    US[1]=Uart2DataBuffer[8];
                }
                else
                {
                    Uart2InstructFlag=0x00;//接收失败
                }
                break;
            }
            default :
              Uart2RXFlag=0x00;
              break;
            }
            break ;
        case 4:break ;  // Vector 4 - TXIFG
        default: break ;  
    }
}
/*******************************************************************************
串口3数据接收模块 485总线
*******************************************************************************/
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
            case 0x00://数据头
              if(Uart3RXBuffer[0]==0xFE)
              {
                  Uart3RXFlag=0x01;
              }
              break;
            case 0x01://数据长度位
              Uart3DataBfLength=Uart3RXBuffer[0];//记录数据长度
              Uart3DataNum=0;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//保存数据长度
              Uart3RXFlag=0x02;//找到数据长度位
              break;
            case 0x02://目的地址
              Uart3DataNum++;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//保存目的地址
              Uart3InstructAim=Uart3RXBuffer[0];
              Uart3RXFlag=0x03;//找到数据长度位
              break;
            case 0x03://源地址
              Uart3DataNum++;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//保存源地址
              Uart3InstructSource=Uart3RXBuffer[0];
              Uart3RXFlag=0x04;//找到数据长度位
              break;
            case 0x04://指令编号
              Uart3DataNum++;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//保存指令编号
              Uart3InstructNum=Uart3RXBuffer[0];
              Uart3RXFlag=0x05;//找到数据长度位
              break;
            case 0x05://数据位和CRC位
              Uart3DataNum++;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//数据位
              if(Uart3DataBfLength<=Uart3DataNum)//接收预计数量的数据
              {
                unsigned char crc=Uart3RXBuffer[0];//保存CRC位
                if(crc == CRC(Uart3DataBuffer,Uart3DataBfLength))
                {
                  Uart3InstructFlag=0x01;
                  Uart3InstructNum=Uart3ExtrInstr(Uart3DataBuffer ,Uart3DataBfLength);//提取指令中有用数据
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
计算CRC校验位

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
/******************************************************
*函数名称:CRC16RTU
*输   入:pszBuf  要校验的数据
        unLength 校验数据的长
*输   出:校验值
*功   能:循环冗余校验-16
         （RTU标准-0xA001）
*******************************************************/
unsigned int CRC16RTU( unsigned char * pszBuf, unsigned int unLength)
{
    unsigned int CRC=0XFFFF;
    unsigned long CRC_count;
    for(CRC_count=0;CRC_count<unLength;CRC_count++)
    {
        int i;
        CRC=CRC^*(pszBuf+CRC_count);

        for(i=0;i<8;i++)
        {
            if(CRC&1)
            {
                CRC>>=1;
                CRC^=0xA001;
            }
            else
            { 
                CRC>>=1;
            }
        }
    }
    return CRC;
}

/*******************************************************************************
对上位机传来的指令进行提取
*******************************************************************************/
unsigned char Uart1ExtrInstr(unsigned char *buffer ,int length)
{
    switch (buffer[1])
    {

        case 0x04://复位请求指令
            return 0x04;

        case 0x05://机器人版本信息请求指令
            return 0x05;

        case 0x06://机器人充电状态请求指令
            return 0x06;

        case 0x07://机器人运动总里程请求指令
            return 0x07;

        case 0x08://机器人状态请求指令
            return 0x08;
            
        case 0x09://电量信息请求指令
            return 0x09;
            
        case 0x0A://移动机器人请求指令，保存运动轴、运动方向、运动速度信息
            for (int i=0;i<4;i++)
            {
                Uart1moveRes[i] = buffer[i+2];
            }            
            return 0x0A;
            
        case 0x0B://停止移动机器人请求指令
            return 0x0B;
            
        case 0x0C://紧急停止移动机器人请求指令
            return 0x0C;
            
        case 0x0D://获取坐标信息指令
            return 0x0D;
            
        case 0x0E://移动机器人到指定坐标请求指令，保存三维坐标
            for(int i=0;i<8;i++)
            {
                Uart1movetoRes[i] = buffer[i+2];
            }
            
            return 0x0E;

        case 0x0f://红外云台转动请求指令，保存方向和速度
            for (int i=0;i<3;i++)
            {
                Uart1ptzRollRes[i] = buffer[2+i];
            }
            return 0x0f;
                    
        case 0X10://停止红外云台转动请求指令
            return 0x10;

        case 0x11://云台控制请求指令，保存二维坐标
            for (int i=0;i<4;i++)
            {
                Uart1ctrlInfraPTZRes[i] = buffer[2+i];
            }
            return 0x11;

        case 0x12://获取红外云台位置请求指令
            return 0x12;

        case 0x13://局放（TEV）检测请求指令
            return 0x13;
            
        case 0x14://局放（超声波）检测请求指令
            return 0x14;
            
        case 0x15://O2浓度检测请求指令
            return 0x15;
            
        case 0x16://SF6浓度检测请求指令
            return 0x16;
            
        case 0x17://环境参数检测请求指令
            return 0x17;
        case 0x18://Update请求指令
            Uart1UpdateID=buffer[2];//设备ID
            Uart1UpdateDataNum=buffer[3];//数据序号
            Uart1UpdateStatus=buffer[4];//状态位
            
            return 0x18;
        default:
            return 0x00;//表示提取数据失败
    }
}

/*******************************************************************************
对子控制板传来的指令进行提取
*******************************************************************************/
unsigned char Uart3ExtrInstr(unsigned char *buffer ,int length)
{
    switch (buffer[3])//指令编号
    {
        case 0x01://状态查询指令
            if(buffer[2]==0x02)//数据采集板回复
            {
                O2[0]=buffer[4];
                O2[1]=buffer[5];
                SF6[0]=buffer[6];
                SF6[1]=buffer[7];
                volume[0]=buffer[8];
                volume[1]=buffer[9];
                O2_status=buffer[10];
                SF6_status=buffer[11];
                Volume_status=buffer[12];
            }
            else if(buffer[2]==0x03)//局放运动控制板数据回复
            {
                positionZ[0]=buffer[4];//电动推杆位置
                positionZ[1]=buffer[5];
                motor_status_Z=buffer[6];//Z电机状态
                encoder_status_Z=buffer[7];//Z轴编码器状态
                movetoZ_status=buffer[8];//
                CyuntaiX[0]=buffer[9];//云台位置
                CyuntaiX[1]=buffer[10];
                CyuntaiY[0]=buffer[11];//云台位置
                CyuntaiY[1]=buffer[12];
                yunatai_status=buffer[13];//云台状态
                isObstacleUart3=buffer[14];//障碍物信息
            }
            else if(buffer[2]==0x04)//小车和伸缩杆运动控制板数据回复
            {
                positionX[0]=buffer[4];//小车位置
                positionX[1]=buffer[5];
                positionX[2]=buffer[6];
                positionX[3]=buffer[7];
                motor_status_X=buffer[8];//X电机状态
                encoder_status_X=buffer[9];//小车编码器状态
                movetoX_status=buffer[10];
                
                positionY[0]=buffer[11];//伸缩杆位置
                positionY[1]=buffer[12];
                motor_status_Y=buffer[13];//Y电机状态
                encoder_status_Y=buffer[14];//伸缩杆编码器状态
                movetoY_status=buffer[15];
                
                humidity[0]=buffer[16];//湿度
                humidity[1]=buffer[17];//湿度
                temperature[0]=buffer[18];//温度
                temperature[1]=buffer[19];//温度
                
                TaH_status=buffer[20];//温湿度传感器状态
                
                //总里程
                for(int i=0;i<10;i++)
                {
                    totalMile[i]=buffer[21+i];
                }
            }
            else if(buffer[2]==0x05)//电源管理板数据回复
            {
                charge_status=buffer[4];
                Electricity=buffer[5];
                current[0]=buffer[6];
                current[1]=buffer[7];
            }
            else{}
              
            return 0x01;
            
        case 0x02://
            return 0x02;
                    
        case 0x03://
            return 0x03;

        case 0x04://
            return 0x04;

        case 0x05://
            return 0x05;

        case 0x06://
            return 0x06;

        case 0x07://
            return 0x07;

        case 0x08://
            return 0x08;
            
        case 0x09://
            return 0x09;

        default:
            return 0x00;//
    }
}





