//===========================================================================//
//                                                                           //
// 文件：  Control.c                                                         //
// 说明：  功能函数，根据通信协议做相应动作                                  //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "Control.h"
#include "SysInit.h"
#include "Uart.h"

unsigned char O2_status=0x00;//O2传感器状态，0x00表示正常，0x01表示故障
extern int SF6_FailCount=0x00;
unsigned char SF6_status=0x00;//SF6传感器状态，0x00表示正常，0x01表示故障
unsigned char volume_status=0x00;//声音传感器状态，0x00表示正常，0x01表示故障
char O2_FailCount=0x00;
char volume_FailCount=0x00;

extern unsigned char timer0Flag;//定时时间到标志
extern int timer0Count;//定时时间到标志

extern unsigned char O2Sensor[];
extern unsigned char SF6Sensor[];//存放SF6浓度数据
extern unsigned char volumeSensor[];

extern unsigned char Uart0InstructFlag;//接收到O2指令标志
extern unsigned char Uart2InstructFlag;//接收到指令标志
extern unsigned char Uart3InstructFlag;//接收到指令标志
extern unsigned char Uart3InstructNum;//指令编号


/*******************************************************************************
获取数据采集板数据、状态信息
*******************************************************************************/
void DetectState()
{
    unsigned char buffer_1[13];
    buffer_1[0]=0x0d;//长度位
    buffer_1[1]=0x01;//目的地址位
    buffer_1[2]=0x02;//源地址位
    buffer_1[3]=0x01;//指令编号
    
    unsigned int intO2=(O2Sensor[0]-48)*1000+(O2Sensor[1]-48)*100+(O2Sensor[2]-48)*10+(O2Sensor[3]-48)*1;
    buffer_1[4]=(unsigned char)(intO2>>8);
    buffer_1[5]=(unsigned char)intO2;
    
    unsigned int intSF6=(SF6Sensor[0]-48)*10000+(SF6Sensor[1]-48)*1000+(SF6Sensor[2]-48)*100+(SF6Sensor[3]-48)*10+(SF6Sensor[4]-48)*1;
    buffer_1[6]=(unsigned char)(intSF6>>8);
    buffer_1[7]=(unsigned char)intSF6;
    
    buffer_1[8]=volumeSensor[0];
    buffer_1[9]=volumeSensor[1];
    
    if(SF6_FailCount>1000)//认为有故障
      SF6_status=0x01;
    else
      SF6_status=0x00;
    buffer_1[10]=O2_status;
    buffer_1[11]=SF6_status;
    buffer_1[12]=volume_status;
    unsigned char crc = CRC(buffer_1,13);
    
    unsigned char buffer[15];
    buffer[0]=0xfe;
    for(int i=0;i<13;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[14]=crc;
    Uart3_send(buffer,15);
}
/*******************************************************************************
指令处理函数
*******************************************************************************/
void InstructProcess()
{
    if(Uart3InstructFlag==0x01)//有主控指令
    {
        Uart3InstructFlag=0x00;//清标志
        switch (Uart3InstructNum)
        {
        case 0x01:
          DetectState();
          break;
          
        case 0x02:
          
          break;
        case 0x18:
        {
            
        }
     
        default :
          break;
          
          
        }
    }
}

/*******************************************************************************
最小时间片
*******************************************************************************/
void MinTimeslice()
{
    if(Uart3InstructFlag != 0x01)//没有主控指令
    {
        Init_Timer0_A5();
        while(1)
        {
            if(timer0Flag==1)
            {
                timer0Count++;
                if(timer0Count>200)
                  timer0Count=0;
                break;
            }
        }
    }

}
/*******************************************************************************
检测O2值
*******************************************************************************/
void DetectO2()
{
    if(Uart3InstructFlag != 0x01)
    {
        unsigned char bf[4]={0x25 ,0x0d ,0x0a};
        Uart0_send(bf,3);//o2传感器检测指令
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart0InstructFlag == 1)//O2传感器回复
            {
                O2_FailCount=0;
                O2_status=0x00;//O2传感器状态正常
                Uart0InstructFlag = 0;//清标志
            }
            if(timer0Flag==1)
            {
                O2_FailCount++;
                Uart0InstructFlag = 0;//清标志
                timer0Count++;
                break;
            }
        }
        if(O2_FailCount>=10)
        {
          O2_status=0x01;//O2传感器故障
        }
    }
}
/*******************************************************************************
检测声音值
*******************************************************************************/
void DetectVolume()
{
    if(Uart3InstructFlag != 0x01)
    {
        unsigned char bf[8]={0x01 , 0x04 , 0x00 , 0x00 , 0x00 , 0x01 , 0x31 ,0xca};
        Uart2_send(bf,8);
      
        Init_Timer0_A5();//开20ms定时器
        while(1)
        {
            if(Uart2InstructFlag == 1)//声音传感器回复
            {
                volume_FailCount=0;
                volume_status=0x00;//声音传感器状态正常
                Uart2InstructFlag = 0;//清标志
            }
            if(timer0Flag==1)
            {
                volume_FailCount++;
                Uart2InstructFlag = 0;//清标志
                timer0Count++;
                break;
            }
        }
        if(volume_FailCount>=3)
        {
          volume_status=0x01;//噪音传感器故障
        }
    }
}
/*******************************************************************************
按照时序检测状态
*******************************************************************************/
void DetectSensor()
{
    switch(timer0Count)
    {
    case 100:
      DetectO2();
      break;
    case 200:
      DetectVolume();
      break;
    default:
      MinTimeslice();
      SF6_FailCount++;
      break;
    }
}