//===========================================================================//
//                                                                           //
// 文件：  Control.c                                                         //
// 说明：  功能模块                                                          //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "Control.h"
#include "Uart.h"
#include "SysInit.h"
#include "PIN_DEF.h"
#include "eeprom.h"

extern unsigned char Uart0InstructFlag;//接收到指令标志
extern unsigned char distData[2];
extern unsigned char isObstacle;//有障碍物

extern unsigned char timer0Flag;//定时时间到标志
extern unsigned char Uart1InstructFlag;//接收到指令标志
extern unsigned char Uart1InstructNum;//指令编号
extern unsigned char Uart1moveRes[4];//上位机移动指令
extern unsigned char Uart1movetoRes[8];//上位机移动到指令
extern unsigned char Uart1ptzRollRes[3];//上位机红外云台移动指令
extern unsigned char Uart1ctrlInfraPTZRes[4];//上位机红外云台移动到指令

extern unsigned char Uart1UpdateID;//rom更新设备ID
extern unsigned char Uart1UpdateDataNum;//rom更新数据序号
extern unsigned char UpdateDataNum;
extern unsigned char Uart1UpdateStatus;//rom更新状态位
extern unsigned int addrEeprom;//
extern unsigned int Uart1UpdateDataCount;

extern unsigned char Uart2InstructFlag;//接收到指令标志

extern unsigned char Uart3InstructFlag;//接收到指令标志
extern unsigned char Uart3InstructSource;//指令发送源地址
extern unsigned char isObstacleUart3;//局放控制板的障碍物信息

unsigned char moveRecfail=0;//move函数接受失败
unsigned char stopRecfail=0;//stop函数接受失败
unsigned char emergencyStopfail=0;//紧急停止函数接收失败
unsigned char infraPtzRollFail=0;//转动云台函数接收失败
unsigned char infraPtzStopFail=0;//停止云台转动函数接收失败
unsigned char InfraPtzRollToRecfail=0;//移动云台到位接收失败
unsigned char detectStateFail_02=0;//获取小车板状态信息失败
unsigned char detectStateFail_03=0;//获取伸缩板状态信息失败
unsigned char detectStateFail_04=0;//获取局放板状态信息失败
unsigned char detectStateFail_05=0;//获取电源板状态信息失败
unsigned char detectStateFailCount_02=0;//获取小车板状态信息失败计数
unsigned char detectStateFailCount_03=0;//获取伸缩板状态信息失败计数
unsigned char detectStateFailCount_04=0;//获取局放板状态信息失败计数
unsigned char detectStateFailCount_05=0;//获取电源板状态信息失败计数

unsigned char movetoXYZ=0;//移动到指定位置指令
unsigned char movetoXsend=0;//
unsigned char movetoYsend=0;//
unsigned char movetoZsend=0;//
unsigned char movetoX=0;//X轴移动到达
unsigned char movetoY=0;//y轴移动到达
unsigned char movetoZ=0;//z轴移动到达

unsigned char moveDir=0x00;//0x00表示停止，0x01表示前，0x02表示后，0x03表示下


int yuntaiX=9300;//对应X轴云台的90度
int yuntaiY=11500;//对应Y轴云台的90度

unsigned char yuntaiRollUp=0;
unsigned char yuntaiRollDown=0;
unsigned char yuntaiRollLeft=0;
unsigned char yuntaiRollRight=0;


extern unsigned char movetoX_status=0;//X轴移动到达
extern unsigned char movetoY_status=0;//y轴移动到达
extern unsigned char movetoZ_status=0;//z轴移动到达

extern unsigned char motor_status_X=0x00;//小车电机状态
extern unsigned char motor_status_Y=0x00;//伸缩电机状态
extern unsigned char motor_status_Z=0x00;//局放电机状态
extern unsigned char encoder_status_X = 0x00;//小车板子编码器
extern unsigned char encoder_status_Y = 0x00;//伸缩杆板子编码器
extern unsigned char encoder_status_Z = 0x00;//局放板子编码器
extern unsigned char xianwei_status_X=0x00;//小车限位状态
extern unsigned char xianwei_status_Y=0x00;//伸缩限位状态
extern unsigned char xianwei_status_Z=0x00;//局放限位状态
extern unsigned char TEV_status=0x00;//tev故障
extern unsigned char US_status=0x00;//超声波故障
extern unsigned char SF6_status=0x00;//SF6传感器故障
extern unsigned char O2_status=0x00;//O2传感器故障
extern unsigned char TaH_status=0x00;//温湿度传感器故障
extern unsigned char Volume_status=0x00;//噪音传感器故障
extern unsigned char Inf_status=0x00;//红外热像故障
extern unsigned char yunatai_status=0x00;//云台故障
extern unsigned char camare_status=0x00;//摄像头故障
extern unsigned char charge_status=0x00;//充电故障
char TEV_FailCount=0x00;
char US_FailCount=0x00;

unsigned char movetoXYZ_Ok=0;    //指令成功完成标志

extern unsigned char MoveToRES_X=0;                      //X轴位置指令回复指示变量 
extern unsigned char MoveToRES_Y=0;                      //Y轴位置指令回复指示变量 
extern unsigned char MoveToRES_Z=0;                      //Z轴位置指令回复指示变量 


unsigned char isObstacleBack1=0x00;//右侧障碍物
unsigned char isObstacleBack2=0x00;//右侧障碍物
unsigned char isObstacleBack3=0x00;//右侧障碍物
unsigned char isObstacleBack4=0x00;//右侧障碍物
char isObstacleBackCount=0x00;//右侧障碍物计数
unsigned char isObstacleBack=0x00;//右侧障碍物
unsigned char isObstacleFront=0x00;//左侧障碍物
unsigned char isObstacleBelow=0x00;//下面障碍物
unsigned char chargeState5=0x00;//充电状态，发给电源控制板，0表示不需充电，1表示开始充电，2表示停止充电
unsigned char lowPower=0x00;//低电量状态
unsigned char goHome=0x00;//回原点标志，当接收到moveto(0,0,0)时候置位

unsigned char obstacleState=0x00;//障碍物状态，0000 0000表示无障碍物，0x01表示前方障碍物，0x02表示后方有障碍物
                                  //0x03表示前后方都有障碍物
unsigned char detectNum=0;       //用来决定进行哪块板子的检测
/*******************************************************************************
向上位机传送相关数据，由主控板或者其他控制板得到，由IPORT口返回
*******************************************************************************/
extern unsigned char ChrgStateNow=0x00;//充电状态
extern unsigned char totalMile[32]={0x00};//总里程信息
extern long totalMileBMQ=0;
extern long LtotalMile=0;
extern unsigned char Electricity=0x00;//电量
extern unsigned char current[2]={0x00};//电流
extern unsigned char positionX[4]={0x00};//X轴坐标
extern unsigned char positionY[2]={0x00};//Y轴坐标
extern unsigned char positionZ[2]={0x00};//Z轴坐标
extern unsigned char CyuntaiX[2]={0x00};
extern unsigned char CyuntaiY[2]={0x00};
extern unsigned char TEV[2]={0x00};//TEV测量值
extern unsigned char US[2]={0x00};//局放超声波
extern unsigned char O2[2]={0x00};//氧气浓度
extern unsigned char SF6[2]={0x00};//SF6气体浓度
extern unsigned char temperature[2]={0x00};//温度
extern unsigned char humidity[2]={0x00};//湿度
extern unsigned char volume[2]={0x00};//噪音值

int detectStateOverTime_01=0;//小车板状态检测超时
int detectStateOverTime_02=0;//伸缩板状态检测超时
int detectStateOverTime_03=0;//局放板状态检测超时
int detectStateOverTime_04=0;//电源板状态检测超时 
/*******************************************************************************
接收到复位指令后动作
*******************************************************************************/
void  Reset()
{
    unsigned char bf[4]={0xfe, 0x02, 0x04, 0x1d};
    Uart1_send(bf,4); 
    
    int num=0;
    unsigned char resetFlag=0x00;
    while(num<=3)
    {
        num++;
        unsigned char buffer[6]={0xfe, 0x04, 0x04, 0x01, 0x08, 0x6a};
        Uart3_send(buffer,6); 
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                resetFlag=0x00;
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//小车和伸缩杆运动控制板回复
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                resetFlag=0x01;
                break;
            }
            else{}
        }
        if(resetFlag==0x01)
        {
          break;
        }
    }
}
/*******************************************************************************
获取版本信息，主控板接收到该信息后直接回复上位机硬件版本信息，
32个字节，版本信息存高字节
*******************************************************************************/
void GetVersionReq()
{
    unsigned char versionInf[32]={'H','-','V','e','r','1','.','0',';','S','-','V','e','r','1','.','0'};
    unsigned char bf[34];
    bf[0]=0x22;
    bf[1]=0x05;
    for (int i=0;i<32;i++)
    {
        bf[2+i]=versionInf[i];
    }
    unsigned char crc =CRC(bf,34);
    unsigned char buffer[36];
    buffer[0]=0xFE;
    for (int i=0;i<34;i++)
    {
        buffer[1+i]=bf[i];
    }
    buffer[35]=crc;
    Uart1_send(buffer,36);
}
/*******************************************************************************
获取机器人充电状态信息，
主控板接收到该指令后，先向电源板发出充电状态信息查询指令
电源板反馈回来充电状态
主控板再把这个信息反馈给上位机
一个字节，0X00表示未充电，0X01表示充电，0x02表示获取失败
*******************************************************************************/
void GetChrgStateReq()
{
    //ChrgState=0X00;
    if(detectStateOverTime_04==1)//获取电源板数据失败
    {
        ChrgStateNow=0X02;
    }
    unsigned char bf_1[3]={0x03, 0x06, ChrgStateNow};
    unsigned char crc=CRC(bf_1,3);
    unsigned char buffer[5]={0xFE, 0x03, 0x06, ChrgStateNow, crc};//返回查询结果
    
    Uart1_send(buffer,5);
}
/*******************************************************************************
e.获取机器人运行总里程信息请求
主控板接收到该指令后，向小车板发送请求指令
等待小车板回复相关信息，再将该信息打包发给上位机
*******************************************************************************/
void GetODOReq()
{
    int j=0;
    
    unsigned char bf_1[34];
    bf_1[0]=0x22;
    bf_1[1]=0x07;
    for(int i=0;i<9;i++)
    {
        if(totalMile[i]!=0x30)
        {
            j=10-i;
            break;
        }
    }
    for(int i=0;i<32;i++)
    {
        if(i<j)
        {
            bf_1[i+2]=totalMile[i+10-j];
        }
        else
        {
            bf_1[i+2]=0x00;
        }
      
    }
    if(j==0)
    {
        bf_1[2]=0x30;
    }
    if(detectStateOverTime_01==1)//获取小车板数据失败
    {
        totalMile[0]=0x00;
    }
    /*
    bf_1[2]=0x31;
    bf_1[3]=0x32;
    bf_1[4]=0x33;
    bf_1[5]=0x34;
    bf_1[6]=0x35;
    */
    unsigned char crc=CRC(bf_1,34);
    unsigned char buffer[36];
    buffer[0]=0xFE;
    for(int i=0;i<34;i++)
    {
         buffer[i+1]=bf_1[i];
    }
    buffer[35]=crc;
    
    Uart1_send(buffer,36);
}
/*******************************************************************************
f.获取机器人状态请求
*******************************************************************************/
void GetStatusReq()
{
    unsigned char status[4]={0x00,0x00,0x00,0x00};
    /*
    if(detectStateOk_02==0)
    {
        //motor_status_X=0x01;
        bianmaqi_status_X = 0x01;
        //xianwei_status_X=0x01;
        TaH_status=0x01;
    }
    if(detectStateOk_03==0)
    {
        //motor_status_Y=0x01;
        bianmaqi_status_Y = 0x01;
        //xianwei_status_Y=0x01;
    }
    if(detectStateOk_04==0)
    {
        //motor_status_Z=0x01;
        bianmaqi_status_Z = 0x01;
        //xianwei_status_Z=0x01;
        TEV_status=0x01;
        US_status=0x01;
        Volume_status=0x01;
    }
    */
    status[3] = motor_status_X | (encoder_status_X<<1) | (xianwei_status_X<<2) | (encoder_status_Y<<3) 
      | (xianwei_status_Y<<4) | (motor_status_Y<<5) | (motor_status_Z<<6) | (encoder_status_Z<<7);
    status[2] = (xianwei_status_Z) | (TEV_status<<1) | (US_status<<2) | (SF6_status<<3)
      | (O2_status<<4) | (TaH_status<<5) | (Volume_status<<6) | (Inf_status<<7);
    status[1] = (yunatai_status) | (camare_status<<1) | (charge_status<<2)| (detectStateFail_02<<3) 
      | (detectStateFail_03<<4) | (detectStateFail_04<<5) | (detectStateFail_05<<6);
    unsigned char buffer_1[7];
    buffer_1[0]=0x07;
    buffer_1[1]=0x08;
    buffer_1[2]=0x01;
    buffer_1[3]=status[0];
    buffer_1[4]=status[1];
    buffer_1[5]=status[2];
    buffer_1[6]=status[3];
    unsigned char crc=CRC(buffer_1,7);
    
    //unsigned char buffer[]={0xFE, 0x07, 0x08, 0x01,0x00 ,0x00 ,0x00 ,0x00 , 0xf4};
    unsigned char buffer[9];
    buffer[0]=0xfe;
    for(int i=0;i<7;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[8]=crc;
    Uart1_send(buffer,9);
    
}
/*******************************************************************************
g.获取电池电量请求
主控板接收到该指令后，向电源板发送获取电池电量请求，
等待电源板回复相关信息，再将该信息打包发给上位机
*******************************************************************************/
void GetBatteryReq()
{ 
    if(detectStateFail_05==1)//获取电源板数据失败
    {
        Electricity=0XFF;
    }
    //Electricity=50;
    unsigned char bf_1[3]={0x03, 0x09, Electricity};
    unsigned char crc=CRC(bf_1,3);
    unsigned char buffer[5]={0xFE, 0x03, 0x09, Electricity, crc};//返回接收失败
    Uart1_send(buffer,5);
}
/*******************************************************************************
h.移动机器人
主控板接收到该指令后，先判断移动轴，再决定给那块运动板发送请求指令
然后等待该板子回复接收到信息，再回复上位机接收到
*******************************************************************************/
void Move()
{
    int num=0;
    unsigned char buffer[4]={0xFE, 0x02, 0x0a, 0x02};
    Uart1_send(buffer,4);//回复接收到指令
    
    if(lowPower==0x01)//电量过低的时候，move指令不起作用 
    {
      return;
    }
    if((ChrgStateNow==0x01)&&(Electricity<30))//充电状态且电量小于50，move指令不起作用 （出航条件）
    {
      return;
    }
    ChrgStateNow=0x00;//充电状态清零
    
    //判断是否有移动到指令，有的话需要将移动到指令认为失败，并通知下位机和各子控版
    if(movetoXYZ == 1)//
    {
      movetoXYZ_Ok = 2;//向上位机回复移动到位置指令执行失败；向各个子控板发送位置任务完成指令，提醒其清除相关状态
      MoveToXYZ_RES();
      MoveToXYZ_END();
      movetoX_status = 0;
      movetoY_status = 0;
      movetoZ_status = 0;
      movetoXYZ = 0;
      movetoXYZ_Ok = 0;
      
      moveDir=0x00;//停止
    }
    
    if ((Uart1moveRes[0]==0x00)||(Uart1moveRes[0]==0x01))//移动小车轴、伸缩杆指令
    {
        if((Uart1moveRes[0]==0x00)&&(Uart1moveRes[1]==0x00))//小车向前移动
            moveDir=0x01;
        if((Uart1moveRes[0]==0x00)&&(Uart1moveRes[1]==0x01))//小车向前移动
            moveDir=0x02;
        if((Uart1moveRes[0]==0x01)&&(Uart1moveRes[1]==0x00))//向下移动
            moveDir=0x03;
        if((Uart1moveRes[0]==0x01)&&(Uart1moveRes[1]==0x01))//小车向上移动
            moveDir=0x04;
        if((Uart1moveRes[0]==0x02)&&(Uart1moveRes[1]==0x00))//伸
            moveDir=0x05;
        if((Uart1moveRes[0]==0x02)&&(Uart1moveRes[1]==0x01))//缩
            moveDir=0x06;
        unsigned char bf_1[8];
        bf_1[0]=0x08;
        bf_1[1]=0x04;
        bf_1[2]=0x01;
        bf_1[3]=0x02;
        bf_1[4]=Uart1moveRes[0];//移动轴
        bf_1[5]=Uart1moveRes[1];//移动方向
        bf_1[6]=Uart1moveRes[2];//移动速度
        bf_1[7]=Uart1moveRes[3];//移动速度
        unsigned char crc=CRC(bf_1,8);
        unsigned char bf[10];
        bf[0]=0xfe;
        for(int i=0;i<8;i++)
        {
            bf[i+1]=bf_1[i];
        }
        bf[9]=crc;
        
        num=0;
        while(num<3)
        {
            num++;
            moveRecfail=1;//接收失败
            
            Uart3_send(bf,10);//向小车板发送机器人移动指令
        
            Init_Timer0_A5();//开10ms定时器
            
            while(1)
            {
                if(timer0Flag==1)
                {
                    Uart3InstructFlag = 0;//清标志
                    Uart3InstructSource = 0x00;//清标志
                    moveRecfail=1;//接收失败
                    break;
                }
                if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//小车和伸缩杆运动控制板回复
                {
                    moveRecfail=0;//接收成功
                    Uart3InstructFlag = 0;//清标志
                    Uart3InstructSource = 0x00;//清标志
                    break;
                }
                else{}
            }
            if(moveRecfail==0)
            {
                
                break;
            }
        }
        
        
    }
    else if (Uart1moveRes[0]==0x02)//移动局放轴指令
    {
        unsigned char bf_1[7];
        bf_1[0]=0x07;
        bf_1[1]=0x03;
        bf_1[2]=0x01;
        bf_1[3]=0x02;
        bf_1[4]=Uart1moveRes[1];//移动方向
        bf_1[5]=Uart1moveRes[2];
        bf_1[6]=Uart1moveRes[3];
        unsigned char crc=CRC(bf_1,7);
        unsigned char bf[9];
        bf[0]=0xfe;
        for(int i=0;i<7;i++)
        {
            bf[i+1]=bf_1[i];
        }
        bf[8]=crc;
        
        num=0;
        while(num<3)
        {
            num++;
            moveRecfail=1;
            
            Uart3_send(bf,9);//向局放板发送机器人移动指令
        
            Init_Timer0_A5();//开10ms定时器
            
            while(1)
            {
                if(timer0Flag==1)
                {
                    Uart3InstructFlag = 0;//清标志
                    Uart3InstructSource = 0x00;//清标志
                    moveRecfail=1;//接收失败
                    break;
                }
                if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//小车板回复
                {
                    moveRecfail=0;//接收成功
                    Uart3InstructFlag = 0;//清标志
                    Uart3InstructSource = 0x00;//清标志
                    break;
                }
                else{}
            }
            if(moveRecfail==0)
            {
                
                break;
            }
        }
        
        
    }
    else //指令错误
    {
      
    }
}
/*******************************************************************************
i.停止移动机器人
主控板接收到该指令后，向各运动板发送停止指令
因担心485总线堵塞，选择依次发送
*******************************************************************************/
//停止指令能否用一条通用指令？
void Stop()
{
    
    unsigned char buffer[4]={0xFE, 0x02, 0x0b, 0x33};
    Uart1_send(buffer,4);//回复接收到指令
    
    moveDir=0x00;
    
    if(lowPower==0x01)//电量过低的时候，move指令不起作用 
    {
      return;
    }
    //判断是否有移动到指令，有的话需要将移动到指令认为失败，并通知下位机和各子控版
    if(movetoXYZ == 1)//
    {
      movetoXYZ_Ok = 2;//向上位机回复移动到位置指令执行失败；向各个子控板发送位置任务完成指令，提醒其清除相关状态
      MoveToXYZ_RES();
      MoveToXYZ_END();
      movetoX_status = 0;
      movetoY_status = 0;
      movetoZ_status = 0;
      movetoXYZ = 0;
      movetoXYZ_Ok = 0;
    }
    
    int num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;
        //先向小车和伸缩杆运动控制板发送停止指令fe 04 04 01 03 CRC
        unsigned char bf_02[6]={0xfe, 0x04, 0x04, 0x01, 0x03, 0x80};
        Uart3_send(bf_02,6);//向小车板发送停止指令
        //delay_ms(1);
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                stopRecfail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//小车和伸缩杆运动控制板回复
            {
                stopRecfail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
            
            break;
        }
    }
    
    delay_ms(5);//在这里延时1ms，在调试时出现中间的部分没有停下来的问题，并不知道是何原因    
  
    num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;
        //再向局放运动控制板发送停止命令fe 04 03 01 03 crc
        unsigned char bf_02[6]={0xfe, 0x04, 0x03, 0x01, 0x03, 0x63};
        Uart3_send(bf_02,6);//向小车板发送停止指令
        
        //delay_ms(1);
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                stopRecfail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//局放运动控制板回复
            {
                stopRecfail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
            
            break;
        }
    }
}
/*******************************************************************************
j.紧急停止移动机器人
紧急停止是使各个电机停止供电，局放传感器、高清、红外停止供电
其中小车电机和伸缩杆电机供电由电源板控制，
局放传感器、高清、红外供电由主控控制，通过继电器控制
*******************************************************************************/
void EmergencyStop()
{
    unsigned char buffer[4]={0xFE, 0x02, 0x0c, 0xa4};
    Uart1_send(buffer,4);//回复接收成功
    
    moveDir=0x00;
    
    if(lowPower==0x01)//电量过低的时候，move指令不起作用 
    {
      return;
    }
    
    //判断是否有移动到指令，有的话需要将移动到指令认为失败，并通知下位机和各子控版
    if(movetoXYZ == 1)//
    {
      movetoXYZ_Ok = 2;//向上位机回复移动到位置指令执行失败；向各个子控板发送位置任务完成指令，提醒其清除相关状态
      MoveToXYZ_RES();
      MoveToXYZ_END();
      movetoX_status = 0;
      movetoY_status = 0;
      movetoZ_status = 0;
      movetoXYZ = 0;
      movetoXYZ_Ok = 0;
    }
    
    int num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;
        //先向小车和伸缩杆运动控制板发送停止指令fe 04 04 01 03 CRC
        unsigned char bf_02[6]={0xfe, 0x04, 0x04, 0x01, 0x03, 0x80};
        Uart3_send(bf_02,6);//向小车板发送停止指令
        //delay_ms(1);
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                stopRecfail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//小车和伸缩杆运动控制板回复
            {
                stopRecfail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
            
            break;
        }
    }
    
    delay_ms(5);//在这里延时1ms，在调试时出现中间的部分没有停下来的问题，并不知道是何原因    
  
    num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;
        //再向局放运动控制板发送停止命令fe 04 02 01 03 crc
        unsigned char bf_02[6]={0xfe, 0x04, 0x02, 0x01, 0x03, 0x63};
        Uart3_send(bf_02,6);//向小车板发送停止指令
        
        //delay_ms(1);
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                stopRecfail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//局放运动控制板回复
            {
                stopRecfail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
            
            break;
        }
    }
}
/*******************************************************************************
k.获取机器人位置坐标请求
主控板接收到该命令后，分别向三块运动板发送位置请求指令
等待回复，再将三个位置信息整合发给上位机
先小车再伸缩再局放
*******************************************************************************/
void GetPositionReq()
{
    unsigned char buffer_1[11];
    buffer_1[0]=0x0b;
    buffer_1[1]=0x0d;
    if(detectStateOverTime_01==1||detectStateOverTime_02==1||detectStateOverTime_03==1)//查询小车板、伸缩板、局放板失败
    {
        buffer_1[2]=0x00;
    }
    else{buffer_1[2]=0x01;}
    for(int i=0;i<4;i++)
    {
        buffer_1[i+3]=positionX[i];
    }
    for(int i=0;i<2;i++)
    {
        buffer_1[i+7]=positionY[i];
    }
    for(int i=0;i<2;i++)
    {
        buffer_1[i+9]=positionZ[i];
    }
    unsigned char crc=CRC(buffer_1,11);
    unsigned char buffer[13];
    buffer[0]=0xFE;
    for(int i=0;i<11;i++)
    {
        buffer[1+i]=buffer_1[i];
    }
    buffer[12]=crc;
  
    Uart1_send(buffer,13);
}
/*******************************************************************************
m.红外云台转动
参数说明：	
uDirection: 移动方向 0：上 1：下，2：左，3:右
uSpeed：移动速度（单位：°/s）
*******************************************************************************/
void InfraPtzRoll ()
{
    unsigned char buffer[4]={0xFE, 0x02, 0x0f, 0xf7};
    Uart1_send(buffer,4);
    
    unsigned char bf_1[7];
    bf_1[0]=0x07;
    bf_1[1]=0x03;
    bf_1[2]=0x01;
    bf_1[3]=0x06;
    bf_1[4]=Uart1ptzRollRes[0];//移动方向
    bf_1[5]=Uart1ptzRollRes[1];
    bf_1[6]=Uart1ptzRollRes[2];
    unsigned char crc=CRC(bf_1,7);
    unsigned char bf[9];
    bf[0]=0xfe;
    for(int i=0;i<7;i++)
    {
        bf[i+1]=bf_1[i];
    }
    bf[8]=crc;
    
    int num=0;
    while(num<3)
    {
        num++;
        infraPtzRollFail=1;
        Uart3_send(bf,9);//向局放板发送机器人移动指令
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                infraPtzRollFail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x02)//小车板回复
            {
                infraPtzRollFail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(infraPtzRollFail==0)
        {
            break;
        }
    }
}
/*******************************************************************************
n.停止红外云台转动
*******************************************************************************/
void InfraPtzStop ()
{
    unsigned char buffer[4]={0xFE, 0x02, 0x10, 0x9A};
    Uart1_send(buffer,4);
    
    int num=0;
    while(num<3)
    {
        num++;
        infraPtzStopFail=1;
        
        unsigned char bf[6]={0xfe, 0x04, 0x03, 0x01, 0x07, 0xa7};
        Uart3_send(bf,6);//向小车板发送停止指令
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                infraPtzStopFail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//小车和伸缩杆运动控制板回复
            {
                infraPtzStopFail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(infraPtzStopFail==0)
        {
            
            break;
        }
    }
}
/*
o.转动红外云台到指定位置请求
参数说明：
dwXAngle：横向角度
dwYAngle：纵向角度
*/
void InfraPtzRollToReq ()
{
    unsigned char buffer[4]={0xFE, 0x02, 0x11, 0xAB};
    Uart1_send(buffer,4);
    
    unsigned char bf_1[8];
    bf_1[0]=0x08;
    bf_1[1]=0x03;
    bf_1[2]=0x01;
    bf_1[3]=0x08;
    bf_1[4]=Uart1ctrlInfraPTZRes[0];//移动角度
    bf_1[5]=Uart1ctrlInfraPTZRes[1];
    bf_1[6]=Uart1ctrlInfraPTZRes[2];
    bf_1[7]=Uart1ctrlInfraPTZRes[3];
    unsigned char crc=CRC(bf_1,8);
    unsigned char bf[10];
    bf[0]=0xfe;
    for(int i=0;i<8;i++)
    {
        bf[i+1]=bf_1[i];
    }
    bf[9]=crc;
    
    int num=0;
    while(num<3)
    {
        num++;
        InfraPtzRollToRecfail=1;
        
        Uart3_send(bf,10);//向局放板发送机器人移动指令
    
        Init_Timer0_A5();//开10ms定时器
        
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                InfraPtzRollToRecfail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//小车板回复
            {
                InfraPtzRollToRecfail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(InfraPtzRollToRecfail==0)
        {
            break;
        }
    }
    
    unsigned char buffer_1[7];
    buffer_1[0]=0x07;
    buffer_1[1]=0x02;
    if(InfraPtzRollToRecfail==0)
      buffer_1[2]=0x01;
    else  buffer_1[2]=0x00;
    
    buffer_1[3]=Uart1ctrlInfraPTZRes[0];
    buffer_1[4]=Uart1ctrlInfraPTZRes[1];
    
    buffer_1[5]=Uart1ctrlInfraPTZRes[2];
    buffer_1[6]=Uart1ctrlInfraPTZRes[3];
    
    unsigned char crc2=CRC(buffer_1,7);
    unsigned char buffer_2[9];
    buffer_2[0]=0xFE;
    for(int i=0;i<7;i++)
    {
        buffer_2[i+1]=buffer_1[i];
    }
    buffer_2[8]=crc2;
    Uart1_send(buffer_2,9);   
    
}
/*******************************************************************************
p.获取红外云台角度请求
*******************************************************************************/
void InfraPtzGetAngleReq ()
{
    //FE 07 12 01 00 FF 00 FF E2
    unsigned char buffer_1[7];
    buffer_1[0]=0x07;
    buffer_1[1]=0x12;
    buffer_1[2]=0x01;
    
    buffer_1[3]=CyuntaiX[0];
    buffer_1[4]=CyuntaiX[1];
    
    buffer_1[5]=CyuntaiY[0];
    buffer_1[6]=CyuntaiY[1];
    
    unsigned char crc=CRC(buffer_1,7);
    unsigned char buffer[9];
    buffer[0]=0xFE;
    for(int i=0;i<7;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[8]=crc;
    Uart1_send(buffer,9);
}
/*******************************************************************************
q.局放(TEV)检测请求
*******************************************************************************/
void DetectTEVReq()
{
    unsigned char buffer_1[4];
    buffer_1[0]=0x04;
    buffer_1[1]=0x13;
    /*
    if(TEV_status==0x01)
    {
      TEV[0]=0x00;
      TEV[1]=0x00;
    }
    buffer_1[2]=TEV[0];
    buffer_1[3]=TEV[1];
    */
    buffer_1[2]=current[0];
    buffer_1[3]=current[1];
    unsigned char crc=CRC(buffer_1,4);
    unsigned char buffer[6];
    buffer[0]=0xFE;
    for(int i=0;i<4;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[5]=crc;
    
    
  //FE 06 11 00 FF 00 FF 8B
    //unsigned char buffer[]={0xFE, 0x06, 0x13, 0x30, 0x32, 0x30, 0x36, 0x2d, 0x21};
    Uart1_send(buffer,6);
}
/*******************************************************************************
r.局放(超声波)检测请求
*******************************************************************************/
void DetectUSReq()
{
    unsigned char buffer_1[4];
    buffer_1[0]=0x04;
    buffer_1[1]=0x14;
    if(US_status==0x01)
    {
      US[0]=0x00;
      US[1]=0x00;
    }
    buffer_1[2]=US[0];
    buffer_1[3]=US[1];
    unsigned char crc=CRC(buffer_1,4);
    unsigned char buffer[6];
    buffer[0]=0xFE;
    for(int i=0;i<4;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[5]=crc;
    
    Uart1_send(buffer,6);
}
/*******************************************************************************
s.O2浓度监测请求
*******************************************************************************/
void DetectO2Req()
{
    unsigned char buffer_1[4];
    buffer_1[0]=0x04;
    buffer_1[1]=0x15;
    if(detectStateFail_02==1)//检测数据采集板失败
    {
      O2[0]=0xFF;
      O2[1]=0xFF;
    }
    if(O2_status==0x01)
    {
      O2[0]=0x00;
      O2[1]=0x00;
    }
    buffer_1[2]=O2[0];
    buffer_1[3]=O2[1];
    unsigned char crc=CRC(buffer_1,4);
    unsigned char buffer[6];
    buffer[0]=0xFE;
    for(int i=0;i<4;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[5]=crc;
    
    Uart1_send(buffer,6);
}
/*******************************************************************************
t.SF6浓度监测请求
*******************************************************************************/
void DetectSF6Req()
{
    unsigned char buffer_1[4];
    buffer_1[0]=0x04;
    buffer_1[1]=0x16;
    if(detectStateFail_02==1)//检测数据采集板失败
    {
      SF6[0]=0x00;
      SF6[1]=0x00;
    }
    buffer_1[2]=SF6[0];
    buffer_1[3]=SF6[1];
    unsigned char crc=CRC(buffer_1,4);
    unsigned char buffer[6];
    buffer[0]=0xFE;
    for(int i=0;i<4;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[5]=crc;
    Uart1_send(buffer,6);
}
/*******************************************************************************
u.环境参数检测请求
环境参数包括环境温湿度和噪音，温湿度传感器在小车电路板上
噪音传感器在局放板子上
先检测温湿度，再检测噪音
*******************************************************************************/
void DetectEnvParamReq()
{
    unsigned char buffer_1[9];
    buffer_1[0]=0x09;
    buffer_1[1]=0x17;
    if(detectStateFail_02==1)//检测数据采集板失败
    {
      volume[0]=0x00;
      volume[1]=0x00;
    }
    if(detectStateFail_04==1)
    {
      temperature[0]=0x00;
      temperature[1]=0x00;
      humidity[0]=0x00;
      humidity[1]=0x00;
    }
    if((detectStateFail_02==1)||(detectStateFail_04==1))
      buffer_1[2]=0x00;
    else
      buffer_1[2]=0x01;//状态位
    buffer_1[3]=temperature[0];
    buffer_1[4]=temperature[1];
    buffer_1[5]=humidity[0];
    buffer_1[6]=humidity[1];
    buffer_1[7]=volume[0];
    buffer_1[8]=volume[1];
    unsigned char crc=CRC(buffer_1,9);
    
    unsigned char buffer[11];
    buffer[0]=0xFE;
    for(int i=0;i<9;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[10]=crc;

    Uart1_send(buffer,11);
}
/*******************************************************************************
v.更新ROM
将接受到的数据写到EEPROM里面
*******************************************************************************/
void UpdateRomReq()
{
}

/*******************************************************************************
按步骤让各个轴依次运动
*******************************************************************************/
void MoveTo_Origin(void)
{
    int num=0;
    if((movetoX_status == 0)&&(movetoY_status == 0)&&(movetoZ_status == 0)&&MoveToRES_Z == 0)
    {
      if(  ( (long)(positionZ[0])*256 + (long)(positionZ[1]) )
       - ( (long)(Uart1movetoRes[6])*256 + (long)(Uart1movetoRes[7]))
         >=0)//判断小车移动方向
        moveDir=0x06;
      else
        moveDir=0x05;
      
      //向局放轴 Z轴  发送运动到当前指定位置的位置指令
      //Uart2_send(buffer,9);//向小车板发送状态请求指令                         //485总线
      unsigned char buffer[8];                                                   //unsigned  char  ??  2016.01.18 10:07
      unsigned char buffer_1[6];
      
      buffer_1[0]=0x06;
      buffer_1[1]=0x03;
      buffer_1[2]=0x01;
      buffer_1[3]=0x04;
      buffer_1[4]=Uart1movetoRes[6];
      buffer_1[5]=Uart1movetoRes[7];
      unsigned char crc=CRC(buffer_1,6);
      
      buffer[0]=0xfe;
      for(int i=0;i<6;i++)
      {
          buffer[i+1]=buffer_1[i];
      }
      buffer[7]=crc;

      num=0;
      while(num<3)
      {
          num++;
          Uart3_send(buffer,8);                                                      //向局放板子发送位置指令
          
          Init_Timer0_A5();//开10ms定时器
          while(1)
          {
              if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//局放轴 Z轴 回复   //肯定是对位置指令的回复，所以不需要核对全部内容
              {
                  MoveToRES_Z=1;//接收成功
                  Uart3InstructFlag = 0;//清标志
                  Uart3InstructSource = 0x00;//清标志
              }
              else
              {
              }
              if(timer0Flag==1)
              {
                  Uart3InstructFlag = 0;//清标志
                  Uart3InstructSource = 0x00;//清标志
                  break;
              }
          }
          if(MoveToRES_Z == 1)
          {
              break;
          }
      }
      if(MoveToRES_Z == 0)                                                //局放轴 Z轴  对主控板的位置指令响应超时，最终认为整个任务失败
      {
        movetoZ_status = 4;
      }             
    }
  
    if((movetoX_status == 0)&&(movetoY_status == 0)&&(movetoZ_status == 1)&&MoveToRES_Y == 0)
    {
      if(  ( (long)(positionY[0])*256 + (long)(positionY[1]) )
       - ( (long)(Uart1movetoRes[4])*256 + (long)(Uart1movetoRes[5]))
         >=0)//判断小车移动方向
        moveDir=0x04;
      else
        moveDir=0x03;
      
      //向伸缩杆 Y轴  发送运动到当前指定位置的位置指令
      //Uart2_send(buffer,9);//向小车板发送状态请求指令                         //485总线 
      unsigned char buffer[8];                                                   //unsigned  char  ??  2016.01.18 10:07
      unsigned char buffer_1[6];
      buffer_1[0]=0x06;
      buffer_1[1]=0x04;
      buffer_1[2]=0x01;
      buffer_1[3]=0x06;
      buffer_1[4]=Uart1movetoRes[4];
      buffer_1[5]=Uart1movetoRes[5];
      unsigned char crc=CRC(buffer_1,6);
      
      buffer[0]=0xfe;
      for(int i=0;i<6;i++)
      {
          buffer[i+1]=buffer_1[i];
      }
      buffer[7]=crc;

      num=0;
      while(num<3)
      {
          num++;
          Uart3_send(buffer,8);                                                      //向伸缩杆板子发送位置指令
      
          Init_Timer0_A5();//开10ms定时器
          while(1)
          {
              if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//伸缩杆 Y轴回复   //肯定是对位置指令的回复，所以不需要核对全部内容
              {
                  MoveToRES_Y=1;//接收成功
                  Uart3InstructFlag = 0;//清标志
                  Uart3InstructSource = 0x00;//清标志
              }
              else
              {
              }
              if(timer0Flag==1)
              {
                  Uart3InstructFlag = 0;//清标志
                  Uart3InstructSource = 0x00;//清标志
                  break;
              }
          }
          if(MoveToRES_Y == 1)
          {
              break;
          }
      }
      
      if(MoveToRES_Y == 0)                                                //伸缩杆 Y轴  对主控板的位置指令响应超时，最终认为整个任务失败
      {
        movetoY_status = 4;
      }            
      
    }
  
    if((movetoX_status == 0)&&(movetoY_status == 1)&&(movetoZ_status == 1)&&MoveToRES_X == 0)
    {
      if(  ( (long)(positionX[0])*256*256*256 + (long)(positionX[1])*256*256 +(long)(positionX[2])*256 + (long)(positionX[3]) )
       - ( (long)(Uart1movetoRes[0])*256*256*256 + (long)(Uart1movetoRes[1])*256*256 + (long)(Uart1movetoRes[2])*256 + (long)(Uart1movetoRes[3]))
         >=0)//判断小车移动方向
        moveDir=0x02;
      else
        moveDir=0x01;
      
      //向小车  X轴  发送运动到当前指定位置的位置指令
      //Uart3_send(buffer,9);//向小车板发送状态请求指令                         //485总线使用的是Uart3   
      unsigned char buffer[10];                                                   //
      unsigned char buffer_1[8];
      buffer_1[0]=0x08;
      buffer_1[1]=0x04;
      buffer_1[2]=0x01;
      buffer_1[3]=0x04;
      buffer_1[4]=Uart1movetoRes[0];
      buffer_1[5]=Uart1movetoRes[1];
      buffer_1[6]=Uart1movetoRes[2];
      buffer_1[7]=Uart1movetoRes[3];
      unsigned char crc=CRC(buffer_1,8);
      
      buffer[0]=0xfe;
      for(int i=0;i<8;i++)
      {
          buffer[i+1]=buffer_1[i];
      }
      buffer[9]=crc;    
      
      num=0;
      while(num<3)
      {
          num++;
          Uart3_send(buffer,10);                                                      //向小车板位置指令
          Init_Timer0_A5();//开10ms定时器
          while(1)
          {
              if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//小车  X轴回复   //肯定是对位置指令的回复，所以不需要核对全部内容
              {
                  MoveToRES_X=1;//接收成功
                  Uart3InstructFlag = 0;//清标志
                  Uart3InstructSource = 0x00;//清标志
              }
              else
              {
              }
              if(timer0Flag==1)
              {
                  Uart3InstructFlag = 0;//清标志
                  Uart3InstructSource = 0x00;//清标志
                  break;
              }
          }
          if(MoveToRES_X == 1)
          {
              break;
          }
      }
      if(MoveToRES_X == 0)                                                //小车  X轴  对主控板的位置指令响应超时，最终认为整个任务失败
      {
        movetoX_status = 4;
      }
    }
  
}


void MoveTo_Disribute(void)
{
  int num=0;
  if((movetoX_status == 0)&&(movetoY_status == 0)&&(movetoZ_status == 0)&&MoveToRES_X == 0)
  {
    if(  ( (long)(positionX[0])*256*256*256 + (long)(positionX[1])*256*256 +(long)(positionX[2])*256 + (long)(positionX[3]) )
       - ( (long)(Uart1movetoRes[0])*256*256*256 + (long)(Uart1movetoRes[1])*256*256 + (long)(Uart1movetoRes[2])*256 + (long)(Uart1movetoRes[3]))
         >=0)//判断小车移动方向
        moveDir=0x02;
      else
        moveDir=0x01;
    
    //向小车  X轴  发送运动到当前指定位置的位置指令
    //Uart3_send(buffer,9);//向小车板发送状态请求指令                         //485总线使用的是Uart3   
    unsigned char buffer[10];                                                   //
    unsigned char buffer_1[8];
    buffer_1[0]=0x08;
    buffer_1[1]=0x04;
    buffer_1[2]=0x01;
    buffer_1[3]=0x04;
    buffer_1[4]=Uart1movetoRes[0];
    buffer_1[5]=Uart1movetoRes[1];
    buffer_1[6]=Uart1movetoRes[2];
    buffer_1[7]=Uart1movetoRes[3];
    unsigned char crc=CRC(buffer_1,8);
    
    buffer[0]=0xfe;
    for(int i=0;i<8;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[9]=crc;    
    
    num=0;
    while(num<3)
    {
        num++;
        Uart3_send(buffer,10);                                                      //向小车板位置指令
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//小车  X轴回复   //肯定是对位置指令的回复，所以不需要核对全部内容
            {
                MoveToRES_X=1;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
            }
            else
            {
            }
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
        }
        if(MoveToRES_X == 1)
        {
            break;
        }
    }
    if(MoveToRES_X == 0)                                                //小车  X轴  对主控板的位置指令响应超时，最终认为整个任务失败
    {
      movetoX_status = 4;
    }
  }
  if((movetoX_status == 1)&&(movetoY_status == 0)&&(movetoZ_status == 0)&&MoveToRES_Y == 0)
  {
    if(  ( (long)(positionY[0])*256 + (long)(positionY[1]) )
       - ( (long)(Uart1movetoRes[4])*256 + (long)(Uart1movetoRes[5]))
         >=0)//判断小车移动方向
        moveDir=0x04;
      else
        moveDir=0x03;
      
    //向伸缩杆 Y轴  发送运动到当前指定位置的位置指令
    //Uart2_send(buffer,9);//向小车板发送状态请求指令                         //485总线 
    unsigned char buffer[8];                                                   //unsigned  char  ??  2016.01.18 10:07
    unsigned char buffer_1[6];
    buffer_1[0]=0x06;
    buffer_1[1]=0x04;
    buffer_1[2]=0x01;
    buffer_1[3]=0x06;
    buffer_1[4]=Uart1movetoRes[4];
    buffer_1[5]=Uart1movetoRes[5];
    unsigned char crc=CRC(buffer_1,6);
    
    buffer[0]=0xfe;
    for(int i=0;i<6;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[7]=crc;

    num=0;
    while(num<3)
    {
        num++;
        Uart3_send(buffer,8);                                                      //向伸缩杆板子发送位置指令
    
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//伸缩杆 Y轴回复   //肯定是对位置指令的回复，所以不需要核对全部内容
            {
                MoveToRES_Y=1;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
            }
            else
            {
            }
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
        }
        if(MoveToRES_Y == 1)
        {
            break;
        }
    }
    
    if(MoveToRES_Y == 0)                                                //伸缩杆 Y轴  对主控板的位置指令响应超时，最终认为整个任务失败
    {
      movetoY_status = 4;
    }            
  }  
  if((movetoX_status == 1)&&(movetoY_status == 1)&&(movetoZ_status == 0)&&MoveToRES_Z == 0)
  {
    if(  ( (long)(positionZ[0])*256 + (long)(positionZ[1]) )
       - ( (long)(Uart1movetoRes[6])*256 + (long)(Uart1movetoRes[7]))
         >=0)//判断小车移动方向
        moveDir=0x06;
      else
        moveDir=0x05;
      
    //向局放轴 Z轴  发送运动到当前指定位置的位置指令
    //Uart2_send(buffer,9);//向小车板发送状态请求指令                         //485总线
    unsigned char buffer[8];                                                   //unsigned  char  ??  2016.01.18 10:07
    unsigned char buffer_1[6];
    
    buffer_1[0]=0x06;
    buffer_1[1]=0x03;
    buffer_1[2]=0x01;
    buffer_1[3]=0x04;
    buffer_1[4]=Uart1movetoRes[6];
    buffer_1[5]=Uart1movetoRes[7];
    unsigned char crc=CRC(buffer_1,6);
    
    buffer[0]=0xfe;
    for(int i=0;i<6;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[7]=crc;

    num=0;
    while(num<3)
    {
        num++;
        Uart3_send(buffer,8);                                                      //向局放板子发送位置指令
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//局放轴 Z轴 回复   //肯定是对位置指令的回复，所以不需要核对全部内容
            {
                MoveToRES_Z=1;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
            }
            else
            {
            }
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
        }
        if(MoveToRES_Z == 1)
        {
            break;
        }
    }
    if(MoveToRES_Z == 0)                                                //局放轴 Z轴  对主控板的位置指令响应超时，最终认为整个任务失败
    {
      movetoZ_status = 4;
    }             
  }
}
/*******************************************************************************
位置指令任务结束或任务失败时候  对上位机的回复   
*******************************************************************************/
void MoveToXYZ_RES(void)
{
  unsigned char buffer_1[11];
  //指令长度 2字节
  buffer_1[0] = 0x0b;
  //指令编号 1字节
  buffer_1[1] = 0x01;
  //数据 位置指令任务状态1字节+ X轴4字节 +Y轴2字节 +Z轴2字节
  buffer_1[2] = movetoXYZ_Ok;                            //成功                           //目前是全局变量  ，可以改成入口参数

  //x轴
  buffer_1[3] = positionX[0];
  buffer_1[4] = positionX[1];
  buffer_1[5] = positionX[2];
  buffer_1[6] = positionX[3];    
  //y轴
  buffer_1[7] = positionY[0];
  buffer_1[8] = positionY[1];
  //z轴
  buffer_1[9] = positionZ[0];
  buffer_1[10] = positionZ[1];  
              
  unsigned char crc=CRC(buffer_1,11);
  unsigned char buffer[13];
  buffer[0]=0xFE;
  for(int i=0;i<11;i++)
  {
      buffer[1+i]=buffer_1[i];
  }
  buffer[12]=crc;  
  Uart1_send(buffer,13);        //回复上位机
}
/*******************************************************************************
位置指令任务结束或任务失败时候  对下面各个运动板的通知   
*******************************************************************************/
void MoveToXYZ_END(void)
{
    //先向小车板发送停止指令
    int num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;//接收失败fe 04 04 01 05 26
        unsigned char buffer1[6]={0xfe ,0x04 ,0x04 ,0x01 ,0x05 ,0x26};
        Uart3_send(buffer1,6);//向小车板发送位置指令完成  清除变量指令
        //delay_ms(1);
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                stopRecfail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//小车板回复
            {
                stopRecfail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
            
            break;
        }
        
    }
    
    delay_ms(5);//在这里延时1ms，在调试时出现中间的部分没有停下来的问题，并不知道是何原因
    
    
    //再向伸缩板发送停止指令
    //24 24 03 01 30 32 02 D4 21
    num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;//fe 04 04 01 07 44
        unsigned char buffer2[6]={0xfe ,0x04 ,0x04 ,0x01 ,0x07 ,0x44};
        Uart3_send(buffer2,6);//向伸缩杆发送位置指令完成  清除变量指令    
        //delay_ms(1);
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                stopRecfail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//小车板回复
            {
                stopRecfail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
            break;
        }
    }
    
    
    delay_ms(5);
    
  
    //再向局放板发送停止命令
    num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;//fe 04 03 01 05 C5
        unsigned char buffer3[6]={0xfe ,0x04 ,0x03 ,0x01 ,0x05 ,0xC5};
        Uart3_send(buffer3,6);//向局放板发送位置指令完成  清除变量指令 
        //delay_ms(1);
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                stopRecfail=1;//接收失败
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//局放板回复
            {
                stopRecfail=0;//接收成功
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
          break;
        }
    }
    delay_ms(5);
}


/*******************************************************************************
正常的位置指令全部操作，  包含了MoveTo_Disribute();MoveToXYZ_RES();MoveToXYZ_END();
*******************************************************************************/
void MovetoXYZ(void)
{
  if(movetoXYZ == 1)
  {
    //（1）依次向各个板子发送相应的位置指令
    if(goHome==0x00)
    {
        MoveTo_Disribute();
    }
    if(goHome==0x01)
    {
        MoveTo_Origin();
    }
    //（2）判断现在各个运动轴的到达情况
    if((movetoX_status == 1)&&(movetoY_status == 1)&&(movetoZ_status == 1))
    {
      movetoXYZ_Ok = 1;                    //定义为全局变量，在调试似乎可以观看  是否到达，在下一次新派发指令的时候会重新复位为0 
      
      //a.向上位机补充回复位置指令执行成功；b.向各个运动控制板发送，位置任务完成指令，提醒其清除相关状态变量
      //a.向上位机补充回复位置指令执行成功；MoveToXYZ_RES();
      MoveToXYZ_RES();
      //b.
      MoveToXYZ_END();
      moveDir=0x00;//停止
      
      movetoX_status = 0;
      movetoY_status = 0;
      movetoZ_status = 0;
      movetoXYZ = 0;            
      //成功回到目标位置
      
      if((Uart1movetoRes[0]==0)&&(Uart1movetoRes[1]==0)&&(Uart1movetoRes[2]==0)&&(Uart1movetoRes[3]==0)
             &&(Uart1movetoRes[4]==0)&&(Uart1movetoRes[5]==0)&&(Uart1movetoRes[6]==0)&&(Uart1movetoRes[7]==0))
      {
        ChrgStateNow=0x01;//充电状态指示
        //关继电器
        
        
      }
        //chargeState5=0x01;
    }
    else if(((movetoX_status == 3)||(movetoY_status == 3)||(movetoZ_status == 3)) ||      //3  编码器错误失败
              ((movetoX_status == 4)||(movetoY_status == 4)||(movetoZ_status == 4)) ||    //4  3轴指令分发超时失败
                ((movetoX_status == 5)||(movetoY_status == 5))                            //5 障碍物存在导致任务失败
                )      
    {
      if((movetoX_status == 5)||(movetoY_status == 5))
      {
          movetoXYZ_Ok = 5;
      }
      if((movetoX_status == 4)||(movetoY_status == 4)||(movetoZ_status == 4))
      {
        if(movetoX_status == 4)
        {
            movetoXYZ_Ok = 6;
        }
        if(movetoY_status == 4)
        {
            movetoXYZ_Ok = 7;
        }
        if(movetoZ_status == 4)
        {
            movetoXYZ_Ok = 8;        
        }
      }
      else if((movetoX_status == 3)||(movetoY_status == 3)||(movetoZ_status == 3))
      {
        movetoXYZ_Ok = 3;        
      }
//       movetoXYZ_Ok = 0;
      //向上位机补充回复位置指令执行失败；向各个运动控制板发送，位置任务完成指令，提醒其清除相关状态变量 
      //a.向上位机补充回复位置指令执行成功；b.向各个运动控制板发送，位置任务完成指令，提醒其清除相关状态变量
      //a.
       MoveToXYZ_RES();
      //b.
      MoveToXYZ_END();
      moveDir=0x00;//停止

      movetoX_status = 0;
      movetoY_status = 0;
      movetoZ_status = 0;
      movetoXYZ = 0;
    }
  }
}
/*******************************************************************************
接收到上位机指令后进行处理
*******************************************************************************/
void InstructProcess()
{
    if(Uart1InstructFlag==0x01)//有上位机指令
    {
        Uart1InstructFlag=0x00;//清标志
        switch (Uart1InstructNum)
        {
        case 0x04://复位指令
          Reset();
          break;
        case 0x05://获取版本信息
          GetVersionReq();
          break;
        case 0x06://获取充电状态
          GetChrgStateReq();
          break;
        case 0x07://获取总里程
          GetODOReq();
          break;
        case 0x08://获取机器人状态
          GetStatusReq();
          break;
        case 0x09://获取电池电量
          GetBatteryReq();
          break;
        case 0x0a://移动机器人
        {
          Move();
          break;
        }
        case 0x0b://停止移动机器人
          if(lowPower==0x00)//电量过低的时候，stop指令不起作用 
          {
            Stop();
          }
          break;
        case 0x0c://紧急停止机器人
          if(lowPower==0x00)//电量过低的时候，紧急停止指令不起作用
          {
            EmergencyStop();
          }
          break;
        case 0x0d://获取机器人位置坐标
          GetPositionReq();
          break;
        case 0x0e://移动机器人到指定位置
        {
          if(lowPower==0x01)//电量过低的时候，moveto指令不起作用 
          {
            //moveto(0,0,0);
            
            break;
          }
          if((ChrgStateNow==0x01)&&(Electricity<80))//充电状态且电量小于80，moveto指令不起作用 （出航条件）
          {
            break;
          }
          unsigned char buffer[4]={0xfe,0x02,0x0e,0xc6};
          Uart1_send(buffer,4);//立即回复上位机
          
          //判断是不是移动到原点的指令，如果是的话，表示要做充电工作
          
          if(ChrgStateNow==0x01)//如果是充电状态，出航时需告知电源板关闭充电继电器，停止充电。
          {
            if((Uart1movetoRes[0]!=0)||(Uart1movetoRes[1]!=0)||(Uart1movetoRes[2]!=0)||(Uart1movetoRes[3]!=0)
                 ||(Uart1movetoRes[4]!=0)||(Uart1movetoRes[5]!=0)||(Uart1movetoRes[6]!=0)||(Uart1movetoRes[7]!=0))
              {
                ChrgStateNow=0x00;
              }
          }
          //如果是回原点
          if((Uart1movetoRes[0]==0)&&(Uart1movetoRes[1]==0)&&(Uart1movetoRes[2]==0)&&(Uart1movetoRes[3]==0)
             &&(Uart1movetoRes[4]==0)&&(Uart1movetoRes[5]==0)&&(Uart1movetoRes[6]==0)&&(Uart1movetoRes[7]==0))//
          {
            goHome=0x01;//回原点标志位
          }
          else 
          {
            goHome=0x00;//回原点标志位
          }
          MoveToXYZ_END();//结束上一次运动到指令，清状态
          moveDir=0x00;//停止
          
          movetoXYZ = 1;//移动到指令到
          movetoX_status = 0;
          movetoY_status = 0;
          movetoZ_status = 0;
          
          MoveToRES_X=0;
          MoveToRES_Y=0;
          MoveToRES_Z=0;
          movetoXYZ_Ok=0;
          break;
        }
        case 0x0f://红外云台转动请求
          InfraPtzRoll();
          break;
        case 0x10://停止移动红外云台
          InfraPtzStop();
          break;
        case 0x11://移动红外云台到指定位置
          InfraPtzRollToReq ();
          
          break;
        case 0x12://获取红外角度
          InfraPtzGetAngleReq ();
          break;
        case 0x13://局放（TEV）检测请求
          DetectTEVReq();
          break;
        case 0x14://局放（超声波）检测请求
          DetectUSReq();
          break;
        case 0x15://O2浓度检测
          DetectO2Req();
          break;
        case 0x16://SF6浓度检测
          DetectSF6Req();
          break;
        case 0x17://环境参数检测
          DetectEnvParamReq();
          break;
        case 0x18://软件更新，数据接收
        {
        }
        default :
          break;
        }
    }
}
/*******************************************************************************
检测数据采集板状态指令
*******************************************************************************/
void DetectState_02()
{
    if(Uart1InstructFlag == 0)
    {
        unsigned char buffer[6]={0xfe ,0x04 , 0x02 ,0x01 ,0x01 , 0x47 };
        Uart3_send(buffer,6);//向数据采集板发送状态请求指令
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart3InstructFlag == 1)//小车板回复
            {
                if(Uart3InstructSource == 0x02)
                {
                    detectStateFailCount_02=0;
                    detectStateFail_02=0;//接收成功
                    Uart3InstructFlag = 0;//清标志
                    Uart3InstructSource = 0x00;//清标志
                }
            }
            if(timer0Flag==1)
            {
                detectStateFailCount_02++;
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
        }
        if(detectStateFailCount_02>=3)
        {
          detectStateFail_02=1;
        }
    }
}

/*******************************************************************************
检测局放轴运动控制板状态指令
*******************************************************************************/
void DetectState_03()
{
    if(Uart1InstructFlag == 0)
    {
        unsigned char buffer_1[5]={0x05,0x03,0x01,0x01,moveDir};
        unsigned char crc=CRC(buffer_1,5);
        
        unsigned char buffer[7]={0xfe ,0x05 ,0x03 ,0x01 ,0x01 ,moveDir, crc };
        Uart3_send(buffer,7);//向电源板发送状态请求指令
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart3InstructFlag == 1)//伸缩板回复
            {
                if(Uart3InstructSource == 0x03)
                {
                    detectStateFailCount_03=0;
                    detectStateFail_03=0;//接收成功
                    Uart3InstructFlag = 0;//清标志
                    Uart3InstructSource = 0x00;//清标志
                }
                
            }
            if(timer0Flag==1)
            {
                detectStateFailCount_03++;
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
        }
        if(detectStateFailCount_03>=3)
        {
          detectStateFail_03=1;
        }
    }
}

/*******************************************************************************
检测小车和伸缩杆运动控制板状态指令
*******************************************************************************/
void DetectState_04()
{
    if(Uart1InstructFlag == 0)
    {
        obstacleState = isObstacleBelow | (isObstacleBack<<1) | (isObstacleUart3);
        /*
        if((obstacleState&0x01)==0x01)
        {
          P1OUT |= WSI_R;
        }
        else
        {
          P1OUT &= ~WSI_R;
        }
        
        
        if((obstacleState&0x02)==0x02)
        {
          P1OUT |= WSI_Y;
        }
        else
        {
          P1OUT &= ~WSI_Y;
        }
        
        if((isObstacleUart3&0x04)==0x04)
        {
          P1OUT |= WSI_G;
        }
        else
        {
          P1OUT &= ~WSI_G;
        }
        */
        unsigned char buffer_1[5]={0x05,0x04,0x01,0x01,obstacleState};
        unsigned char crc=CRC(buffer_1,5);
        
        unsigned char buffer[7]={0xfe ,0x05 ,0x04 ,0x01 ,0x01 ,obstacleState, crc };
        Uart3_send(buffer,7);//向伸缩板发送状态请求指令

        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart3InstructFlag == 1) 
            {
                if(Uart3InstructSource == 0x04)//局放板回复
                {
                    detectStateFailCount_04=0;
                    detectStateFail_04=0;//接收成功
                    Uart3InstructFlag = 0;//清标志
                    Uart3InstructSource = 0x00;//清标志
                }
            }
            if(timer0Flag==1)
            {
                detectStateFailCount_04++;
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
        }
        if(detectStateFailCount_04>=3)
        {
          detectStateFail_04=1;
        }
    }
}

/*******************************************************************************
检测电源管理板状态指令
*******************************************************************************/
void DetectState_05()
{
    if(Uart1InstructFlag == 0)
    {
        
        unsigned char buffer_1[5]={0x05,0x05,0x01,0x01,ChrgStateNow};
        unsigned char crc=CRC(buffer_1,5);
        
        unsigned char buffer[7]={0xfe ,0x05 ,0x05 ,0x01 ,0x01 ,ChrgStateNow, crc };
        Uart3_send(buffer,7);//向电源板发送状态请求指令

        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart3InstructFlag == 1 )//电源板回复
            {
                if(Uart3InstructSource == 0x05)
                {
                    detectStateFailCount_05=0;
                    detectStateFail_05=0;//接收成功
                    Uart3InstructFlag = 0;//清标志
                    Uart3InstructSource = 0x00;//清标志
                }
            }
            if(timer0Flag==1)
            {
                detectStateFailCount_05++;
                Uart3InstructFlag = 0;//清标志
                Uart3InstructSource = 0x00;//清标志
                break;
            }
        }
        if(detectStateFailCount_05>=3)
        {
          detectStateFail_05=1;
        }
    }
}
/*******************************************************************************
局方数据读取partail discharge
*******************************************************************************/
void DetectPD()
{
    if(Uart1InstructFlag == 0)
    {
        
        //01 03 00 03 00 03 cb f5
        unsigned char buffer[8]={0x01 ,0x03, 0x00, 0x03, 0x00, 0x03, 0xcb, 0xf5};
        Uart2_send(buffer,8);//向局部放电采集板发送寄存器读取指令

        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart2InstructFlag == 1 )//
            {
                TEV_FailCount=0;
                TEV_status=0x00;//tev正常
                US_status=0x00;//超声波正常
                Uart2InstructFlag = 0;//清标志
            }
            if(timer0Flag==1)
            {
                TEV_FailCount++;
                Uart2InstructFlag = 0;//清标志
                break;
            }
        }
        if(TEV_FailCount>=3)
        {
          TEV_status=0x01;
          US_status=0x01;
        }
    }
}

/*******************************************************************************
超声波传感器读取，返回有无
*******************************************************************************/
unsigned char DetectObstacle()
{
    if(Uart1InstructFlag == 0)
    {
        unsigned char a=0x01;
        Uart0_send(&a,1);//串口发送数据请求指令
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart0InstructFlag == 1)//传感器有数据传回来
            {
                if(isObstacle==0x01)//存在障碍物
                {
                    Uart0InstructFlag = 0;
                    isObstacle=0x00;
                    return 0x01;
                }
                else
                {
                    Uart0InstructFlag = 0;
                    isObstacle=0x00;
                    return 0x00;
                }
            }
            if(timer0Flag==1)
            {
                Uart0InstructFlag = 0;
                isObstacle=0x00;
                return 0x00;
            }
        }
    }
    return 0x00;
}
/*******************************************************************************
状态检测函数，轮巡检测
*******************************************************************************/
void DetectState()
{ 
    detectNum++;
    switch(detectNum)
    {
      case 1:
      {
        DetectState_02();//数据采集板状态获取
        
        unsigned int dist=distData[0]*256+distData[1];//障碍物距离
        if((dist>250)&&(dist<400))
          isObstacleBelow=0x01;//有障碍物
        else 
          isObstacleBelow=0x00;//无障碍物
        
        distData[0]=0;
        distData[1]=0;
        
        if(moveDir==0x02)//向后运动时开启
        {
            P8OUT &= ~USC1;//选择第一个超声波传感器 后  左下 E
            P8OUT &= ~USC2;
            P8OUT &= ~USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//串口发送数据请求指令
            
        }
        break;
      }
      case 2:
      {
        DetectState_03();//局放轴运动控制板状态采集
        break;
      }
      case 3:
      {
        DetectState_04();//小车和伸缩杆运动控制板状态采集
        
        unsigned int dist=distData[0]*256+distData[1];//障碍物距离
        if((dist>250)&&(dist<600))
        {
          isObstacleBack=0x01;//有障碍物
          isObstacleBack1=0x01;
        }
        else 
          isObstacleBack1=0x00;//无障碍物
        
        distData[0]=0;
        distData[1]=0;
        if(moveDir==0x02)//向后运动时开启
        {
            P8OUT &= ~USC1;//选择第二个超声波传感器，后半面左上 C
            P8OUT &= ~USC2;
            P8OUT |= USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//串口发送数据请求指令
            
        }
        break;
      }
      case 4:
      {
        DetectState_05();//电源管理板状态采集
        break;
      }
      case 5:
      {
        DetectPD();//检测局放控制板
        
        unsigned int dist=distData[0]*256+distData[1];//障碍物距离
        if((dist>250)&&(dist<600))
        {
          isObstacleBack=0x01;//有障碍物
          isObstacleBack2=0x01;
        }
        else 
          isObstacleBack2=0x00;//无障碍物
        
        distData[0]=0;
        distData[1]=0;
        if(moveDir==0x02)//向后运动时开启
        {
            P8OUT |= USC1;//选择第三个超声波传感器，后半面右上 B
            P8OUT &= ~USC2;
            P8OUT |= USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//串口发送数据请求指令
            
        }
        break;
      }
      case 6:
      {
        DetectState_02();//数据采集板状态获取
        
        break;
      }
      case 7:
      {
        DetectState_03();//局放轴运动控制板状态采集
        
        unsigned int dist=distData[0]*256+distData[1];//障碍物距离
        if((dist>250)&&(dist<600))
        {
          isObstacleBack=0x01;//有障碍物
          isObstacleBack3=0x01;
        }
        else 
          isObstacleBack3=0x00;//无障碍物
        
        distData[0]=0;
        distData[1]=0;
        if(moveDir==0x02)//向后运动时开启
        {
            P8OUT &= ~USC1;//选择第四个超声波传感器，后半面右下 D
            P8OUT |= USC2;
            P8OUT |= USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//串口发送数据请求指令
            
        }
        break;
      }
      case 8:
      {
        DetectState_04();//小车和伸缩杆运动控制板状态采集
        
        break;
      }
      case 9:
      {
        DetectState_05();//电源管理板状态采集
        
        unsigned int dist=distData[0]*256+distData[1];//障碍物距离
        if((dist>250)&&(dist<600))
        {
          isObstacleBack=0x01;//有障碍物
          isObstacleBack4=0x01;
        }
        else 
          isObstacleBack4=0x00;//无障碍物
        
        distData[0]=0;
        distData[1]=0;
        if(moveDir==0x03)//向下运动时开启
        {
            P8OUT |= USC1;//选择第五个超声波传感器，后半面 底 A
            P8OUT |= USC2;
            P8OUT |= USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//串口发送数据请求指令
            
        }
        break;
      }
      case 10:
      {
        DetectPD();//检测局放控制板
        
        break;
      }
      default:
      {
        detectNum=0;
        //判断电量，有没有到最低阈值
        if(detectStateFail_05==0)
        {
          if(Electricity>=50)
          {
              if(ChrgStateNow==0x00)//亮灯
              {
                  P1OUT |= WSI_G;
                  P1OUT &= ~WSI_R;
                  P1OUT &= ~WSI_Y;
              }
              lowPower=0x00;
          }
          else
          {
              if(Electricity>=30)
              {
                  if(ChrgStateNow==0x00)//亮灯
                  {
                      P1OUT |= WSI_Y;
                      P1OUT &= ~WSI_R;
                      P1OUT &= ~WSI_G;
                  }
                  lowPower=0x00;
              }
              else
              {
                  if(lowPower==0x00)
                  {
                      lowPower=0x01;
                      if(ChrgStateNow==0x00)//亮灯
                      {
                          P1OUT |= WSI_R;
                          P1OUT &= ~WSI_G;
                          P1OUT &= ~WSI_Y;
                      }
                      for(char i=0;i<8;i++)//moveto坐标（0，0，0）
                      {
                        Uart1movetoRes[i]=0;
                      }
                      MoveToXYZ_END();//结束上一次运动到指令，清状态
                      moveDir=0x00;//停止
                    
                      movetoXYZ = 1;//移动到指令到
                      movetoX_status = 0;
                      movetoY_status = 0;
                      movetoZ_status = 0;
                      
                      MoveToRES_X=0;
                      MoveToRES_Y=0;
                      MoveToRES_Z=0;
                      movetoXYZ_Ok=0;
                      goHome=0X01;
                  }
              }
          }
        }
        if((isObstacleBack1==0)&&(isObstacleBack2==0)&&(isObstacleBack3==0)&&(isObstacleBack4==0))
        {
          isObstacleBack=0;
        }
        
        if(ChrgStateNow==0x00)
        {
            P1OUT |= RELAY1;//关
        }
        else 
        {
            P1OUT &= ~RELAY1;//开
        }
        
        break;
      }
        
    }
}

/*******************************************************************************
软件更新
*******************************************************************************/
void UpdateProgram()
{
    //unsigned char a[16]={"\r\nStart Update\r\n"};
    //Uart1_send(a,16);
    //UCA0IE &= ~UCRXIE ; // 关闭接收中断
    //UCA1IE &= ~UCRXIE ; // 关闭接收中断
    //UCA2IE &= ~UCRXIE ; // 关闭接收中断
    
    delay_ms(1000);
    /*假设一共需要N帧才能把数据发送完，
    即Uart1UpdateDataCount/249==N-1 && Uart1UpdateDataCount%249 != 0，
    或者Uart1UpdateDataCount/249==N && Uart1UpdateDataCount%249 == 0
    这两种情况，Uart1UpdateDataCount/249不相等，
    先发送前n-1帧，再根据长度发送最后一帧
    */
    unsigned char Num=0x00;
    unsigned char Rem=0x00;
    if(Uart1UpdateDataCount%249 == 0)
    {
      Num=(unsigned char)(Uart1UpdateDataCount/249);
      Rem=(unsigned char)249;
    }
    else
    {
      Num=(unsigned char)(Uart1UpdateDataCount/249)+1;
      Rem=(unsigned char)(Uart1UpdateDataCount%249);
    }
    
    for(unsigned int i=0;i<Num-1;i++)//循环，把前N-1帧发送出去
    {
        unsigned char buffer_1[249]={0x00};
        //_DINT();//关总中断
        eeprom_readpage( 0x0000+i*249 , buffer_1 , 249);
        //_EINT();//关总中断
        unsigned char buffer_1_1[255]={0x00};
        //buffer[0]=0xFE;
        buffer_1_1[0]=0xFF;//指令长度位
        buffer_1_1[1]=Uart1UpdateID;//目的地址
        buffer_1_1[2]=0x01;//源地址
        buffer_1_1[3]=0x18;//指令编号
        buffer_1_1[4]=(unsigned char)i;//数据序号
        buffer_1_1[5]=0x00;//状态位
        for(unsigned char j=0;j<249;j++)
        {
            buffer_1_1[6+j]=buffer_1[j];//数据位
        }
        unsigned char crc=CRC(buffer_1_1,255);
        unsigned char buffer[257];
        buffer[0]=0xFE;//数据头
        for(unsigned int j=0;j<255;j++)
        {
           buffer[j+1]=buffer_1_1[j];
        }
        buffer[256]=crc;
        
        int num=0;
        while(num<5)//重发机制
        {
            Uart3_send(buffer,257);//发送软件更新指令
            
            Init_Timer0_A5();//开10ms定时器
            while(1)
            {
                if(Uart3InstructFlag == 1 )//子控制板回复
                {
                    Uart3InstructFlag=0;//清标志
                    break;
                }
                if(timer0Flag==1)
                {
                    Uart3InstructFlag = 0;//清标志
                    break;
                }
            }
            if(timer0Flag==0)//规定时间内回复
            {
              break;//不用再发第二次了
            }
            else//规定时间内没有回复
            {
              num++;//准备下一次发送
            }
        }
        delay_ms(5);
    }
    //发送第N-1帧数据
    unsigned char buffer_1[249]={0x00};
    _DINT();//关总中断
    eeprom_readpage( 0x0000+(Uart1UpdateDataCount/249)*249 , buffer_1 , Rem);
    _EINT();//关总中断
    unsigned char buffer_1_1[255]={0x00};
    //buffer[0]=0xFE;
    buffer_1_1[0]=(unsigned char)(Rem+6);//指令长度位：包括数据位和一些标志位
    buffer_1_1[1]=Uart1UpdateID;//目的地址
    buffer_1_1[2]=0x01;//源地址
    buffer_1_1[3]=0x18;//指令编号
    buffer_1_1[4]=Num-1;//数据序号
    buffer_1_1[5]=0x01;//状态位
    for(unsigned char j=0;j<Rem;j++)
    {
        buffer_1_1[6+j]=buffer_1[j];//数据位
    }
    unsigned char crc=CRC(buffer_1_1,(unsigned char)(Rem+6));
    unsigned char buffer[257];
    buffer[0]=0xFE;//数据头
    for(unsigned int j=0;j<Rem+6;j++)
    {
       buffer[j+1]=buffer_1_1[j];
    }
    buffer[Rem+7]=crc;
    int num=0;
    while(num<5)//重发机制
    {
        Uart3_send(buffer,Rem+8);//发送软件更新指令
        
        Init_Timer0_A5();//开10ms定时器
        while(1)
        {
            if(Uart3InstructFlag == 1 )//子控制板回复
            {
                Uart3InstructFlag=0;//清标志
                break;
            }
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//清标志
                break;
            }
        }
        if(timer0Flag==0)//规定时间内回复
        {
          break;//不用再发第二次了
        }
        else//规定时间内没有回复
        {
          num++;//准备下一次发送
        }
    }
    
    //发送完成后，等待更新完成指令,需要一个长时间的定时器。
    unsigned int timerCount=0;//定时计数
    while(1)
    {
        Init_Timer0_A5();//50ms定时器
        while(1)
        {
            if(Uart3InstructFlag == 1 )//子控制板回复，先跳出循环
            {
                //Uart3InstructFlag=0;//清标志
                break;
            }
            if(timer0Flag==1)//50ms定时到
            {
                timerCount++;
                break;
                //Uart3InstructFlag = 0;//清标志
            }
        }
        if(Uart3InstructFlag == 1)//子控板回复，可以回复上位机，并跳出循环
        {
            unsigned char a[6]={0xFE , 0x03 , 0x19 , 0x01 , 0x56};
            Uart1_send(a,5);
            Uart3InstructFlag=0;//清标志
            break;
        }
        if(timerCount>=400)//预计时间内没有回复，说明更新失败，跳出循环
        {
            timerCount=0;
            break;
        }
    }
    
    //UCA0IE |= UCRXIE ; // 关闭接收中断
    //UCA1IE |= UCRXIE ; // 关闭接收中断
    //UCA2IE |= UCRXIE ; // 关闭接收中断
    
}