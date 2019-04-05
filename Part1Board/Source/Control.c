//===========================================================================//
//                                                                           //
// �ļ���  Control.c                                                         //
// ˵����  ����ģ��                                                          //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "Control.h"
#include "Uart.h"
#include "SysInit.h"
#include "PIN_DEF.h"
#include "eeprom.h"

extern unsigned char Uart0InstructFlag;//���յ�ָ���־
extern unsigned char distData[2];
extern unsigned char isObstacle;//���ϰ���

extern unsigned char timer0Flag;//��ʱʱ�䵽��־
extern unsigned char Uart1InstructFlag;//���յ�ָ���־
extern unsigned char Uart1InstructNum;//ָ����
extern unsigned char Uart1moveRes[4];//��λ���ƶ�ָ��
extern unsigned char Uart1movetoRes[8];//��λ���ƶ���ָ��
extern unsigned char Uart1ptzRollRes[3];//��λ��������̨�ƶ�ָ��
extern unsigned char Uart1ctrlInfraPTZRes[4];//��λ��������̨�ƶ���ָ��

extern unsigned char Uart1UpdateID;//rom�����豸ID
extern unsigned char Uart1UpdateDataNum;//rom�����������
extern unsigned char UpdateDataNum;
extern unsigned char Uart1UpdateStatus;//rom����״̬λ
extern unsigned int addrEeprom;//
extern unsigned int Uart1UpdateDataCount;

extern unsigned char Uart2InstructFlag;//���յ�ָ���־

extern unsigned char Uart3InstructFlag;//���յ�ָ���־
extern unsigned char Uart3InstructSource;//ָ���Դ��ַ
extern unsigned char isObstacleUart3;//�ַſ��ư���ϰ�����Ϣ

unsigned char moveRecfail=0;//move��������ʧ��
unsigned char stopRecfail=0;//stop��������ʧ��
unsigned char emergencyStopfail=0;//����ֹͣ��������ʧ��
unsigned char infraPtzRollFail=0;//ת����̨��������ʧ��
unsigned char infraPtzStopFail=0;//ֹͣ��̨ת����������ʧ��
unsigned char InfraPtzRollToRecfail=0;//�ƶ���̨��λ����ʧ��
unsigned char detectStateFail_02=0;//��ȡС����״̬��Ϣʧ��
unsigned char detectStateFail_03=0;//��ȡ������״̬��Ϣʧ��
unsigned char detectStateFail_04=0;//��ȡ�ַŰ�״̬��Ϣʧ��
unsigned char detectStateFail_05=0;//��ȡ��Դ��״̬��Ϣʧ��
unsigned char detectStateFailCount_02=0;//��ȡС����״̬��Ϣʧ�ܼ���
unsigned char detectStateFailCount_03=0;//��ȡ������״̬��Ϣʧ�ܼ���
unsigned char detectStateFailCount_04=0;//��ȡ�ַŰ�״̬��Ϣʧ�ܼ���
unsigned char detectStateFailCount_05=0;//��ȡ��Դ��״̬��Ϣʧ�ܼ���

unsigned char movetoXYZ=0;//�ƶ���ָ��λ��ָ��
unsigned char movetoXsend=0;//
unsigned char movetoYsend=0;//
unsigned char movetoZsend=0;//
unsigned char movetoX=0;//X���ƶ�����
unsigned char movetoY=0;//y���ƶ�����
unsigned char movetoZ=0;//z���ƶ�����

unsigned char moveDir=0x00;//0x00��ʾֹͣ��0x01��ʾǰ��0x02��ʾ��0x03��ʾ��


int yuntaiX=9300;//��ӦX����̨��90��
int yuntaiY=11500;//��ӦY����̨��90��

unsigned char yuntaiRollUp=0;
unsigned char yuntaiRollDown=0;
unsigned char yuntaiRollLeft=0;
unsigned char yuntaiRollRight=0;


extern unsigned char movetoX_status=0;//X���ƶ�����
extern unsigned char movetoY_status=0;//y���ƶ�����
extern unsigned char movetoZ_status=0;//z���ƶ�����

extern unsigned char motor_status_X=0x00;//С�����״̬
extern unsigned char motor_status_Y=0x00;//�������״̬
extern unsigned char motor_status_Z=0x00;//�ַŵ��״̬
extern unsigned char encoder_status_X = 0x00;//С�����ӱ�����
extern unsigned char encoder_status_Y = 0x00;//�����˰��ӱ�����
extern unsigned char encoder_status_Z = 0x00;//�ַŰ��ӱ�����
extern unsigned char xianwei_status_X=0x00;//С����λ״̬
extern unsigned char xianwei_status_Y=0x00;//������λ״̬
extern unsigned char xianwei_status_Z=0x00;//�ַ���λ״̬
extern unsigned char TEV_status=0x00;//tev����
extern unsigned char US_status=0x00;//����������
extern unsigned char SF6_status=0x00;//SF6����������
extern unsigned char O2_status=0x00;//O2����������
extern unsigned char TaH_status=0x00;//��ʪ�ȴ���������
extern unsigned char Volume_status=0x00;//��������������
extern unsigned char Inf_status=0x00;//�����������
extern unsigned char yunatai_status=0x00;//��̨����
extern unsigned char camare_status=0x00;//����ͷ����
extern unsigned char charge_status=0x00;//������
char TEV_FailCount=0x00;
char US_FailCount=0x00;

unsigned char movetoXYZ_Ok=0;    //ָ��ɹ���ɱ�־

extern unsigned char MoveToRES_X=0;                      //X��λ��ָ��ظ�ָʾ���� 
extern unsigned char MoveToRES_Y=0;                      //Y��λ��ָ��ظ�ָʾ���� 
extern unsigned char MoveToRES_Z=0;                      //Z��λ��ָ��ظ�ָʾ���� 


unsigned char isObstacleBack1=0x00;//�Ҳ��ϰ���
unsigned char isObstacleBack2=0x00;//�Ҳ��ϰ���
unsigned char isObstacleBack3=0x00;//�Ҳ��ϰ���
unsigned char isObstacleBack4=0x00;//�Ҳ��ϰ���
char isObstacleBackCount=0x00;//�Ҳ��ϰ������
unsigned char isObstacleBack=0x00;//�Ҳ��ϰ���
unsigned char isObstacleFront=0x00;//����ϰ���
unsigned char isObstacleBelow=0x00;//�����ϰ���
unsigned char chargeState5=0x00;//���״̬��������Դ���ư壬0��ʾ�����磬1��ʾ��ʼ��磬2��ʾֹͣ���
unsigned char lowPower=0x00;//�͵���״̬
unsigned char goHome=0x00;//��ԭ���־�������յ�moveto(0,0,0)ʱ����λ

unsigned char obstacleState=0x00;//�ϰ���״̬��0000 0000��ʾ���ϰ��0x01��ʾǰ���ϰ��0x02��ʾ�����ϰ���
                                  //0x03��ʾǰ�󷽶����ϰ���
unsigned char detectNum=0;       //�������������Ŀ���ӵļ��
/*******************************************************************************
����λ������������ݣ������ذ�����������ư�õ�����IPORT�ڷ���
*******************************************************************************/
extern unsigned char ChrgStateNow=0x00;//���״̬
extern unsigned char totalMile[32]={0x00};//�������Ϣ
extern long totalMileBMQ=0;
extern long LtotalMile=0;
extern unsigned char Electricity=0x00;//����
extern unsigned char current[2]={0x00};//����
extern unsigned char positionX[4]={0x00};//X������
extern unsigned char positionY[2]={0x00};//Y������
extern unsigned char positionZ[2]={0x00};//Z������
extern unsigned char CyuntaiX[2]={0x00};
extern unsigned char CyuntaiY[2]={0x00};
extern unsigned char TEV[2]={0x00};//TEV����ֵ
extern unsigned char US[2]={0x00};//�ַų�����
extern unsigned char O2[2]={0x00};//����Ũ��
extern unsigned char SF6[2]={0x00};//SF6����Ũ��
extern unsigned char temperature[2]={0x00};//�¶�
extern unsigned char humidity[2]={0x00};//ʪ��
extern unsigned char volume[2]={0x00};//����ֵ

int detectStateOverTime_01=0;//С����״̬��ⳬʱ
int detectStateOverTime_02=0;//������״̬��ⳬʱ
int detectStateOverTime_03=0;//�ַŰ�״̬��ⳬʱ
int detectStateOverTime_04=0;//��Դ��״̬��ⳬʱ 
/*******************************************************************************
���յ���λָ�����
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
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                resetFlag=0x00;
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//С�����������˶����ư�ظ�
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
��ȡ�汾��Ϣ�����ذ���յ�����Ϣ��ֱ�ӻظ���λ��Ӳ���汾��Ϣ��
32���ֽڣ��汾��Ϣ����ֽ�
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
��ȡ�����˳��״̬��Ϣ��
���ذ���յ���ָ��������Դ�巢�����״̬��Ϣ��ѯָ��
��Դ�巴���������״̬
���ذ��ٰ������Ϣ��������λ��
һ���ֽڣ�0X00��ʾδ��磬0X01��ʾ��磬0x02��ʾ��ȡʧ��
*******************************************************************************/
void GetChrgStateReq()
{
    //ChrgState=0X00;
    if(detectStateOverTime_04==1)//��ȡ��Դ������ʧ��
    {
        ChrgStateNow=0X02;
    }
    unsigned char bf_1[3]={0x03, 0x06, ChrgStateNow};
    unsigned char crc=CRC(bf_1,3);
    unsigned char buffer[5]={0xFE, 0x03, 0x06, ChrgStateNow, crc};//���ز�ѯ���
    
    Uart1_send(buffer,5);
}
/*******************************************************************************
e.��ȡ�����������������Ϣ����
���ذ���յ���ָ�����С���巢������ָ��
�ȴ�С����ظ������Ϣ���ٽ�����Ϣ���������λ��
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
    if(detectStateOverTime_01==1)//��ȡС��������ʧ��
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
f.��ȡ������״̬����
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
g.��ȡ��ص�������
���ذ���յ���ָ������Դ�巢�ͻ�ȡ��ص�������
�ȴ���Դ��ظ������Ϣ���ٽ�����Ϣ���������λ��
*******************************************************************************/
void GetBatteryReq()
{ 
    if(detectStateFail_05==1)//��ȡ��Դ������ʧ��
    {
        Electricity=0XFF;
    }
    //Electricity=50;
    unsigned char bf_1[3]={0x03, 0x09, Electricity};
    unsigned char crc=CRC(bf_1,3);
    unsigned char buffer[5]={0xFE, 0x03, 0x09, Electricity, crc};//���ؽ���ʧ��
    Uart1_send(buffer,5);
}
/*******************************************************************************
h.�ƶ�������
���ذ���յ���ָ������ж��ƶ��ᣬ�پ������ǿ��˶��巢������ָ��
Ȼ��ȴ��ð��ӻظ����յ���Ϣ���ٻظ���λ�����յ�
*******************************************************************************/
void Move()
{
    int num=0;
    unsigned char buffer[4]={0xFE, 0x02, 0x0a, 0x02};
    Uart1_send(buffer,4);//�ظ����յ�ָ��
    
    if(lowPower==0x01)//�������͵�ʱ��moveָ������� 
    {
      return;
    }
    if((ChrgStateNow==0x01)&&(Electricity<30))//���״̬�ҵ���С��50��moveָ������� ������������
    {
      return;
    }
    ChrgStateNow=0x00;//���״̬����
    
    //�ж��Ƿ����ƶ���ָ��еĻ���Ҫ���ƶ���ָ����Ϊʧ�ܣ���֪ͨ��λ���͸��ӿذ�
    if(movetoXYZ == 1)//
    {
      movetoXYZ_Ok = 2;//����λ���ظ��ƶ���λ��ָ��ִ��ʧ�ܣ�������ӿذ巢��λ���������ָ�������������״̬
      MoveToXYZ_RES();
      MoveToXYZ_END();
      movetoX_status = 0;
      movetoY_status = 0;
      movetoZ_status = 0;
      movetoXYZ = 0;
      movetoXYZ_Ok = 0;
      
      moveDir=0x00;//ֹͣ
    }
    
    if ((Uart1moveRes[0]==0x00)||(Uart1moveRes[0]==0x01))//�ƶ�С���ᡢ������ָ��
    {
        if((Uart1moveRes[0]==0x00)&&(Uart1moveRes[1]==0x00))//С����ǰ�ƶ�
            moveDir=0x01;
        if((Uart1moveRes[0]==0x00)&&(Uart1moveRes[1]==0x01))//С����ǰ�ƶ�
            moveDir=0x02;
        if((Uart1moveRes[0]==0x01)&&(Uart1moveRes[1]==0x00))//�����ƶ�
            moveDir=0x03;
        if((Uart1moveRes[0]==0x01)&&(Uart1moveRes[1]==0x01))//С�������ƶ�
            moveDir=0x04;
        if((Uart1moveRes[0]==0x02)&&(Uart1moveRes[1]==0x00))//��
            moveDir=0x05;
        if((Uart1moveRes[0]==0x02)&&(Uart1moveRes[1]==0x01))//��
            moveDir=0x06;
        unsigned char bf_1[8];
        bf_1[0]=0x08;
        bf_1[1]=0x04;
        bf_1[2]=0x01;
        bf_1[3]=0x02;
        bf_1[4]=Uart1moveRes[0];//�ƶ���
        bf_1[5]=Uart1moveRes[1];//�ƶ�����
        bf_1[6]=Uart1moveRes[2];//�ƶ��ٶ�
        bf_1[7]=Uart1moveRes[3];//�ƶ��ٶ�
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
            moveRecfail=1;//����ʧ��
            
            Uart3_send(bf,10);//��С���巢�ͻ������ƶ�ָ��
        
            Init_Timer0_A5();//��10ms��ʱ��
            
            while(1)
            {
                if(timer0Flag==1)
                {
                    Uart3InstructFlag = 0;//���־
                    Uart3InstructSource = 0x00;//���־
                    moveRecfail=1;//����ʧ��
                    break;
                }
                if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//С�����������˶����ư�ظ�
                {
                    moveRecfail=0;//���ճɹ�
                    Uart3InstructFlag = 0;//���־
                    Uart3InstructSource = 0x00;//���־
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
    else if (Uart1moveRes[0]==0x02)//�ƶ��ַ���ָ��
    {
        unsigned char bf_1[7];
        bf_1[0]=0x07;
        bf_1[1]=0x03;
        bf_1[2]=0x01;
        bf_1[3]=0x02;
        bf_1[4]=Uart1moveRes[1];//�ƶ�����
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
            
            Uart3_send(bf,9);//��ַŰ巢�ͻ������ƶ�ָ��
        
            Init_Timer0_A5();//��10ms��ʱ��
            
            while(1)
            {
                if(timer0Flag==1)
                {
                    Uart3InstructFlag = 0;//���־
                    Uart3InstructSource = 0x00;//���־
                    moveRecfail=1;//����ʧ��
                    break;
                }
                if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//С����ظ�
                {
                    moveRecfail=0;//���ճɹ�
                    Uart3InstructFlag = 0;//���־
                    Uart3InstructSource = 0x00;//���־
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
    else //ָ�����
    {
      
    }
}
/*******************************************************************************
i.ֹͣ�ƶ�������
���ذ���յ���ָ�������˶��巢��ָֹͣ��
����485���߶�����ѡ�����η���
*******************************************************************************/
//ָֹͣ���ܷ���һ��ͨ��ָ�
void Stop()
{
    
    unsigned char buffer[4]={0xFE, 0x02, 0x0b, 0x33};
    Uart1_send(buffer,4);//�ظ����յ�ָ��
    
    moveDir=0x00;
    
    if(lowPower==0x01)//�������͵�ʱ��moveָ������� 
    {
      return;
    }
    //�ж��Ƿ����ƶ���ָ��еĻ���Ҫ���ƶ���ָ����Ϊʧ�ܣ���֪ͨ��λ���͸��ӿذ�
    if(movetoXYZ == 1)//
    {
      movetoXYZ_Ok = 2;//����λ���ظ��ƶ���λ��ָ��ִ��ʧ�ܣ�������ӿذ巢��λ���������ָ�������������״̬
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
        //����С�����������˶����ư巢��ָֹͣ��fe 04 04 01 03 CRC
        unsigned char bf_02[6]={0xfe, 0x04, 0x04, 0x01, 0x03, 0x80};
        Uart3_send(bf_02,6);//��С���巢��ָֹͣ��
        //delay_ms(1);
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                stopRecfail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//С�����������˶����ư�ظ�
            {
                stopRecfail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
            
            break;
        }
    }
    
    delay_ms(5);//��������ʱ1ms���ڵ���ʱ�����м�Ĳ���û��ͣ���������⣬����֪���Ǻ�ԭ��    
  
    num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;
        //����ַ��˶����ư巢��ֹͣ����fe 04 03 01 03 crc
        unsigned char bf_02[6]={0xfe, 0x04, 0x03, 0x01, 0x03, 0x63};
        Uart3_send(bf_02,6);//��С���巢��ָֹͣ��
        
        //delay_ms(1);
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                stopRecfail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//�ַ��˶����ư�ظ�
            {
                stopRecfail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
j.����ֹͣ�ƶ�������
����ֹͣ��ʹ�������ֹͣ���磬�ַŴ����������塢����ֹͣ����
����С������������˵�������ɵ�Դ����ƣ�
�ַŴ����������塢���⹩�������ؿ��ƣ�ͨ���̵�������
*******************************************************************************/
void EmergencyStop()
{
    unsigned char buffer[4]={0xFE, 0x02, 0x0c, 0xa4};
    Uart1_send(buffer,4);//�ظ����ճɹ�
    
    moveDir=0x00;
    
    if(lowPower==0x01)//�������͵�ʱ��moveָ������� 
    {
      return;
    }
    
    //�ж��Ƿ����ƶ���ָ��еĻ���Ҫ���ƶ���ָ����Ϊʧ�ܣ���֪ͨ��λ���͸��ӿذ�
    if(movetoXYZ == 1)//
    {
      movetoXYZ_Ok = 2;//����λ���ظ��ƶ���λ��ָ��ִ��ʧ�ܣ�������ӿذ巢��λ���������ָ�������������״̬
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
        //����С�����������˶����ư巢��ָֹͣ��fe 04 04 01 03 CRC
        unsigned char bf_02[6]={0xfe, 0x04, 0x04, 0x01, 0x03, 0x80};
        Uart3_send(bf_02,6);//��С���巢��ָֹͣ��
        //delay_ms(1);
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                stopRecfail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//С�����������˶����ư�ظ�
            {
                stopRecfail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
            
            break;
        }
    }
    
    delay_ms(5);//��������ʱ1ms���ڵ���ʱ�����м�Ĳ���û��ͣ���������⣬����֪���Ǻ�ԭ��    
  
    num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;
        //����ַ��˶����ư巢��ֹͣ����fe 04 02 01 03 crc
        unsigned char bf_02[6]={0xfe, 0x04, 0x02, 0x01, 0x03, 0x63};
        Uart3_send(bf_02,6);//��С���巢��ָֹͣ��
        
        //delay_ms(1);
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                stopRecfail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//�ַ��˶����ư�ظ�
            {
                stopRecfail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
k.��ȡ������λ����������
���ذ���յ�������󣬷ֱ��������˶��巢��λ������ָ��
�ȴ��ظ����ٽ�����λ����Ϣ���Ϸ�����λ��
��С���������پַ�
*******************************************************************************/
void GetPositionReq()
{
    unsigned char buffer_1[11];
    buffer_1[0]=0x0b;
    buffer_1[1]=0x0d;
    if(detectStateOverTime_01==1||detectStateOverTime_02==1||detectStateOverTime_03==1)//��ѯС���塢�����塢�ַŰ�ʧ��
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
m.������̨ת��
����˵����	
uDirection: �ƶ����� 0���� 1���£�2����3:��
uSpeed���ƶ��ٶȣ���λ����/s��
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
    bf_1[4]=Uart1ptzRollRes[0];//�ƶ�����
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
        Uart3_send(bf,9);//��ַŰ巢�ͻ������ƶ�ָ��
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                infraPtzRollFail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x02)//С����ظ�
            {
                infraPtzRollFail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
n.ֹͣ������̨ת��
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
        Uart3_send(bf,6);//��С���巢��ָֹͣ��
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                infraPtzStopFail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//С�����������˶����ư�ظ�
            {
                infraPtzStopFail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
o.ת��������̨��ָ��λ������
����˵����
dwXAngle������Ƕ�
dwYAngle������Ƕ�
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
    bf_1[4]=Uart1ctrlInfraPTZRes[0];//�ƶ��Ƕ�
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
        
        Uart3_send(bf,10);//��ַŰ巢�ͻ������ƶ�ָ��
    
        Init_Timer0_A5();//��10ms��ʱ��
        
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                InfraPtzRollToRecfail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//С����ظ�
            {
                InfraPtzRollToRecfail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
p.��ȡ������̨�Ƕ�����
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
q.�ַ�(TEV)�������
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
r.�ַ�(������)�������
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
s.O2Ũ�ȼ������
*******************************************************************************/
void DetectO2Req()
{
    unsigned char buffer_1[4];
    buffer_1[0]=0x04;
    buffer_1[1]=0x15;
    if(detectStateFail_02==1)//������ݲɼ���ʧ��
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
t.SF6Ũ�ȼ������
*******************************************************************************/
void DetectSF6Req()
{
    unsigned char buffer_1[4];
    buffer_1[0]=0x04;
    buffer_1[1]=0x16;
    if(detectStateFail_02==1)//������ݲɼ���ʧ��
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
u.���������������
������������������ʪ�Ⱥ���������ʪ�ȴ�������С����·����
�����������ھַŰ�����
�ȼ����ʪ�ȣ��ټ������
*******************************************************************************/
void DetectEnvParamReq()
{
    unsigned char buffer_1[9];
    buffer_1[0]=0x09;
    buffer_1[1]=0x17;
    if(detectStateFail_02==1)//������ݲɼ���ʧ��
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
      buffer_1[2]=0x01;//״̬λ
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
v.����ROM
�����ܵ�������д��EEPROM����
*******************************************************************************/
void UpdateRomReq()
{
}

/*******************************************************************************
�������ø����������˶�
*******************************************************************************/
void MoveTo_Origin(void)
{
    int num=0;
    if((movetoX_status == 0)&&(movetoY_status == 0)&&(movetoZ_status == 0)&&MoveToRES_Z == 0)
    {
      if(  ( (long)(positionZ[0])*256 + (long)(positionZ[1]) )
       - ( (long)(Uart1movetoRes[6])*256 + (long)(Uart1movetoRes[7]))
         >=0)//�ж�С���ƶ�����
        moveDir=0x06;
      else
        moveDir=0x05;
      
      //��ַ��� Z��  �����˶�����ǰָ��λ�õ�λ��ָ��
      //Uart2_send(buffer,9);//��С���巢��״̬����ָ��                         //485����
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
          Uart3_send(buffer,8);                                                      //��ַŰ��ӷ���λ��ָ��
          
          Init_Timer0_A5();//��10ms��ʱ��
          while(1)
          {
              if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//�ַ��� Z�� �ظ�   //�϶��Ƕ�λ��ָ��Ļظ������Բ���Ҫ�˶�ȫ������
              {
                  MoveToRES_Z=1;//���ճɹ�
                  Uart3InstructFlag = 0;//���־
                  Uart3InstructSource = 0x00;//���־
              }
              else
              {
              }
              if(timer0Flag==1)
              {
                  Uart3InstructFlag = 0;//���־
                  Uart3InstructSource = 0x00;//���־
                  break;
              }
          }
          if(MoveToRES_Z == 1)
          {
              break;
          }
      }
      if(MoveToRES_Z == 0)                                                //�ַ��� Z��  �����ذ��λ��ָ����Ӧ��ʱ��������Ϊ��������ʧ��
      {
        movetoZ_status = 4;
      }             
    }
  
    if((movetoX_status == 0)&&(movetoY_status == 0)&&(movetoZ_status == 1)&&MoveToRES_Y == 0)
    {
      if(  ( (long)(positionY[0])*256 + (long)(positionY[1]) )
       - ( (long)(Uart1movetoRes[4])*256 + (long)(Uart1movetoRes[5]))
         >=0)//�ж�С���ƶ�����
        moveDir=0x04;
      else
        moveDir=0x03;
      
      //�������� Y��  �����˶�����ǰָ��λ�õ�λ��ָ��
      //Uart2_send(buffer,9);//��С���巢��״̬����ָ��                         //485���� 
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
          Uart3_send(buffer,8);                                                      //�������˰��ӷ���λ��ָ��
      
          Init_Timer0_A5();//��10ms��ʱ��
          while(1)
          {
              if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//������ Y��ظ�   //�϶��Ƕ�λ��ָ��Ļظ������Բ���Ҫ�˶�ȫ������
              {
                  MoveToRES_Y=1;//���ճɹ�
                  Uart3InstructFlag = 0;//���־
                  Uart3InstructSource = 0x00;//���־
              }
              else
              {
              }
              if(timer0Flag==1)
              {
                  Uart3InstructFlag = 0;//���־
                  Uart3InstructSource = 0x00;//���־
                  break;
              }
          }
          if(MoveToRES_Y == 1)
          {
              break;
          }
      }
      
      if(MoveToRES_Y == 0)                                                //������ Y��  �����ذ��λ��ָ����Ӧ��ʱ��������Ϊ��������ʧ��
      {
        movetoY_status = 4;
      }            
      
    }
  
    if((movetoX_status == 0)&&(movetoY_status == 1)&&(movetoZ_status == 1)&&MoveToRES_X == 0)
    {
      if(  ( (long)(positionX[0])*256*256*256 + (long)(positionX[1])*256*256 +(long)(positionX[2])*256 + (long)(positionX[3]) )
       - ( (long)(Uart1movetoRes[0])*256*256*256 + (long)(Uart1movetoRes[1])*256*256 + (long)(Uart1movetoRes[2])*256 + (long)(Uart1movetoRes[3]))
         >=0)//�ж�С���ƶ�����
        moveDir=0x02;
      else
        moveDir=0x01;
      
      //��С��  X��  �����˶�����ǰָ��λ�õ�λ��ָ��
      //Uart3_send(buffer,9);//��С���巢��״̬����ָ��                         //485����ʹ�õ���Uart3   
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
          Uart3_send(buffer,10);                                                      //��С����λ��ָ��
          Init_Timer0_A5();//��10ms��ʱ��
          while(1)
          {
              if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//С��  X��ظ�   //�϶��Ƕ�λ��ָ��Ļظ������Բ���Ҫ�˶�ȫ������
              {
                  MoveToRES_X=1;//���ճɹ�
                  Uart3InstructFlag = 0;//���־
                  Uart3InstructSource = 0x00;//���־
              }
              else
              {
              }
              if(timer0Flag==1)
              {
                  Uart3InstructFlag = 0;//���־
                  Uart3InstructSource = 0x00;//���־
                  break;
              }
          }
          if(MoveToRES_X == 1)
          {
              break;
          }
      }
      if(MoveToRES_X == 0)                                                //С��  X��  �����ذ��λ��ָ����Ӧ��ʱ��������Ϊ��������ʧ��
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
         >=0)//�ж�С���ƶ�����
        moveDir=0x02;
      else
        moveDir=0x01;
    
    //��С��  X��  �����˶�����ǰָ��λ�õ�λ��ָ��
    //Uart3_send(buffer,9);//��С���巢��״̬����ָ��                         //485����ʹ�õ���Uart3   
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
        Uart3_send(buffer,10);                                                      //��С����λ��ָ��
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//С��  X��ظ�   //�϶��Ƕ�λ��ָ��Ļظ������Բ���Ҫ�˶�ȫ������
            {
                MoveToRES_X=1;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
            }
            else
            {
            }
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                break;
            }
        }
        if(MoveToRES_X == 1)
        {
            break;
        }
    }
    if(MoveToRES_X == 0)                                                //С��  X��  �����ذ��λ��ָ����Ӧ��ʱ��������Ϊ��������ʧ��
    {
      movetoX_status = 4;
    }
  }
  if((movetoX_status == 1)&&(movetoY_status == 0)&&(movetoZ_status == 0)&&MoveToRES_Y == 0)
  {
    if(  ( (long)(positionY[0])*256 + (long)(positionY[1]) )
       - ( (long)(Uart1movetoRes[4])*256 + (long)(Uart1movetoRes[5]))
         >=0)//�ж�С���ƶ�����
        moveDir=0x04;
      else
        moveDir=0x03;
      
    //�������� Y��  �����˶�����ǰָ��λ�õ�λ��ָ��
    //Uart2_send(buffer,9);//��С���巢��״̬����ָ��                         //485���� 
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
        Uart3_send(buffer,8);                                                      //�������˰��ӷ���λ��ָ��
    
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//������ Y��ظ�   //�϶��Ƕ�λ��ָ��Ļظ������Բ���Ҫ�˶�ȫ������
            {
                MoveToRES_Y=1;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
            }
            else
            {
            }
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                break;
            }
        }
        if(MoveToRES_Y == 1)
        {
            break;
        }
    }
    
    if(MoveToRES_Y == 0)                                                //������ Y��  �����ذ��λ��ָ����Ӧ��ʱ��������Ϊ��������ʧ��
    {
      movetoY_status = 4;
    }            
  }  
  if((movetoX_status == 1)&&(movetoY_status == 1)&&(movetoZ_status == 0)&&MoveToRES_Z == 0)
  {
    if(  ( (long)(positionZ[0])*256 + (long)(positionZ[1]) )
       - ( (long)(Uart1movetoRes[6])*256 + (long)(Uart1movetoRes[7]))
         >=0)//�ж�С���ƶ�����
        moveDir=0x06;
      else
        moveDir=0x05;
      
    //��ַ��� Z��  �����˶�����ǰָ��λ�õ�λ��ָ��
    //Uart2_send(buffer,9);//��С���巢��״̬����ָ��                         //485����
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
        Uart3_send(buffer,8);                                                      //��ַŰ��ӷ���λ��ָ��
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//�ַ��� Z�� �ظ�   //�϶��Ƕ�λ��ָ��Ļظ������Բ���Ҫ�˶�ȫ������
            {
                MoveToRES_Z=1;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
            }
            else
            {
            }
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                break;
            }
        }
        if(MoveToRES_Z == 1)
        {
            break;
        }
    }
    if(MoveToRES_Z == 0)                                                //�ַ��� Z��  �����ذ��λ��ָ����Ӧ��ʱ��������Ϊ��������ʧ��
    {
      movetoZ_status = 4;
    }             
  }
}
/*******************************************************************************
λ��ָ���������������ʧ��ʱ��  ����λ���Ļظ�   
*******************************************************************************/
void MoveToXYZ_RES(void)
{
  unsigned char buffer_1[11];
  //ָ��� 2�ֽ�
  buffer_1[0] = 0x0b;
  //ָ���� 1�ֽ�
  buffer_1[1] = 0x01;
  //���� λ��ָ������״̬1�ֽ�+ X��4�ֽ� +Y��2�ֽ� +Z��2�ֽ�
  buffer_1[2] = movetoXYZ_Ok;                            //�ɹ�                           //Ŀǰ��ȫ�ֱ���  �����Ըĳ���ڲ���

  //x��
  buffer_1[3] = positionX[0];
  buffer_1[4] = positionX[1];
  buffer_1[5] = positionX[2];
  buffer_1[6] = positionX[3];    
  //y��
  buffer_1[7] = positionY[0];
  buffer_1[8] = positionY[1];
  //z��
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
  Uart1_send(buffer,13);        //�ظ���λ��
}
/*******************************************************************************
λ��ָ���������������ʧ��ʱ��  ����������˶����֪ͨ   
*******************************************************************************/
void MoveToXYZ_END(void)
{
    //����С���巢��ָֹͣ��
    int num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;//����ʧ��fe 04 04 01 05 26
        unsigned char buffer1[6]={0xfe ,0x04 ,0x04 ,0x01 ,0x05 ,0x26};
        Uart3_send(buffer1,6);//��С���巢��λ��ָ�����  �������ָ��
        //delay_ms(1);
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                stopRecfail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//С����ظ�
            {
                stopRecfail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                break;
            }
            else{}
        }
        if(stopRecfail==0)
        {
            
            break;
        }
        
    }
    
    delay_ms(5);//��������ʱ1ms���ڵ���ʱ�����м�Ĳ���û��ͣ���������⣬����֪���Ǻ�ԭ��
    
    
    //���������巢��ָֹͣ��
    //24 24 03 01 30 32 02 D4 21
    num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;//fe 04 04 01 07 44
        unsigned char buffer2[6]={0xfe ,0x04 ,0x04 ,0x01 ,0x07 ,0x44};
        Uart3_send(buffer2,6);//�������˷���λ��ָ�����  �������ָ��    
        //delay_ms(1);
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                stopRecfail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x04)//С����ظ�
            {
                stopRecfail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
    
  
    //����ַŰ巢��ֹͣ����
    num=0;
    while(num<3)
    {
        num++;
        stopRecfail=1;//fe 04 03 01 05 C5
        unsigned char buffer3[6]={0xfe ,0x04 ,0x03 ,0x01 ,0x05 ,0xC5};
        Uart3_send(buffer3,6);//��ַŰ巢��λ��ָ�����  �������ָ�� 
        //delay_ms(1);
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
                stopRecfail=1;//����ʧ��
                break;
            }
            if(Uart3InstructFlag == 1 && Uart3InstructSource == 0x03)//�ַŰ�ظ�
            {
                stopRecfail=0;//���ճɹ�
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
������λ��ָ��ȫ��������  ������MoveTo_Disribute();MoveToXYZ_RES();MoveToXYZ_END();
*******************************************************************************/
void MovetoXYZ(void)
{
  if(movetoXYZ == 1)
  {
    //��1��������������ӷ�����Ӧ��λ��ָ��
    if(goHome==0x00)
    {
        MoveTo_Disribute();
    }
    if(goHome==0x01)
    {
        MoveTo_Origin();
    }
    //��2���ж����ڸ����˶���ĵ������
    if((movetoX_status == 1)&&(movetoY_status == 1)&&(movetoZ_status == 1))
    {
      movetoXYZ_Ok = 1;                    //����Ϊȫ�ֱ������ڵ����ƺ����Թۿ�  �Ƿ񵽴����һ�����ɷ�ָ���ʱ������¸�λΪ0 
      
      //a.����λ������ظ�λ��ָ��ִ�гɹ���b.������˶����ư巢�ͣ�λ���������ָ�������������״̬����
      //a.����λ������ظ�λ��ָ��ִ�гɹ���MoveToXYZ_RES();
      MoveToXYZ_RES();
      //b.
      MoveToXYZ_END();
      moveDir=0x00;//ֹͣ
      
      movetoX_status = 0;
      movetoY_status = 0;
      movetoZ_status = 0;
      movetoXYZ = 0;            
      //�ɹ��ص�Ŀ��λ��
      
      if((Uart1movetoRes[0]==0)&&(Uart1movetoRes[1]==0)&&(Uart1movetoRes[2]==0)&&(Uart1movetoRes[3]==0)
             &&(Uart1movetoRes[4]==0)&&(Uart1movetoRes[5]==0)&&(Uart1movetoRes[6]==0)&&(Uart1movetoRes[7]==0))
      {
        ChrgStateNow=0x01;//���״ָ̬ʾ
        //�ؼ̵���
        
        
      }
        //chargeState5=0x01;
    }
    else if(((movetoX_status == 3)||(movetoY_status == 3)||(movetoZ_status == 3)) ||      //3  ����������ʧ��
              ((movetoX_status == 4)||(movetoY_status == 4)||(movetoZ_status == 4)) ||    //4  3��ָ��ַ���ʱʧ��
                ((movetoX_status == 5)||(movetoY_status == 5))                            //5 �ϰ�����ڵ�������ʧ��
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
      //����λ������ظ�λ��ָ��ִ��ʧ�ܣ�������˶����ư巢�ͣ�λ���������ָ�������������״̬���� 
      //a.����λ������ظ�λ��ָ��ִ�гɹ���b.������˶����ư巢�ͣ�λ���������ָ�������������״̬����
      //a.
       MoveToXYZ_RES();
      //b.
      MoveToXYZ_END();
      moveDir=0x00;//ֹͣ

      movetoX_status = 0;
      movetoY_status = 0;
      movetoZ_status = 0;
      movetoXYZ = 0;
    }
  }
}
/*******************************************************************************
���յ���λ��ָ�����д���
*******************************************************************************/
void InstructProcess()
{
    if(Uart1InstructFlag==0x01)//����λ��ָ��
    {
        Uart1InstructFlag=0x00;//���־
        switch (Uart1InstructNum)
        {
        case 0x04://��λָ��
          Reset();
          break;
        case 0x05://��ȡ�汾��Ϣ
          GetVersionReq();
          break;
        case 0x06://��ȡ���״̬
          GetChrgStateReq();
          break;
        case 0x07://��ȡ�����
          GetODOReq();
          break;
        case 0x08://��ȡ������״̬
          GetStatusReq();
          break;
        case 0x09://��ȡ��ص���
          GetBatteryReq();
          break;
        case 0x0a://�ƶ�������
        {
          Move();
          break;
        }
        case 0x0b://ֹͣ�ƶ�������
          if(lowPower==0x00)//�������͵�ʱ��stopָ������� 
          {
            Stop();
          }
          break;
        case 0x0c://����ֹͣ������
          if(lowPower==0x00)//�������͵�ʱ�򣬽���ָֹͣ�������
          {
            EmergencyStop();
          }
          break;
        case 0x0d://��ȡ������λ������
          GetPositionReq();
          break;
        case 0x0e://�ƶ������˵�ָ��λ��
        {
          if(lowPower==0x01)//�������͵�ʱ��movetoָ������� 
          {
            //moveto(0,0,0);
            
            break;
          }
          if((ChrgStateNow==0x01)&&(Electricity<80))//���״̬�ҵ���С��80��movetoָ������� ������������
          {
            break;
          }
          unsigned char buffer[4]={0xfe,0x02,0x0e,0xc6};
          Uart1_send(buffer,4);//�����ظ���λ��
          
          //�ж��ǲ����ƶ���ԭ���ָ�����ǵĻ�����ʾҪ����繤��
          
          if(ChrgStateNow==0x01)//����ǳ��״̬������ʱ���֪��Դ��رճ��̵�����ֹͣ��硣
          {
            if((Uart1movetoRes[0]!=0)||(Uart1movetoRes[1]!=0)||(Uart1movetoRes[2]!=0)||(Uart1movetoRes[3]!=0)
                 ||(Uart1movetoRes[4]!=0)||(Uart1movetoRes[5]!=0)||(Uart1movetoRes[6]!=0)||(Uart1movetoRes[7]!=0))
              {
                ChrgStateNow=0x00;
              }
          }
          //����ǻ�ԭ��
          if((Uart1movetoRes[0]==0)&&(Uart1movetoRes[1]==0)&&(Uart1movetoRes[2]==0)&&(Uart1movetoRes[3]==0)
             &&(Uart1movetoRes[4]==0)&&(Uart1movetoRes[5]==0)&&(Uart1movetoRes[6]==0)&&(Uart1movetoRes[7]==0))//
          {
            goHome=0x01;//��ԭ���־λ
          }
          else 
          {
            goHome=0x00;//��ԭ���־λ
          }
          MoveToXYZ_END();//������һ���˶���ָ���״̬
          moveDir=0x00;//ֹͣ
          
          movetoXYZ = 1;//�ƶ���ָ�
          movetoX_status = 0;
          movetoY_status = 0;
          movetoZ_status = 0;
          
          MoveToRES_X=0;
          MoveToRES_Y=0;
          MoveToRES_Z=0;
          movetoXYZ_Ok=0;
          break;
        }
        case 0x0f://������̨ת������
          InfraPtzRoll();
          break;
        case 0x10://ֹͣ�ƶ�������̨
          InfraPtzStop();
          break;
        case 0x11://�ƶ�������̨��ָ��λ��
          InfraPtzRollToReq ();
          
          break;
        case 0x12://��ȡ����Ƕ�
          InfraPtzGetAngleReq ();
          break;
        case 0x13://�ַţ�TEV���������
          DetectTEVReq();
          break;
        case 0x14://�ַţ����������������
          DetectUSReq();
          break;
        case 0x15://O2Ũ�ȼ��
          DetectO2Req();
          break;
        case 0x16://SF6Ũ�ȼ��
          DetectSF6Req();
          break;
        case 0x17://�����������
          DetectEnvParamReq();
          break;
        case 0x18://������£����ݽ���
        {
        }
        default :
          break;
        }
    }
}
/*******************************************************************************
������ݲɼ���״ָ̬��
*******************************************************************************/
void DetectState_02()
{
    if(Uart1InstructFlag == 0)
    {
        unsigned char buffer[6]={0xfe ,0x04 , 0x02 ,0x01 ,0x01 , 0x47 };
        Uart3_send(buffer,6);//�����ݲɼ��巢��״̬����ָ��
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart3InstructFlag == 1)//С����ظ�
            {
                if(Uart3InstructSource == 0x02)
                {
                    detectStateFailCount_02=0;
                    detectStateFail_02=0;//���ճɹ�
                    Uart3InstructFlag = 0;//���־
                    Uart3InstructSource = 0x00;//���־
                }
            }
            if(timer0Flag==1)
            {
                detectStateFailCount_02++;
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
���ַ����˶����ư�״ָ̬��
*******************************************************************************/
void DetectState_03()
{
    if(Uart1InstructFlag == 0)
    {
        unsigned char buffer_1[5]={0x05,0x03,0x01,0x01,moveDir};
        unsigned char crc=CRC(buffer_1,5);
        
        unsigned char buffer[7]={0xfe ,0x05 ,0x03 ,0x01 ,0x01 ,moveDir, crc };
        Uart3_send(buffer,7);//���Դ�巢��״̬����ָ��
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart3InstructFlag == 1)//������ظ�
            {
                if(Uart3InstructSource == 0x03)
                {
                    detectStateFailCount_03=0;
                    detectStateFail_03=0;//���ճɹ�
                    Uart3InstructFlag = 0;//���־
                    Uart3InstructSource = 0x00;//���־
                }
                
            }
            if(timer0Flag==1)
            {
                detectStateFailCount_03++;
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
���С�����������˶����ư�״ָ̬��
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
        Uart3_send(buffer,7);//�������巢��״̬����ָ��

        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart3InstructFlag == 1) 
            {
                if(Uart3InstructSource == 0x04)//�ַŰ�ظ�
                {
                    detectStateFailCount_04=0;
                    detectStateFail_04=0;//���ճɹ�
                    Uart3InstructFlag = 0;//���־
                    Uart3InstructSource = 0x00;//���־
                }
            }
            if(timer0Flag==1)
            {
                detectStateFailCount_04++;
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
����Դ�����״ָ̬��
*******************************************************************************/
void DetectState_05()
{
    if(Uart1InstructFlag == 0)
    {
        
        unsigned char buffer_1[5]={0x05,0x05,0x01,0x01,ChrgStateNow};
        unsigned char crc=CRC(buffer_1,5);
        
        unsigned char buffer[7]={0xfe ,0x05 ,0x05 ,0x01 ,0x01 ,ChrgStateNow, crc };
        Uart3_send(buffer,7);//���Դ�巢��״̬����ָ��

        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart3InstructFlag == 1 )//��Դ��ظ�
            {
                if(Uart3InstructSource == 0x05)
                {
                    detectStateFailCount_05=0;
                    detectStateFail_05=0;//���ճɹ�
                    Uart3InstructFlag = 0;//���־
                    Uart3InstructSource = 0x00;//���־
                }
            }
            if(timer0Flag==1)
            {
                detectStateFailCount_05++;
                Uart3InstructFlag = 0;//���־
                Uart3InstructSource = 0x00;//���־
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
�ַ����ݶ�ȡpartail discharge
*******************************************************************************/
void DetectPD()
{
    if(Uart1InstructFlag == 0)
    {
        
        //01 03 00 03 00 03 cb f5
        unsigned char buffer[8]={0x01 ,0x03, 0x00, 0x03, 0x00, 0x03, 0xcb, 0xf5};
        Uart2_send(buffer,8);//��ֲ��ŵ�ɼ��巢�ͼĴ�����ȡָ��

        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart2InstructFlag == 1 )//
            {
                TEV_FailCount=0;
                TEV_status=0x00;//tev����
                US_status=0x00;//����������
                Uart2InstructFlag = 0;//���־
            }
            if(timer0Flag==1)
            {
                TEV_FailCount++;
                Uart2InstructFlag = 0;//���־
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
��������������ȡ����������
*******************************************************************************/
unsigned char DetectObstacle()
{
    if(Uart1InstructFlag == 0)
    {
        unsigned char a=0x01;
        Uart0_send(&a,1);//���ڷ�����������ָ��
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart0InstructFlag == 1)//�����������ݴ�����
            {
                if(isObstacle==0x01)//�����ϰ���
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
״̬��⺯������Ѳ���
*******************************************************************************/
void DetectState()
{ 
    detectNum++;
    switch(detectNum)
    {
      case 1:
      {
        DetectState_02();//���ݲɼ���״̬��ȡ
        
        unsigned int dist=distData[0]*256+distData[1];//�ϰ������
        if((dist>250)&&(dist<400))
          isObstacleBelow=0x01;//���ϰ���
        else 
          isObstacleBelow=0x00;//���ϰ���
        
        distData[0]=0;
        distData[1]=0;
        
        if(moveDir==0x02)//����˶�ʱ����
        {
            P8OUT &= ~USC1;//ѡ���һ�������������� ��  ���� E
            P8OUT &= ~USC2;
            P8OUT &= ~USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//���ڷ�����������ָ��
            
        }
        break;
      }
      case 2:
      {
        DetectState_03();//�ַ����˶����ư�״̬�ɼ�
        break;
      }
      case 3:
      {
        DetectState_04();//С�����������˶����ư�״̬�ɼ�
        
        unsigned int dist=distData[0]*256+distData[1];//�ϰ������
        if((dist>250)&&(dist<600))
        {
          isObstacleBack=0x01;//���ϰ���
          isObstacleBack1=0x01;
        }
        else 
          isObstacleBack1=0x00;//���ϰ���
        
        distData[0]=0;
        distData[1]=0;
        if(moveDir==0x02)//����˶�ʱ����
        {
            P8OUT &= ~USC1;//ѡ��ڶ�������������������������� C
            P8OUT &= ~USC2;
            P8OUT |= USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//���ڷ�����������ָ��
            
        }
        break;
      }
      case 4:
      {
        DetectState_05();//��Դ�����״̬�ɼ�
        break;
      }
      case 5:
      {
        DetectPD();//���ַſ��ư�
        
        unsigned int dist=distData[0]*256+distData[1];//�ϰ������
        if((dist>250)&&(dist<600))
        {
          isObstacleBack=0x01;//���ϰ���
          isObstacleBack2=0x01;
        }
        else 
          isObstacleBack2=0x00;//���ϰ���
        
        distData[0]=0;
        distData[1]=0;
        if(moveDir==0x02)//����˶�ʱ����
        {
            P8OUT |= USC1;//ѡ������������������������������ B
            P8OUT &= ~USC2;
            P8OUT |= USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//���ڷ�����������ָ��
            
        }
        break;
      }
      case 6:
      {
        DetectState_02();//���ݲɼ���״̬��ȡ
        
        break;
      }
      case 7:
      {
        DetectState_03();//�ַ����˶����ư�״̬�ɼ�
        
        unsigned int dist=distData[0]*256+distData[1];//�ϰ������
        if((dist>250)&&(dist<600))
        {
          isObstacleBack=0x01;//���ϰ���
          isObstacleBack3=0x01;
        }
        else 
          isObstacleBack3=0x00;//���ϰ���
        
        distData[0]=0;
        distData[1]=0;
        if(moveDir==0x02)//����˶�ʱ����
        {
            P8OUT &= ~USC1;//ѡ����ĸ������������������������ D
            P8OUT |= USC2;
            P8OUT |= USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//���ڷ�����������ָ��
            
        }
        break;
      }
      case 8:
      {
        DetectState_04();//С�����������˶����ư�״̬�ɼ�
        
        break;
      }
      case 9:
      {
        DetectState_05();//��Դ�����״̬�ɼ�
        
        unsigned int dist=distData[0]*256+distData[1];//�ϰ������
        if((dist>250)&&(dist<600))
        {
          isObstacleBack=0x01;//���ϰ���
          isObstacleBack4=0x01;
        }
        else 
          isObstacleBack4=0x00;//���ϰ���
        
        distData[0]=0;
        distData[1]=0;
        if(moveDir==0x03)//�����˶�ʱ����
        {
            P8OUT |= USC1;//ѡ������������������������� �� A
            P8OUT |= USC2;
            P8OUT |= USC3;
            
            unsigned char a=0x01;
            Uart0_send(&a,1);//���ڷ�����������ָ��
            
        }
        break;
      }
      case 10:
      {
        DetectPD();//���ַſ��ư�
        
        break;
      }
      default:
      {
        detectNum=0;
        //�жϵ�������û�е������ֵ
        if(detectStateFail_05==0)
        {
          if(Electricity>=50)
          {
              if(ChrgStateNow==0x00)//����
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
                  if(ChrgStateNow==0x00)//����
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
                      if(ChrgStateNow==0x00)//����
                      {
                          P1OUT |= WSI_R;
                          P1OUT &= ~WSI_G;
                          P1OUT &= ~WSI_Y;
                      }
                      for(char i=0;i<8;i++)//moveto���꣨0��0��0��
                      {
                        Uart1movetoRes[i]=0;
                      }
                      MoveToXYZ_END();//������һ���˶���ָ���״̬
                      moveDir=0x00;//ֹͣ
                    
                      movetoXYZ = 1;//�ƶ���ָ�
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
            P1OUT |= RELAY1;//��
        }
        else 
        {
            P1OUT &= ~RELAY1;//��
        }
        
        break;
      }
        
    }
}

/*******************************************************************************
�������
*******************************************************************************/
void UpdateProgram()
{
    //unsigned char a[16]={"\r\nStart Update\r\n"};
    //Uart1_send(a,16);
    //UCA0IE &= ~UCRXIE ; // �رս����ж�
    //UCA1IE &= ~UCRXIE ; // �رս����ж�
    //UCA2IE &= ~UCRXIE ; // �رս����ж�
    
    delay_ms(1000);
    /*����һ����ҪN֡���ܰ����ݷ����꣬
    ��Uart1UpdateDataCount/249==N-1 && Uart1UpdateDataCount%249 != 0��
    ����Uart1UpdateDataCount/249==N && Uart1UpdateDataCount%249 == 0
    �����������Uart1UpdateDataCount/249����ȣ�
    �ȷ���ǰn-1֡���ٸ��ݳ��ȷ������һ֡
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
    
    for(unsigned int i=0;i<Num-1;i++)//ѭ������ǰN-1֡���ͳ�ȥ
    {
        unsigned char buffer_1[249]={0x00};
        //_DINT();//�����ж�
        eeprom_readpage( 0x0000+i*249 , buffer_1 , 249);
        //_EINT();//�����ж�
        unsigned char buffer_1_1[255]={0x00};
        //buffer[0]=0xFE;
        buffer_1_1[0]=0xFF;//ָ���λ
        buffer_1_1[1]=Uart1UpdateID;//Ŀ�ĵ�ַ
        buffer_1_1[2]=0x01;//Դ��ַ
        buffer_1_1[3]=0x18;//ָ����
        buffer_1_1[4]=(unsigned char)i;//�������
        buffer_1_1[5]=0x00;//״̬λ
        for(unsigned char j=0;j<249;j++)
        {
            buffer_1_1[6+j]=buffer_1[j];//����λ
        }
        unsigned char crc=CRC(buffer_1_1,255);
        unsigned char buffer[257];
        buffer[0]=0xFE;//����ͷ
        for(unsigned int j=0;j<255;j++)
        {
           buffer[j+1]=buffer_1_1[j];
        }
        buffer[256]=crc;
        
        int num=0;
        while(num<5)//�ط�����
        {
            Uart3_send(buffer,257);//�����������ָ��
            
            Init_Timer0_A5();//��10ms��ʱ��
            while(1)
            {
                if(Uart3InstructFlag == 1 )//�ӿ��ư�ظ�
                {
                    Uart3InstructFlag=0;//���־
                    break;
                }
                if(timer0Flag==1)
                {
                    Uart3InstructFlag = 0;//���־
                    break;
                }
            }
            if(timer0Flag==0)//�涨ʱ���ڻظ�
            {
              break;//�����ٷ��ڶ�����
            }
            else//�涨ʱ����û�лظ�
            {
              num++;//׼����һ�η���
            }
        }
        delay_ms(5);
    }
    //���͵�N-1֡����
    unsigned char buffer_1[249]={0x00};
    _DINT();//�����ж�
    eeprom_readpage( 0x0000+(Uart1UpdateDataCount/249)*249 , buffer_1 , Rem);
    _EINT();//�����ж�
    unsigned char buffer_1_1[255]={0x00};
    //buffer[0]=0xFE;
    buffer_1_1[0]=(unsigned char)(Rem+6);//ָ���λ����������λ��һЩ��־λ
    buffer_1_1[1]=Uart1UpdateID;//Ŀ�ĵ�ַ
    buffer_1_1[2]=0x01;//Դ��ַ
    buffer_1_1[3]=0x18;//ָ����
    buffer_1_1[4]=Num-1;//�������
    buffer_1_1[5]=0x01;//״̬λ
    for(unsigned char j=0;j<Rem;j++)
    {
        buffer_1_1[6+j]=buffer_1[j];//����λ
    }
    unsigned char crc=CRC(buffer_1_1,(unsigned char)(Rem+6));
    unsigned char buffer[257];
    buffer[0]=0xFE;//����ͷ
    for(unsigned int j=0;j<Rem+6;j++)
    {
       buffer[j+1]=buffer_1_1[j];
    }
    buffer[Rem+7]=crc;
    int num=0;
    while(num<5)//�ط�����
    {
        Uart3_send(buffer,Rem+8);//�����������ָ��
        
        Init_Timer0_A5();//��10ms��ʱ��
        while(1)
        {
            if(Uart3InstructFlag == 1 )//�ӿ��ư�ظ�
            {
                Uart3InstructFlag=0;//���־
                break;
            }
            if(timer0Flag==1)
            {
                Uart3InstructFlag = 0;//���־
                break;
            }
        }
        if(timer0Flag==0)//�涨ʱ���ڻظ�
        {
          break;//�����ٷ��ڶ�����
        }
        else//�涨ʱ����û�лظ�
        {
          num++;//׼����һ�η���
        }
    }
    
    //������ɺ󣬵ȴ��������ָ��,��Ҫһ����ʱ��Ķ�ʱ����
    unsigned int timerCount=0;//��ʱ����
    while(1)
    {
        Init_Timer0_A5();//50ms��ʱ��
        while(1)
        {
            if(Uart3InstructFlag == 1 )//�ӿ��ư�ظ���������ѭ��
            {
                //Uart3InstructFlag=0;//���־
                break;
            }
            if(timer0Flag==1)//50ms��ʱ��
            {
                timerCount++;
                break;
                //Uart3InstructFlag = 0;//���־
            }
        }
        if(Uart3InstructFlag == 1)//�ӿذ�ظ������Իظ���λ����������ѭ��
        {
            unsigned char a[6]={0xFE , 0x03 , 0x19 , 0x01 , 0x56};
            Uart1_send(a,5);
            Uart3InstructFlag=0;//���־
            break;
        }
        if(timerCount>=400)//Ԥ��ʱ����û�лظ���˵������ʧ�ܣ�����ѭ��
        {
            timerCount=0;
            break;
        }
    }
    
    //UCA0IE |= UCRXIE ; // �رս����ж�
    //UCA1IE |= UCRXIE ; // �رս����ж�
    //UCA2IE |= UCRXIE ; // �رս����ж�
    
}