//===========================================================================//
//                                                                           //
// �ļ���  Control.c                                                         //
// ˵����  ���ܺ���������ͨ��Э������Ӧ����                                  //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "Control.h"
#include "SysInit.h"
#include "Uart.h"
#include "eeprom.h"
int timerCount=0;
int detectNum=0x00;
int writeCount=0;

extern unsigned char coordFail;//����У׼����
unsigned char AM_status=0x00;//��ʪ�ȴ�����״̬��0x00��ʾ������0x01��ʾ����
extern unsigned char motorX_status=0x00;//С�����״̬
int motorX_FailCount=0x00;//x������ϴ���
extern unsigned char motorY_status=0x00;//�����˵��״̬
int motorY_FailCount=0x00;//x������ϴ���
extern unsigned char EncoderX_status=0x00;//С��������״̬��0x00��ʾ������0x01��ʾ����
extern unsigned char EncoderY_status=0x00;//�����˱�����״̬��0x00��ʾ������0x01��ʾ����
extern unsigned char MoveToRES_X=0x00;//
extern unsigned char MoveToRES_Y=0x00;//
extern long LlocationX;
extern long LlocationY;
long LtotalmileEncoder=0;
extern unsigned long Ltotalmile=0;
unsigned char x_moving=0;
unsigned char y_moving=0;
extern unsigned char moveDir=0x00;

int humidity=0;
int tempreature=0;
unsigned char Chumidity[2]={0x00};
unsigned char Ctempreature[2]={0x00};
unsigned char encoderXFaultNum=0;//��������ȡ�������
unsigned char encoderYFaultNum=0;//��������ȡ�������

extern unsigned char obstacle_status;//�ϰ������
extern unsigned char Uart1moveRes[4];//moveָ�������Ϣ
extern unsigned char Uart1movetoResX[4];//movetoָ��X�����Ϣ
extern unsigned char Uart1movetoResY[2];//movetoָ��Y�����Ϣ

unsigned char movetoX_status=0x00;
unsigned char movetoX_flag=0x00;//С���ƶ���ָ���־
unsigned char movetoY_status=0x00;
unsigned char movetoY_flag=0x00;//�綯�Ƹ��ƶ���ָ���־
/*******************************************************************************
��ȡ���ݲɼ������ݡ�״̬��Ϣ
*******************************************************************************/
void DetectState()
{
    unsigned char buffer_1[31];
    buffer_1[0]=0x1f;//����λ
    buffer_1[1]=0x01;//Ŀ�ĵ�ַλ
    buffer_1[2]=0x04;//Դ��ַλ
    buffer_1[3]=0x01;//ָ����
    
    unsigned char CweizhiX[4];
    CweizhiX[3]=(unsigned char)(LlocationX);
    CweizhiX[2]=(unsigned char)(LlocationX>>8);
    CweizhiX[1]=(unsigned char)(LlocationX>>16);
    CweizhiX[0]=(unsigned char)(LlocationX>>24);
    
    buffer_1[4]=CweizhiX[0];
    buffer_1[5]=CweizhiX[1];
    buffer_1[6]=CweizhiX[2];
    buffer_1[7]=CweizhiX[3];
    buffer_1[8]=motorX_status;//x���״̬
    buffer_1[9]=EncoderX_status;//X������״̬
    buffer_1[10]=movetoX_status;//xλ��ָ��״̬
    
    unsigned char CweizhiY[4];
    CweizhiY[1]=(unsigned char)(LlocationY);
    CweizhiY[0]=(unsigned char)(LlocationY>>8);
    
    buffer_1[11]=CweizhiY[0];
    buffer_1[12]=CweizhiY[1];
    buffer_1[13]=motorY_status;//y���״̬
    buffer_1[14]=EncoderY_status;//y������״̬
    buffer_1[15]=movetoY_status;//yλ��ָ��״̬
    
    unsigned char CTem[2];
    unsigned char CHum[2];
    
    CTem[1]=(unsigned char)(tempreature);
    CTem[0]=(unsigned char)(tempreature>>8);
    CHum[1]=(unsigned char)(humidity);
    CHum[0]=(unsigned char)(humidity>>8);
    
    buffer_1[16]=CHum[0];
    buffer_1[17]=CHum[1];
    buffer_1[18]=CTem[0];
    buffer_1[19]=CTem[1];
    
    buffer_1[20]=AM_status;//��ʪ�ȴ�����״̬
    
    /*
    �����10B
    */
    unsigned char Ctotalmile[10];
    Ctotalmile[9]=(unsigned char)(Ltotalmile/1%10)+48;
    Ctotalmile[8]=(unsigned char)(Ltotalmile/10%10)+48;
    Ctotalmile[7]=(unsigned char)(Ltotalmile/100%10)+48;
    Ctotalmile[6]=(unsigned char)(Ltotalmile/1000%10)+48;
    Ctotalmile[5]=(unsigned char)(Ltotalmile/10000%10)+48;
    Ctotalmile[4]=(unsigned char)(Ltotalmile/100000%10)+48;
    Ctotalmile[3]=(unsigned char)(Ltotalmile/1000000%10)+48;
    Ctotalmile[2]=(unsigned char)(Ltotalmile/10000000%10)+48;
    Ctotalmile[1]=(unsigned char)(Ltotalmile/100000000%10)+48;
    Ctotalmile[0]=(unsigned char)(Ltotalmile/1000000000%10)+48;
    
    buffer_1[21]=Ctotalmile[0];
    buffer_1[22]=Ctotalmile[1];
    buffer_1[23]=Ctotalmile[2];
    buffer_1[24]=Ctotalmile[3];
    buffer_1[25]=Ctotalmile[4];
    buffer_1[26]=Ctotalmile[5];
    buffer_1[27]=Ctotalmile[6];
    buffer_1[28]=Ctotalmile[7];
    buffer_1[29]=Ctotalmile[8];
    buffer_1[30]=Ctotalmile[9];
    unsigned char crc = CRC(buffer_1,31);
    
    unsigned char buffer[33];
    buffer[0]=0xfe;
    for(int i=0;i<31;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[32]=crc;
    //char buffer[13]={0x24 ,0x24 ,0x01 ,0x04 ,0x30 ,0x32 ,0x04 ,0x72 ,0x21};
    Uart1_send(buffer,33);
}
/*******************************************************************************
�ƶ�ָ��
*******************************************************************************/
void Move()
{
    unsigned char buffer[6]={0xfe,0x04 ,0x01 ,0x04 ,0x02,0xa9};
    Uart1_send(buffer,6);
    
    if(Uart1moveRes[0]==0x00)//�ƶ�С��
    {
        x_moving=0x01;//С���ƶ��С�
        int uSpe=Uart1moveRes[3]+Uart1moveRes[2]*256;
        int delta=150*uSpe;//ת�ٱ�����uSpe�йأ��Ǹ����ȹ�ϵ
        if(delta>1500)
          delta=1500;
        if (Uart1moveRes[1]==0x00)//�������������ǰ��
        {
            moveDir=0x01;
            //if((obstacle_status&BIT2)!=BIT2)
            {
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+delta;
            }
        }
        else if(Uart1moveRes[1]==0x01)//��С����
        {
            moveDir=0x02;
            //if((obstacle_status&BIT1)!=BIT1)
            {
                P1OUT &= ~DCMSTOP1 ;//�������  
                TA0CCR4=4000-delta;
            }
            
        }
        else{}
    }
    else if(Uart1moveRes[0]==0x01)//�ƶ�������
    {
        y_moving=0x01;//�������ƶ��С�
        int uSpe=Uart1moveRes[3]+Uart1moveRes[2]*256;
        int delta=500*uSpe;//ת�ٱ�����uSpe�йأ��Ǹ����ȹ�ϵ
        if(delta>2000)
          delta=2000;
        if (Uart1moveRes[1]==0x01)//�����������С����
        {
            moveDir=0x04;
            //if((obstacle_status&BIT0)!=BIT0)
            {
                P1OUT &= ~DCMSTOP2 ;//�������
                TA0CCR1=4000-delta;
            }
        }
        else if(Uart1moveRes[1]==0x00)//�����½�
        {
            moveDir=0x03;
            P1OUT &= ~DCMSTOP2 ;//�������  
            TA0CCR1=4000+delta;
        }
        else{}
    }
    else
    {
      
    }
}
/*******************************************************************************
ָֹͣ��
*******************************************************************************/
void Stop()
{
    unsigned char buffer[6]={0xfe,0x04 ,0x01 ,0x04 ,0x03,0x98};
    Uart1_send(buffer,6);
    x_moving=0x00;//С�������ƶ��С�
    y_moving=0x00;//�����˲����ƶ��С�
    
    moveDir=0x00;
    P1OUT |= DCMSTOP1 ;//С�����ֹͣ 
    TA0CCR4=4000;
    P1OUT |= DCMSTOP2 ;//���ֹͣ
    TA0CCR1=4000;
}
/*******************************************************************************
λ��ָ��ִ�к������ٶȹ滮�������ж�
*******************************************************************************/
void MoveToX(void)
{
  obstacle_status=0x00;
  //λ��ָ��ִ�У�      ����ָ��λ�õ��ٶȹ滮�������ж�
  if(movetoX_flag == 1)                     
  {

    if((EncoderX_status == 1)||(motorX_status==1))    //3���������������ϵ�������ʧ��   ��
    {
      movetoX_flag = 0;
      movetoX_status = 3;                                                //3�����������ϵ�������ʧ��   ��
      //���ֹͣ
      P1OUT |= DCMSTOP1 ;//С�����ֹͣ 
      TA0CCR4=4000;     
      moveDir=0x00;
    }
    else                                                                        //��������������   ����������λ�ù滮�ٶ�
    {
      //�ֶμӼ���//�����ж�
      long locationXTo=(long)(Uart1movetoResX[0])*256*256*256
                    +(long)(Uart1movetoResX[1])*256*256
                    +(long)(Uart1movetoResX[2])*256
                    +(long)(Uart1movetoResX[3]);
//        int Iweizhi = Ibianmaqi/10;                          //?????
      
      if(locationXTo - LlocationX>0)
      {
          moveDir=0x01;
          if((obstacle_status&0x04)!=0x04)//ǰ�����ϰ�
          {
              if(locationXTo - LlocationX < 2)                     //5Ϊ��������ǰ����������
              {
                                                             //**********ֹͣ����
                P1OUT |= DCMSTOP1 ;//С�����ֹͣ 
                TA0CCR4=4000;     
                moveDir=0x00;
                movetoX_status = 1;                           //λ��ָ��״̬��0,����Ĭ��״̬  ��1������λ��   ��
                                                              //2���������˶�����ָ����   ��3�����������ϵ�������ʧ��   ��
                movetoX_flag = 0;                           //���ٽ���λ���ƶ�ָ���ִ��            
              }
              else if(locationXTo - LlocationX >= 2 && locationXTo - LlocationX <= 20)                     //����
              {
                                                             //�������ٶ��趨ֵ��ռ�ձȣ���ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+100;
              
              }
              else if(locationXTo - LlocationX > 20 && locationXTo - LlocationX <= 200)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+300;
              
              }
              else if(locationXTo - LlocationX > 100 && locationXTo - LlocationX <= 200)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+600;
              
              }
              else if(locationXTo - LlocationX > 200 && locationXTo - LlocationX <= 300)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+900;
              
              }
              else if(locationXTo - LlocationX > 300 && locationXTo - LlocationX <= 400)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+1200;
              
              }
              else if(locationXTo - LlocationX > 400 && locationXTo - LlocationX <= 1000)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+1500;
              
              }
              else if(locationXTo - LlocationX > 1000 && locationXTo - LlocationX <= 1200)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+1800;
              
              }
              else if(locationXTo - LlocationX > 1200 && locationXTo - LlocationX <= 1400)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+2100;
              
              }
              else if(locationXTo - LlocationX > 1400 && locationXTo - LlocationX <= 1600)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+2400;
              
              }
              else if(locationXTo - LlocationX > 1600 && locationXTo - LlocationX <= 1800)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+2700;
              
              }
              else if(locationXTo - LlocationX > 1800 && locationXTo - LlocationX <= 2000)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+3000;
              
              }
              
              else
              {
                                                             //���е�����ٶ��趨ֵ   
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000+3200;
              }
          }
//          UpDownFlag = 1;                                //��ʾ��Ҫ����
          else 
          {
              movetoX_flag = 0;
              movetoX_status = 5;                                                //5���ϰ�����ڵ�������ʧ��   ��
              //���ֹͣ
              P1OUT |= DCMSTOP1 ;//�������
                TA0CCR4=4000;  
                moveDir=0x00;
          }
        
      }
      else if( locationXTo - LlocationX <= 0 )
      {
          moveDir=0x02;
          if(((obstacle_status&0x02)!=0x02)||(LlocationX<1000))//�����ϰ�
          {
                    //          UpDownFlag = 0;                                //0��ʾ��Ҫ���� ��1��ʾ��Ҫ����
              if(locationXTo - LlocationX > -2)                     //5Ϊ��������ǰ����������
              {
                                                             //**********ֹͣ����
                  P1OUT |= DCMSTOP1 ;//�������
                TA0CCR4=4000;
                moveDir=0x00;
                  movetoX_status = 1;                           //����λ��   
                  movetoX_flag = 0;                           //���ٽ���λ���ƶ�ָ���ִ��             
              }
              else if(locationXTo - LlocationX <= -2 && locationXTo - LlocationX >= -20)                     //����
              {
                                                             //�������ٶ��趨ֵ��ռ�ձȣ���ϵ
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-100;
              
              }
              else if(locationXTo - LlocationX < -20 && locationXTo - LlocationX >= -100)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-300;
              
              }
              else if(locationXTo - LlocationX < -100 && locationXTo - LlocationX >= -200)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-600;
              
              }
              else if(locationXTo - LlocationX < -200 && locationXTo - LlocationX >= -300)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-900;
              
              }
              else if(locationXTo - LlocationX < -300 && locationXTo - LlocationX >= -400)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-1200;
              
              }
              else if(locationXTo - LlocationX < -400 && locationXTo - LlocationX >= -1000)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-1500;
              
              }
              else if(locationXTo - LlocationX < -1000 && locationXTo - LlocationX >= -1200)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-1800;
              
              }
              else if(locationXTo - LlocationX < -1200 && locationXTo - LlocationX >= -1400)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-2100;
              
              }
              else if(locationXTo - LlocationX < -1400 && locationXTo - LlocationX >= -1600)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-2400;
              
              }
              else if(locationXTo - LlocationX < -1600 && locationXTo - LlocationX >= -1800)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-2700;
              
              }
              else if(locationXTo - LlocationX < -1800 && locationXTo - LlocationX >= -2000)                     //����
              {
                                                             //�ٶ��趨ֵ��ϵ
                P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-3000;
              
              }
              else
              {
                                                             //���е�����ٶ��趨ֵ 
                  P1OUT &= ~DCMSTOP1 ;//�������
                TA0CCR4=4000-3200;
              }          
          }
          else 
          {
              
              movetoX_flag = 0;
              movetoX_status = 5;                                                //5���ϰ�����ڵ�������ʧ��   ��
              //���ֹͣ
              P1OUT |= DCMSTOP1 ;//�������
              TA0CCR4=4000;
              moveDir=0x00;
          }

      }
      
    }
                                            
  }
}
/*******************************************************************************
λ��ָ��   ���  �������ָ��     С���ĺ���
*******************************************************************************/
void MoveToX_END(void)
{
    movetoX_flag = 0;
    movetoX_status = 0;                        //λ��ָ����� ǿ�ƻָ�
    
    moveDir=0x00;
    
    P1OUT |= DCMSTOP1 ;//�������
    TA0CCR4=4000;
    
    unsigned char buffer[6]={0xfe,0x04 ,0x01 ,0x04 ,0x05,0x3e};
    Uart1_send(buffer,6);
}
/*******************************************************************************
λ��ָ��ִ�к������ٶȹ滮�������ж�
*******************************************************************************/
void MoveToY(void)
{
  //λ��ָ��ִ�У�      ����ָ��λ�õ��ٶȹ滮�������ж�
  if(movetoY_flag == 1)                     
  {
    if((EncoderY_status == 1)||(motorY_status==1)) //3���������������ϵ�������ʧ��   ��
    {
      movetoY_flag = 0;
      movetoY_status = 3;                                                //3�����������ϵ�������ʧ��   ��
      //���ֹͣ
      P1OUT |= DCMSTOP2 ;//�������
      TA0CCR1=4000;
    }
    else                                                                        //��������������   ����������λ�ù滮�ٶ�
    {
      //�ֶμӼ���//�����ж�
      long locationYTo=(long)(Uart1movetoResY[0])*256
                    +(long)(Uart1movetoResY[1]);
//        int Iweizhi = Ibianmaqi/10;                          //?????
      if(locationYTo - LlocationY>0) //�����˶�
      {
        moveDir=0x03;
        if((obstacle_status&0x01)!=0x01)//�·����ϰ���
        {
//          UpDownFlag = 1;                                //��ʾ��Ҫ����
          if(locationYTo - LlocationY <= 1)                     //5Ϊ��������ǰ����������
          {
                                                         //**********ֹͣ����
            P1OUT |= DCMSTOP2 ;//�������
            TA0CCR1=4000;
            movetoY_status = 1;                           //λ��ָ��״̬��0,����Ĭ��״̬  ��1������λ��   ��2���������˶�����ָ����   ��3�����������ϵ�������ʧ��   ��
            movetoY_flag = 0;                           //���ٽ���λ���ƶ�ָ���ִ��            
          }
          else if(locationYTo - LlocationY > 1 && locationYTo - LlocationY <= 50)                     //����
          {
                                                         //�������ٶ��趨ֵ��ռ�ձȣ���ϵ
              P1OUT &= ~DCMSTOP2 ;//�������
              TA0CCR1=4000+100;
          
          }
          else if(locationYTo - LlocationY > 50 && locationYTo - LlocationY <= 200)                     //����
          {
                                                         //�ٶ��趨ֵ��ϵ
              P1OUT &= ~DCMSTOP2 ;//�������
                  TA0CCR1=4000+300;
          
          }
          else
          {
                                                         //���е�����ٶ��趨ֵ   
              P1OUT &= ~DCMSTOP2 ;//�������
                  TA0CCR1=4000+800;
          }
        }
        else
        {
            movetoY_flag = 0;
            movetoY_status = 4;                                                //4���ϰ�����ڵ�������ʧ��   ��
            //���ֹͣ
            P1OUT |= DCMSTOP2 ;//�������
            TA0CCR1=4000;
        }
      }
      else if(locationYTo - LlocationY <= 0)
      {
        moveDir=0x04;
//          UpDownFlag = 0;                                //0��ʾ��Ҫ���� ��1��ʾ��Ҫ����
        if(locationYTo - LlocationY >= -1)                     //5Ϊ��������ǰ����������
        {
                                                       //**********ֹͣ����
            P1OUT |= DCMSTOP2 ;//�������
            TA0CCR1=4000;
            movetoY_status = 1;                           //����λ��            
        }
        else if(locationYTo - LlocationY < -1 && locationYTo - LlocationY >= -50)                     //����
        {
                                                       //�������ٶ��趨ֵ��ռ�ձȣ���ϵ
            P1OUT &= ~DCMSTOP2 ;//�������
                TA0CCR1=4000-100;
        
        }
        else if(locationYTo - LlocationY < -50 && locationYTo - LlocationY >= -200)                     //����
        {
                                                       //�ٶ��趨ֵ��ϵ
          P1OUT &= ~DCMSTOP2 ;//�������
                TA0CCR1=4000-300;
        
        }
        else
        {
                                                       //���е�����ٶ��趨ֵ 
            P1OUT &= ~DCMSTOP2 ;//�������
                TA0CCR1=4000-800;
        }          
      }
      else {}
      
    }
                                            
  }
}
/*******************************************************************************
λ��ָ��   ���  �������ָ��
*******************************************************************************/
void MoveToY_END(void)
{
    movetoY_flag = 0;
    movetoY_status = 0;                        //λ��ָ����� ǿ�ƻָ�
    moveDir=0x00;
    
    P1OUT |= DCMSTOP2 ;//�������
    TA0CCR1=4000;
    
    unsigned char buffer[6]={0xfe,0x04 ,0x01 ,0x04 ,0x07,0x5c};
    Uart1_send(buffer,6);
}

/*******************************************************************************
��Сʱ��Ƭ
*******************************************************************************/
void MinTimeslice()
{
    Init_Timer0_B7();
    while(1)
    {
        if(timer0Flag==1)
        {
            timer0Count++;
            break;
        }
    }
}
/*******************************************************************************
ָ�����
*******************************************************************************/
void InstructProcess()
{
    if(Uart1InstructFlag==0x01)//������ָ��
    {
        Uart1InstructFlag=0x00;//���־
        switch (Uart1InstructNum)
        {
        case 0x01:
          DetectState();
          break;
          
        case 0x02:
          Move();
          break;
        case 0x03:
          Stop();
          break;
        case 0x04:
          {
              movetoX_flag = 1;
              movetoX_status=0;
              unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x04 ,0x04 ,0x0f};
              Uart1_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
              motorX_status=0x00;//����޹���
              break;
          }
        case 0x05:
          MoveToX_END();
          break;
        case 0x06:
          {
              movetoY_flag = 1;
              movetoY_status=0;
              unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x04 ,0x06 ,0x6d};
              Uart1_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
              motorY_status=0x00;//����޹���
              break;
          }
        case 0x07:
          MoveToY_END();
          break;
          
        case 0x08:
        {
          unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x04 ,0x08 ,0x72};
          Uart1_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
          Reset();
          break;
        }
        case 0x18:
        {
            
            
        }
        default :
          break;
          
          
        }
    }
}
/*******************************************************************************
��ȡС������ʽ������ֵ���������ֵ���ȼ����X��λ��
*******************************************************************************/
void DetectEncoder1()
{
    if(Uart1InstructFlag != 0x01)
    {
        ReadEncoder1();//��ȡС��������
        timerCount++;
    }
}
/*******************************************************************************
��ȡ�����˾���ʽ������ֵ���������ֵ���ȼ����Y��λ��
*******************************************************************************/
void DetectEncoder2()
{
    if(Uart1InstructFlag != 0x01)
    {
      ReadEncoder2();//��ȡ�����˱�����
      timerCount++;
    }
}
/*******************************************************************************
��ʪ�Ȳ�����⣬������������ݣ�FC00-FC03������FC04-FC08
*******************************************************************************/
void DetectTAH()
{
    if((Uart1InstructFlag != 0x01) && (timerCount>=100))
    {
      DetectAM();//��ȡ��ʪ��
      MinTimeslice();//��ʱ
      //writeCount++;
      
      if((x_moving==0x01)||(movetoX_flag == 1))//С���ƶ���ʱ���д
      {
          unsigned char Ctotalmile[4]={0x00};
          
          Ctotalmile[3]=(unsigned char)Ltotalmile;
          Ctotalmile[2]=(unsigned char)(Ltotalmile>>8);
          Ctotalmile[1]=(unsigned char)(Ltotalmile>>16);
          Ctotalmile[0]=(unsigned char)(Ltotalmile>>24);
          eeprom_writepage(0xFC00 , Ctotalmile, 4);//д��eeprom����
          delay_ms(5);
          //eeprom_writepage(0xFC04 , Ctotalmile, 4);//д��eeprom����
          //delay_ms(5);
      }
      
      timerCount=0;
    }
}
/*******************************************************************************
��ȡС������ʽ������ֵ���������ֵ���ȼ����X��λ��
*******************************************************************************/

void ReadEncoder1()
{
    P5DIR |= ENCODER_SET ;
    P5OUT &= ~ENCODER_SET ;
    
    long encoderX=LencoderX;
    EncoderX_status=0x01;
    unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
    Uart2_send(bf,4);//�ȷ���ָ��
    Init_Timer0_B7();//��10ms��ʱ��
    while (1)
    {
        if(Uart2InstructFlag == 1)//�������ظ�
        {
            encoderXFaultNum=0;//�������������
            EncoderX_status=0x00;//���ճɹ�
            Uart2InstructFlag = 0;//���־
            LencoderXOld=encoderX;//��¼���α�����������������������
        }
        else
        {
        }
        if(timer0Flag==1)
        {
            encoderXFaultNum++;//���������������һ
            break;
        }
    }
    if(encoderXFaultNum>5)
    {
        encoderXFaultNum=0;//�����������������
        EncoderX_status=0x01;//����ʧ��
    }
    
    long i=LencoderX-LencoderXOld;
    if(i>0)
    {
        if(i<1000)
        {
          LtotalmileEncoder+=i;
        }
        else{}
    }
    if(i<0)
    {
        if(i>-1000)
        {
          LtotalmileEncoder-=i;
        }
        else{}
    }
    if(movetoX_flag==1)//x���˶���
    {
        if(i==0)//������ֵ���䣬��Ϊ���������
        {
            if(motorX_status==0x00)
                motorX_FailCount++;//x������ϴ���
        }
        else
        {
            motorX_FailCount=0;
            motorX_status=0x00;//����޹���
        }
    }
    if(motorX_FailCount>100)//���ϴ���
    {
        motorX_status=0x01;//�������
    }
    
    Ltotalmile+=LtotalmileEncoder/280;
    LtotalmileEncoder=LtotalmileEncoder%280;
  
}
/*******************************************************************************
��ȡ�����˾���ʽ������ֵ���������ֵ���ȼ����Y��λ��
*******************************************************************************/

void ReadEncoder2()
{
    P5DIR |= ENCODER_SET ;
    P5OUT &= ~ENCODER_SET ;
    
    long encoderY=LencoderY;
    EncoderY_status=0x01;
    unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
    Uart3_send(bf,4);//�ȷ���ָ��
    Init_Timer0_B7();//��10ms��ʱ��
    while (1)
    {
        if(Uart3InstructFlag == 1)//�������ظ�
        {
            encoderYFaultNum=0;//�������������
            EncoderY_status=0x00;//���ճɹ�
            Uart3InstructFlag = 0;//���־
            LencoderYOld=encoderY;//��¼���α���������
        }
        else
        {
        }
        if(timer0Flag==1)
        {
            encoderYFaultNum++;//���������������һ
            break;
        }
    }
    if(encoderYFaultNum>5)
    {
        encoderYFaultNum=0;//�����������������
        EncoderY_status=0x01;//����ʧ��
    }
    
    long i=LencoderY-LencoderYOld;
    
    if(movetoY_flag==1)//y���˶���
    {
        if(i==0)//������ֵ���䣬��Ϊ���������
        {
            if(motorY_status==0x00)
                motorY_FailCount++;//x������ϴ���
        }
        else
        {
            motorY_FailCount=0;
            motorY_status=0x00;//����޹���
        }
    }
    if(motorY_FailCount>200)//���ϴ���
    {
        motorY_status=0x01;//�������
        
    }
  
}

/*******************************************************************************
��ʪ�Ȳ����������
*******************************************************************************/
/////////////////////////////////////////////
#define DQ BIT7
#define DQ_DIR  P4DIR
#define DQ_IN   P4IN
#define DQ_OUT  P4OUT
/*-----------------------------------------*/
#define DQ1 DQ_OUT |= DQ
#define DQ0 DQ_OUT &=~DQ
#define DIR_IN DQ_DIR &=~DQ
#define DIR_OUT DQ_DIR |=DQ

unsigned char Read_AM2301(void)
{
    int count=0;
    unsigned char rdata = 0;
    for(int i=0;i<8;i++)
    {
        rdata <<= 1;
        count=0;
        while((DQ_IN & DQ)!=DQ)
        {
            count++;
            if(count>1000)
            {
              break;
            }
        }
        delay_us(33);
        if((DQ_IN & DQ)==DQ)  rdata|=BIT0;
        else 
         rdata &= ~BIT0;
        count=0;
        while((DQ_IN & DQ)==DQ)
        {
            count++;
            if(count>1000)
            {
              break;
            }
        }     
    } 
    return  rdata;
}

void DetectAM()
{
    int count=0;
    //��ʪ�ȴ���������ʱ�����Բ��������
    _DINT() ;//��ֹ�ж�
    unsigned char a,b,c,d,f;
    unsigned int e;
    DIR_OUT;     //io�����
    DQ0;             //io����0  
    delay_us(1000);    //
    DIR_IN;         //io������
    delay_us(20);
    
    count=0;
    while((DQ_IN&DQ)==DQ)
    {
        count++;
        if(count>1000)
        {
          break;
        }
    }
    
    delay_us(60);
    
    count=0;
    while(!(DQ_IN&DQ))
    {
        count++;
        if(count>1000)
        {
          break;
        }
    }
    
    delay_us(60);
    while((DQ_IN&DQ)==DQ)
    {
        count++;
        if(count>1000)
        {
          break;
        }
    }
    
    delay_us(50);
    a= Read_AM2301();
    b= Read_AM2301();
    c= Read_AM2301();
    d= Read_AM2301();
    f= Read_AM2301();
    e=(unsigned int) (a+b+c+d);
    //P3OUT &= ~SI1;
    if(e>255) e=e-255;
    if(e=f)
    {
        AM_status=0;//��ʪ�ȴ�����δ����
        humidity=(a*256+b);
        tempreature=(c*256+d);
        
        Chumidity[0]=a;
        Chumidity[1]=b;
        Ctempreature[0]=c;
        Ctempreature[1]=d;
        
    }
    else //��ȡʧ��
    {
        AM_status=1;//��ʪ�ȴ���������
        humidity=0;
        tempreature=0;
    }
    _EINT();         //�����ж�5
}

/*******************************************************************************
ʱ��Ƭ��ѭ��⴫����ֵ������������ʪ�ȴ�����
*******************************************************************************/
void DetectSensor(void)
{
    detectNum++;
    if(detectNum>=100)//��ʪ�������һ��
    {
      DetectTAH(); 
      detectNum=0;
    }
    switch(detectNum%2)
    {
        case 0:
          DetectEncoder1();
          break;
        case 1:
          DetectEncoder2();
          break;
        default:
          break;
    }
}
/*******************************************************************************
��ʼ��������
*******************************************************************************/
void Reset()
{
  //С����������ʼ��
  {
    P5DIR |= ENCODER_SET ;
    P5OUT  |= ENCODER_SET;
    
    //���ñ���������,44 30 30 43 07 0d
    unsigned char buffer1[6]={0x44,0x30,0x30,0x43,0x07,0x0d};
    Uart2_send(buffer1,6);
    
    delay_ms(200);
    
    
    //���ñ�����goggnzuomoshi,44 30 30 43 07 0d
    unsigned char buffer2[6]={0x44,0x30,0x30,0x4e,0x10,0x0d};
    Uart2_send(buffer2,6);
    
    delay_ms(200);
    
    //���ñ��������ֵ
    unsigned char buffer[16]={0x44,0x30 ,0x30 ,0x50,0x4c,0x30 ,0x30 ,0x31 ,0x30 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
    Uart2_send(buffer,16);
        
    delay_ms(200);
    
    //���ñ�������ʼֵ
    while(1)
    {
        unsigned char buffer[16]={0x44,0x30 ,0x30 ,0x4a ,0x30 ,0x30 ,0x30 ,0x31 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
        Uart2_send(buffer,15);
        delay_ms(100);
        if(Uart2InstructFlag == 1)
        {
            Uart2InstructFlag = 0;
            break;
        }
        
    }
    
    P5OUT  &= ~ENCODER_SET;
    
    while(1)
    {
        unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
        Uart2_send(bf,4);//�ȷ���ָ��
        delay_ms(20);
        if(Uart2InstructFlag == 1)
        {
            Uart2InstructFlag = 0;
            break;
        }
        
    }
    
    long int bmq_1=0;
    if(LencoderX<=1000000)
    {
        bmq_1=2000000-LencoderX;
    }
    else 
    {
        bmq_1=12000000-LencoderX;
    }
    unsigned char bmq_2[10]={0x00};
    bmq_2[9]=(unsigned char)(bmq_1%10+48);
    bmq_2[8]=(unsigned char)(bmq_1/10%10+48);
    bmq_2[7]=(unsigned char)(bmq_1/100%10+48);
    bmq_2[6]=(unsigned char)(bmq_1/1000%10+48);
    bmq_2[5]=(unsigned char)(bmq_1/10000%10+48);
    bmq_2[4]=(unsigned char)(bmq_1/100000%10+48);
    bmq_2[3]=(unsigned char)(bmq_1/1000000%10+48);
    bmq_2[2]=(unsigned char)(bmq_1/10000000%10+48);
    bmq_2[1]=(unsigned char)(bmq_1/100000000%10+48);
    bmq_2[0]=(unsigned char)(bmq_1/1000000000%10+48);
    P5DIR |= ENCODER_SET ;
    P5OUT  |= ENCODER_SET;
    delay_ms(100);
    
    while(1)
    {
        unsigned char buffer1[16]={0x44,0x30 ,0x30 ,0x4a ,bmq_2[0] ,bmq_2[1] ,bmq_2[2] ,bmq_2[3] ,bmq_2[4] ,bmq_2[5],bmq_2[6],bmq_2[7],bmq_2[8],bmq_2[9],0x0d};
        Uart2_send(buffer1,15);
        delay_ms(20);
        if(Uart2InstructFlag == 1)
        {
           Uart2InstructFlag = 0;
            break;
        }
        
    }
    delay_ms(100);
    P5OUT  &= ~ENCODER_SET;
    
    delay_ms(200);
    unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
        Uart2_send(bf,4);//�ȷ���ָ��
        delay_ms(20);
  }
  
  
  //�����˱�������ʼ��
  {
    P5DIR |= ENCODER_SET ;
    P5OUT  |= ENCODER_SET;
    delay_ms(200);
    //���ñ���������,44 30 30 43 07 0d
    unsigned char buffer1[6]={0x44,0x30,0x30,0x43,0x06,0x0d};
    Uart3_send(buffer1,6);
    
    delay_ms(200);
    
    
    //���ñ�����gongzuomoshi,44 30 30 43 07 0d
    unsigned char buffer2[6]={0x44,0x30,0x30,0x4e,0x10,0x0d};
    Uart3_send(buffer2,6);
    
    delay_ms(200);
    
    
    //unsigned char buffer1[6]={0x44,0x30 ,0x30 ,0x4E,0x10,0x0d};
    //Uart3_send(buffer1,6);
    //���ñ��������ֵ
    unsigned char buffer[16]={0x44,0x30 ,0x30 ,0x50,0x4c,0x30 ,0x30 ,0x31 ,0x30 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
    Uart3_send(buffer,16);
        
    delay_ms(200);
    //���ñ�������ʼֵ
    while(1)
    {
        unsigned char buffer[16]={0x44,0x30 ,0x30 ,0x4a ,0x30 ,0x30 ,0x30 ,0x31 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
        Uart3_send(buffer,15);
        delay_ms(100);
        if(Uart3InstructFlag == 1)
        {
            Uart3InstructFlag = 0;
            break;
        }
        
    }
    
    P5OUT  &= ~ENCODER_SET;
    
    while(1)
    {
        unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
        Uart3_send(bf,4);//�ȷ���ָ��
        delay_ms(20);
        if(Uart3InstructFlag == 1)
        {
            Uart3InstructFlag = 0;
            break;
        }
        
    }
    
    long int bmq_1=0;
    if(LencoderY<=1000000)
    {
        bmq_1=2000000-LencoderY;
    }
    else 
    {
        bmq_1=12000000-LencoderY;
    }
    unsigned char bmq_2[10]={0x00};
    bmq_2[9]=(unsigned char)(bmq_1%10+48);
    bmq_2[8]=(unsigned char)(bmq_1/10%10+48);
    bmq_2[7]=(unsigned char)(bmq_1/100%10+48);
    bmq_2[6]=(unsigned char)(bmq_1/1000%10+48);
    bmq_2[5]=(unsigned char)(bmq_1/10000%10+48);
    bmq_2[4]=(unsigned char)(bmq_1/100000%10+48);
    bmq_2[3]=(unsigned char)(bmq_1/1000000%10+48);
    bmq_2[2]=(unsigned char)(bmq_1/10000000%10+48);
    bmq_2[1]=(unsigned char)(bmq_1/100000000%10+48);
    bmq_2[0]=(unsigned char)(bmq_1/1000000000%10+48);
    P5DIR |= ENCODER_SET ;
    P5OUT  |= ENCODER_SET;
    delay_ms(100);
    
    while(1)
    {
        unsigned char buffer1[16]={0x44,0x30 ,0x30 ,0x4a ,bmq_2[0] ,bmq_2[1] ,bmq_2[2] ,bmq_2[3] ,bmq_2[4] ,bmq_2[5],bmq_2[6],bmq_2[7],bmq_2[8],bmq_2[9],0x0d};
        Uart3_send(buffer1,15);
        delay_ms(20);
        if(Uart3InstructFlag == 1)
        {
           Uart3InstructFlag = 0;
            break;
        }
        
    }
    delay_ms(100);
    P5OUT  &= ~ENCODER_SET;
    
    delay_ms(200);
    unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
        Uart3_send(bf,4);//�ȷ���ָ��
        delay_ms(20);
    
  }
  
    /*
    //��ȡ����������
    P5DIR |= BME ;
    P5OUT  |= BME;
    delay_ms(2000);
    unsigned char buffer[]={0x44,0x00 ,0x41 ,0x0d};
    Uart0_send(buffer,4);
    delay_ms(2000);
    */
    /*
    //�������ֵ
    P5DIR |= BME ;
    P5OUT  |= BME;
    delay_ms(2000);
    unsigned char buffer[]={0x44,0x30 ,0x30 ,0x50,0x4c,0x30 ,0x30 ,0x31 ,0x30 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
    Uart0_send(buffer,16);
    delay_ms(2000);
    */
    
    /*
    //���÷���
    P5DIR |= BME ;
    P5OUT  |= BME;
    delay_ms(2000);
    unsigned  char buffer1[]={0x44,0x30 ,0x30 ,0x43 ,0x07 ,0x0d };
    Uart0_send(buffer1,6);
    delay_ms(2000);
    */
}