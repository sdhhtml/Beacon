//Time:2018-5-12
//Function:a new mode
//

#include "common.h"
#include "include.h"
#include "math.h"
extern PD pd_var;
extern sPID SPID;

extern const byte F14x16[];
extern byte F8X16;
extern float Kp_motor;
extern float Proportion_1;
extern float Derivative_1;
uint8 img_buf[60][80]; 

int rtdis,m;
int mid_3,mid_second,mid_last1,mid_now,mid_last2,mid_last3,mid_last4,mid_last5,mid_last6,mid_input;

typedef enum
{
  Mode1 = 1,
  Mode2,
} Mode;

uint8 imgbuff[CAMERA_SIZE]; //定义存储接收图像的数组
uint8 img[CAMERA_W * CAMERA_H];

void send_img(int mode);

void main()
{
    int i=0,j=0,k=0;
    int mid_1=0,mid_2=0,l=0;
    DisableInterrupts; 
    Beacon_Init();
    EnableInterrupts;

    while (1)
    {
  /*      camera_get_img();                                   //摄像头获取图像

       img_extract((uint8 *)img_buf,(uint8 *)imgbuff, CAMERA_W*CAMERA_H/8);        //解压为灰度图像，方便发送到上位机显

       for(i=2;i<45;i++)
        { 
          for(j=0;j<79;j++){
          if(img_buf[i][j]!=img_buf[i][j+1])
          {
           // printf("找到跳变点：%d\r\n",j);
           k++;
           mid_1+=j;

          }             
        }
        }
       mid_2=mid_1/k;
       mid_now=mid_2;
       
      mid_1=0;k=0;

      rtdis=Ultrasonic();
      if((rtdis<405)&&(rtdis!=0))m=1;
      else m=0;
  */    
      LCD_P8x16Str(0,0,"DUOJKP:");//duo
  //    LCD_P8x16Str(15,0,"J");//ji
  //    LCD_P8x16Str(30,0,"K");//K
  //    LCD_P8x16Str(39,0,"P");//P
  //    LCD_P8x16Str(48,0,":");//:
      DisplayFloat((Proportion_1*1000),66,0);
     LCD_P8x16Str(0,2,"DUOJKD:");//K
 //     LCD_P8x16Str(39,18,"D");//D
 //     LCD_P8x16Str(48,18,":");//:
      DisplayFloat((Derivative_1*1000),66,2);
      
      LCD_P8x16Str(0,4,"DJKP:");//DIAN
  //    LCD_P8x16Str(15,36,"J");//JI
  //    LCD_P8x16Str(30,36,"K");//K
  //    LCD_P8x16Str(39,36,"P");//P
  //    LCD_P8x16Str(48,36,":");//:
      DisplayFloat((Kp_motor*1000),66,4);
      systick_delay_ms(1000);
    }
}

void send_img(int mode)
{
     if (mode == Mode1)
     {
         vcan_sendimg(imgbuff, CAMERA_SIZE); //发送到上位机
     }
     else if (mode == Mode2)
     {
         img_extract(img, imgbuff, CAMERA_SIZE); //解压图像
         vcan_sendimg(img, CAMERA_W * CAMERA_H); //发送到上位机
     }
}