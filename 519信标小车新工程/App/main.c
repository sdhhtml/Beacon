#include "common.h"
#include "include.h"
#include "math.h"

uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组

uint8 img_buf[60][80]; 
uint8 Flag_50ms = 0,Flag_5ms = 0,Flag_10ms=0,Flag_20ms=0,Flag_100ms=0,Flag_200ms=0;
uint16 Flag_500ms=0;
int rtdis,m;
int mid_3,mid_second,mid_last1,mid_now,mid_last2,mid_last3,mid_last4,mid_last5,mid_last6,mid_input;

extern PD pd_var;
extern sPID SPID;
//函数声明
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void PIT1_IRQHandler(void);
void PORTD_IRQHandler(void);        //PORTD端口中断服务函数

void key_handler2(void); 
void key_handler3(void); 
void key_handler4(void);
void  main(void)
{
    int i=0,j=0,k=0;
    int mid_1=0,mid_2=0,l=0;
//    int mid_first[3];
    gpio_init(PTB20,GPO,1);
    gpio_init(PTB21,GPO,1);
    gpio_init(PTB22,GPO,1);
    gpio_init(PTB23,GPO,1);
    port_init(PTD12, ALT1 | IRQ_RISING | PULLUP ); 
    port_init(PTD13, ALT1 | IRQ_RISING | PULLUP ); 
    port_init(PTD14, ALT1 | IRQ_RISING | PULLUP );     
    camera_init(imgbuff);                                  //这里设定  imgbuff 为采集缓冲区！！！！！！
    Motor_init();
    PD_init(&pd_var);      //舵机参数初始化
    Steering_init();
    spid_init(&SPID);            //电机PID参数的初始化
    //配置中断服务函数
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //设置PORTA的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //设置DMA0的中断服务函数为 DMA0_IRQHandler
    
    pit_init_ms(PIT1,1);                                //初始化PIT1，定时时间为： 1ms
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
    enable_irq(PIT1_IRQn); 
    set_vector_handler(PORTD_VECTORn ,PORTD_IRQHandler);    //设置PORTE的中断服务函数为 PORTE_IRQHandler
    enable_irq (PORTD_IRQn);
 while(1)
    {
   //   for(int l=0;l<3;l++){
        camera_get_img();                                   //摄像头获取图像

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
       if(mid_now!=0)mid_input=mid_now;
       mid_last1=mid_now;
       mid_last2=mid_last1;
       mid_last3=mid_last2;
       mid_last4=mid_last3;
       mid_last5=mid_last4;
       mid_last6=mid_last5;
       if((mid_now==0)&&(mid_last1==0)&&(mid_last2==0)&&(mid_last3==0)&&(mid_last4==0)&&(mid_last5==0)&&(mid_last6==0))mid_input=0;
//       printf("中线的横坐标为：%d\r\n",mid_2);
       mid_1=0;k=0;

      rtdis=Ultrasonic();
      if((rtdis<405)&&(rtdis!=0))m=1;
      else m=0;
  
      }
      
      
// }
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}
void PIT1_IRQHandler(void)
{
    Flag_5ms++;
    Flag_10ms++;
    Flag_20ms++;
    Flag_50ms++;
    Flag_100ms++;
    Flag_200ms++;
    Flag_500ms++;

    if(m==0)   {
      
      //    if(mid_second<5){
                            if(Flag_5ms==5)
                            {
                                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2000);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,2000);
                            }
                            if(Flag_10ms==10)
                            {
                              //printf("中线的横坐标为：%d\r\n",mid_3);
                              PD_calculation(&pd_var,mid_input);     
                            }
                            if(Flag_500ms==500)
                            {                              
                            gpio_turn(PTB20);
                            gpio_turn(PTB21);
                            gpio_turn(PTB22);
                            gpio_turn(PTB23);
                            }                    
  //      }       
/*          if(mid_second>=5){
                            if(Flag_5ms==5)
                            {
                                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,1000);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,1500);
                            }
                            if(Flag_10ms==10)
                            {
                              //printf("中线的横坐标为：%d\r\n",mid_3);
                             ftm_pwm_duty(S3010_FTM,S3010_CH,2340); 
                            }
                            if(Flag_500ms==500)
                            {                              
                            gpio_turn(PTB20);
                            gpio_turn(PTB21);
                            gpio_turn(PTB22);
                            gpio_turn(PTB23);
                            }
                    
              }*/
}
    if(m==1){
                          if(Flag_5ms==5)
                    {
                        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,1500);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,1500);
                    }
                    if(Flag_10ms==10)
                    {
                     // ftm_pwm_duty(S3010_FTM, S3010_CH,2340);  
                      ftm_pwm_duty(S3010_FTM,S3010_CH,2300); 
                    }
                    if(Flag_500ms==500)
                    {
                    gpio_turn(PTB20);
                    gpio_turn(PTB21);
                    gpio_turn(PTB22);
                    gpio_turn(PTB23);
                    }
    }
  
if(Flag_5ms==5)Flag_5ms=0;
if(Flag_10ms==10)Flag_10ms=0;
if(Flag_20ms==20)Flag_20ms=0;
if(Flag_50ms==50)Flag_50ms=0;
if(Flag_100ms==100)Flag_100ms=0; 
if(Flag_200ms==200)Flag_200ms=0;  
if(Flag_500ms==500)Flag_500ms=0;
PIT_Flag_Clear(PIT1);      //清中断标志位
}
void PORTD_IRQHandler(void)
{


  //  PORT_FUNC(D,11,key_handler1);
    PORT_FUNC(D,12,key_handler2);
    PORT_FUNC(D,13,key_handler3);
    PORT_FUNC(D,14,key_handler4);

}


void key_handler2(void)
{
   //j+=1;gpio_turn(PTB22);if(j>4)j=1;
}
void key_handler3(void)
{
    disable_irq(PIT1_IRQn) ;
                        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
}
void key_handler4(void)
{
    enable_irq(PIT1_IRQn); 
}
