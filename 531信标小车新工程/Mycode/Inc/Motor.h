#ifndef _MOTOR_H_
#define _MOTOR_H_
#define MOTOR1_IO   PTE5
#define MOTOR2_IO   PTE6
#define MOTOR3_IO   PTE7
#define MOTOR4_IO   PTE10

#define MOTOR_FTM   FTM3
#define MOTOR1_PWM  FTM_CH0
#define MOTOR2_PWM  FTM_CH1
#define MOTOR3_PWM  FTM_CH2
#define MOTOR4_PWM  FTM_CH5

#define MOTOR1_PWM_IO  FTM3_CH0
#define MOTOR2_PWM_IO  FTM3_CH1
#define MOTOR3_PWM_IO  FTM3_CH2
#define MOTOR4_PWM_IO  FTM3_CH5

//extern uint8 FLAG_STOP;                  //停止标志
#define MAX_SPEED  8000                //定义速度最大值
#define MIN_SPEED  2000                //定义速度最小值

#define BANG_Trh   50                //定义棒棒阈值
#define WEI_Trh    10                //定义微分先行的阈值
#define Integration_Trh    50             //定义积分限幅的阈值
#define Min_Speed  1200               //定义最小速度

#define MOTOR_HZ    (15*1000)

typedef struct PID{
  int16 sp_motor;             //电机设定值
  
  
  float Kp_motor;                //Kp参数
  float Ki_motor;                //Ki参数
  float Kd_motor;                //Kd参数
  
  int result;
  int l_output;
  int r_output;
  int l_result;
  int r_result;
  int l_error;
  int r_error;
  
  
  int16 error;                  //本次偏差
  int16 last_error;             //上一次偏差
  int16 pre_error;              //上上次偏差
  
  int16 output;                 //本次输出
  int16 last_output;            //上次输出
  int16 pre_output;             //上上次输出
  
  int16 PD_Diff_output;         //差速的输出
  
}sPID;
void Motor_init();
void Motor_test();
void spid_init(sPID *spid);
void pid_calculation(sPID *spid,int8 Mid,int8 vertical);
#endif