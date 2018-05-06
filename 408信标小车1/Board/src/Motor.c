#include "common.h"
#include "include.h"
int16 l_result,r_result;
int16 result;                            //左右电机转速平均值
float motor_increment = 0;                                 //增量式PID电机的增量
float kp = 0;                                    //起始时刻给定一个大的kp值用
int16 motor_duty = 0;
//电机PID变量的声明
sPID SPID;
uint8 Mid_error = 0;                             //定义速度控制中线偏差

void Motor_init()
{
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,5000);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,5000);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,5000);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,5000);      //初始化 电机 PWM
}
void Motor_test()
{
        int i = 3000;
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,i);
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,i);
}

//微分先行PID的参数
void spid_init(sPID *spid)                           //速度PID的初始化
{
    spid->sp_motor = 0;                                            //电机设定值
  
    spid->Kp_motor = 10;                                                 //Kp参数
    spid->Ki_motor = 0;                                                 //Ki参数
    spid->Kd_motor = 0;                                                 //Kd参数
    kp = spid->Kp_motor;
    spid->error = 0;                                                   //本次偏差
    spid->last_error = 0;                                            //上一次偏差
    spid->pre_error = 0;                                             //上上次偏差
    
    spid->output = 0;
    spid->last_output = 0;
    spid->pre_output = 0;
    
    spid->PD_Diff_output = 0;                                      //差速输出为0
} 

//微分先行的PID算法
void pid_calculation(sPID *spid,uint8 Mid)
{
    spid->error = spid->sp_motor - result;                        //设定值-测量值   
    //积分限幅
    if(spid->error > Integration_Trh)  
        motor_increment = spid->Kp_motor * (spid->error  - spid->last_error)
                        + spid->Kd_motor * (spid->output - 2 * spid->last_output + spid->pre_output) 
                        - spid->Kd_motor * (spid->output - spid->last_output);
    else
        motor_increment = spid->Kp_motor * (spid->error  - spid->last_error)
                        + spid->Ki_motor * spid->error
                        + spid->Kd_motor * (spid->output - 2 * spid->last_output + spid->pre_output) 
                        - spid->Kd_motor * (spid->output - spid->last_output);
   // motor_duty = (int16)((float)motor_duty + motor_increment);
   // spid->output = motor_duty;                                       //输出限幅控制
    
    spid->output = (int16)((float)spid->last_output + motor_increment);
    
                                     
            //更新数据值
    spid ->pre_error = spid ->last_error;
    spid ->last_error = spid ->error;
    spid ->pre_output = spid->last_output;
    spid ->last_output = spid->output;
          
//    result[0] = 0;
//    result[1] = 0;

    if(spid->output > 7000)     
        spid->output = 7000;     
    if(spid->output < -7000)      
        spid->output = -7000;      
    
    if(!FLAG_STOP && fabs(Mid - 64) > 5)                                            //当不是停车且中线偏差大于四时，即转弯标志
    {
        spid->PD_Diff_output = PD_Diff_calculation(Mid);
       // spid->PD_Diff_output = 0;
    }
    else
    {
        spid->PD_Diff_output = 0;                                                   //差速输出为0
    }
    if(spid->output - spid->PD_Diff_output >= 0 && spid->output + spid->PD_Diff_output >= 0)                                         //正转
    {
         //左电机
         ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,              0);
         ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,   spid->output - spid->PD_Diff_output);
         //右电机
         ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,   spid->output + spid->PD_Diff_output);
         ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,              0);
    }
    else if(spid->output - spid->PD_Diff_output >= 0 && spid->output + spid->PD_Diff_output <= 0)                                         //反转
    {    
        //
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,                0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,     spid->output - spid->PD_Diff_output);
        //
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,                0);      
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,    -(spid->output + spid->PD_Diff_output));
    }
    else if(spid->output - spid->PD_Diff_output <= 0 && spid->output + spid->PD_Diff_output >= 0)                                         //反转
    {    
        //
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,      -(spid->output - spid->PD_Diff_output));          
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,                0);
        //
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,      spid->output + spid->PD_Diff_output);      
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,                0);
    }
    else
    {    
        //
        ftm_pwm_duty(FTM0, MOTOR2_PWM,     -(spid->output - spid->PD_Diff_output)); 
        ftm_pwm_duty(FTM0, MOTOR1_PWM,                0);
        //
        ftm_pwm_duty(FTM0, MOTOR4_PWM,                0);      
        ftm_pwm_duty(FTM0, MOTOR3_PWM,    -(spid->output + spid->PD_Diff_output));
    }
//设置虚拟示波器的输出显示
    

    
}

