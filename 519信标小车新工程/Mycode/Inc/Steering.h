#ifndef _STEERING_H_
#define _STEERING_H_
#define S3010_FTM   FTM0
#define S3010_CH    FTM_CH1//C2
#define S3010_HZ    (100)

#define DUTY_MAX  2340
#define DUTY_MIN  1940      
#define DUTY_MID  2140
#define Coefficient_Trh    10               //定义二次偏差系数阀值 

typedef struct PD {
 
     unsigned char setpoint;          //设定值
     
     float Proportion_1;        //比例系数
     float Derivative_1;        //微分系数
     
     float Proportion_2;        //弯道系数  
     float Derivative_2;        //弯道系数
     
     int16      error;         //偏差
     int16 last_error;        //上一次偏差
     
     float duty;             //占空比的值
}PD;

void Steering_init();
void Steering_test();

//初始化PD参数
void PD_init(PD *pdvar);
//PD的计算
void PD_calculation(PD *pdvar,uint8 Mid);

#endif