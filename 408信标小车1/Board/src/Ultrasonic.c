#include "common.h"
#include "include.h"
uint32 Ultrasonic()
{
    uint32 timevar;
    uint32 flag;
    gpio_init(TRIG,GPO,0);
    gpio_init(ECHG,GPI,0);
    

        flag = 0;
        gpio_set(TRIG,1);               //产生触发脉冲
        pit_delay_us(PIT2,15);
        gpio_set(TRIG,0);
        
        while(gpio_get(ECHG) == 0);             //等待电平变高，低电平一直等待
        pit_time_start  (PIT2);                 //开始计时
        while(gpio_get(ECHG) == 1)              //等待电平变低，高电平一直等待
        {
            flag++;
            if(flag >FLAGWAIT)
            {
                break;
            }
        };             
        
        timevar = pit_time_get_us    (PIT2);    //停止计时，获取计时时间
        if(flag <FLAGWAIT )
        {
            timevar = timevar * 340 /2/1000;
                
            if(timevar > 5)
            {
     //           printf("\n\n距离为：%dmm",timevar); //打印延时时间
            }
        }
        if(timevar > 5)
        return timevar;
        else
          return 0;
    
}