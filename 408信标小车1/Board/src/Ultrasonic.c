#include "common.h"
#include "include.h"
uint32 Ultrasonic()
{
    uint32 timevar;
    uint32 flag;
    gpio_init(TRIG,GPO,0);
    gpio_init(ECHG,GPI,0);
    

        flag = 0;
        gpio_set(TRIG,1);               //������������
        pit_delay_us(PIT2,15);
        gpio_set(TRIG,0);
        
        while(gpio_get(ECHG) == 0);             //�ȴ���ƽ��ߣ��͵�ƽһֱ�ȴ�
        pit_time_start  (PIT2);                 //��ʼ��ʱ
        while(gpio_get(ECHG) == 1)              //�ȴ���ƽ��ͣ��ߵ�ƽһֱ�ȴ�
        {
            flag++;
            if(flag >FLAGWAIT)
            {
                break;
            }
        };             
        
        timevar = pit_time_get_us    (PIT2);    //ֹͣ��ʱ����ȡ��ʱʱ��
        if(flag <FLAGWAIT )
        {
            timevar = timevar * 340 /2/1000;
                
            if(timevar > 5)
            {
     //           printf("\n\n����Ϊ��%dmm",timevar); //��ӡ��ʱʱ��
            }
        }
        if(timevar > 5)
        return timevar;
        else
          return 0;
    
}