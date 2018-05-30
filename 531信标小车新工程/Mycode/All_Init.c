#include "All_Init.h"
#include "include.h"
#include "common.h"

extern uint8 imgbuff[CAMERA_SIZE]; //定义存储接收图像的数组
extern PD pd_var;
extern sPID SPID;

void Beacon_Init()
{
    gpio_init(PTB20,GPO,1);
    gpio_init(PTB21,GPO,1);
    gpio_init(PTB22,GPO,1);
    gpio_init(PTB23,GPO,1);
    
    gpio_init(PTA5,GPI,1);//5
    gpio_init(PTA7,GPI,1);//4
    gpio_init(PTA8,GPI,1);//2
    gpio_init(PTA9,GPI,1);//1
    gpio_init(PTE25,GPI,1);//3
    gpio_init(PTE27,GPI,1);//6

    
    gpio_init(PTE24,GPI,1);//4
    gpio_init(PTE26,GPI,1);//3
    gpio_init(PTE28,GPI,1);//2
    gpio_init(PTA6,GPI,1);//1

    port_init(PTD12, ALT1 | IRQ_RISING | PULLUP ); 
    port_init(PTD13, ALT1 | IRQ_RISING | PULLUP ); 
    port_init(PTD14, ALT1 | IRQ_RISING | PULLUP );    
    
    
    port_init(PTA5, ALT1 | IRQ_FALLING | PULLUP ); 
    port_init(PTA7, ALT1 | IRQ_FALLING | PULLUP );
    port_init(PTA8, ALT1 | IRQ_FALLING | PULLUP );
    port_init(PTA9, ALT1 | IRQ_FALLING | PULLUP );
    port_init(PTE25, ALT1 | IRQ_FALLING | PULLUP );
    port_init(PTE27, ALT1 | IRQ_FALLING | PULLUP );
    
    port_init(PTE24, ALT1 | IRQ_FALLING | PULLUP ); 
    port_init(PTE26, ALT1 | IRQ_FALLING | PULLUP );
    port_init(PTE28, ALT1 | IRQ_FALLING | PULLUP );
    port_init(PTA6, ALT1 | IRQ_FALLING | PULLUP );
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
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler_KEY);    //设置PORTA的中断服务函数为 PORTA_IRQHandler
    enable_irq (PORTA_IRQn); 
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler_KEY);    //设置PORTE的中断服务函数为 PORTE_IRQHandler
    enable_irq (PORTE_IRQn); 
 //   ZK_keyinit();
    LCD_Init();
    LCD_CLS();
}
