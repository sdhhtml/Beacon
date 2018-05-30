#ifndef _INITHANDLER_H_
#define _INITHANDLER_H_

void PORTA_IRQHandler();
void DMA0_IRQHandler();
void PIT1_IRQHandler(void);
void PORTD_IRQHandler(void);        //PORTD端口中断服务函数

void key_handler2(void); 
void key_handler3(void); 
void key_handler4(void);

void PORTA_IRQHandler_KEY(void);
void PORTE_IRQHandler_KEY(void);

void ZKkey1(void);
void ZKkey2(void);
void ZKkey3(void);
void ZKkey4(void);
void ZKkey5(void);
void ZKkey6(void);
#endif // !_INITHANDLER_H_