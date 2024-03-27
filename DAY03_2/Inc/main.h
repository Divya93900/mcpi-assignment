#ifndef LED_H_
#define LED_H_
#include"stm32f407xx.h"
#define LED_GPIO     GPIOD
#define LED_GPIO_EN   3
#define LED_GREEN     12
#define LED_ORANGE    13
#define LED_RED       14
#define LED_BLUE      15
void LedInit(uint32_t pin);
void Ledon(uint32_t pin);
void Ledoff(uint32_t pin);
void LedBlink(uint32_t pin,uint32_t delay);
#endif /* LED_H_ */
