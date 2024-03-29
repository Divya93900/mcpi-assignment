/*
 * LCD1.C
 *
 *  Created on: Mar 29, 2024
 *      Author: divya
 */
#include "LCD.h"

void LcdBusywait(void){
	uint32_t  state;
	//MAKE D7/BF PIN AS INPUT
	LCD_DATA_GPIO->MODER&=~(BV(LCD_BF*2)|BV(LCD_BF*2+1));
//set rs=0,rw1,en=1
	LCD_CTRL_GPIO->BSRR =BV(LCD_RS_CLR)|BV(LCD_RW)|BV(LCD_EN);
			//READ DATA
	do{

		state=LCD_DATA_GPIO->IDR;
	}
	while((state&BV(LCD_BF))!=0);
	//SET RW=0,EN=0
	LCD_CTRL_GPIO->BSRR=BV(LCD_RW_CLR)|BV(LCD_EN_CLR);
	//MAKE D7/BF AS OUTPUT
	LCD_DATA_GPIO->MODER|=BV(LCD_BF*2);

}
void LcdWriteNibble(uint8_t rs,uint8_t val)
{
//set RS,RW=0
	if(rs==LCD_CMD)//RS=0
		LCD_CTRL_GPIO->BSRR=BV(LCD_RS_CLR)|BV(LCD_RW_CLR);
	else//RS=1
	LCD_CTRL_GPIO-> BSRR =BV(LCD_RS)|BV(LCD_RW_CLR);
	//WRITE DATA(4 BITS)
	LCD_DATA_GPIO->ODR&=~(BV(LCD_D7)|BV(LCD_D6)|BV(LCD_D5)|BV(LCD_D4));//bracket
	LCD_DATA_GPIO->ODR|=val<<LCD_D4;
	//FALLING EDGE ON EN
	LCD_CTRL_GPIO->BSRR =BV(LCD_EN);
	DelayMs(1);//spell
	LCD_CTRL_GPIO->BSRR=BV(LCD_EN_CLR);
}
void LcdWrite(uint8_t rs,uint8_t val)
{
	uint8_t high =val>>4,low=val&0x0F;
	//write high niddle
	LcdWriteNibble(rs,high);
	//low
	LcdWriteNibble(rs,low);
	//busy
	LcdBusywait();
	DelayMs(3);
}
void LcdInit(void)
{
	DelayMs(50);
 RCC->AHB1ENR |=BV(LCD_DATA_GPIO_EN);
 RCC->AHB1ENR |=BV(LCD_CTRL_GPIO_EN);
 //INITIALIZE LCD DATA PINS
 LCD_DATA_GPIO->MODER &=~(BV(LCD_D7*2+1)|BV(LCD_D6*2+1)|BV(LCD_D5*2+1)|BV(LCD_D4*2+1));
 LCD_DATA_GPIO->MODER |=~(BV(LCD_D7*2)|BV(LCD_D6*2)|BV(LCD_D5*2)|BV(LCD_D4*2));
 LCD_DATA_GPIO->OSPEEDR &=~(BV(LCD_D7*2+1)|BV(LCD_D7*2)|BV(LCD_D6*2+1)|BV(LCD_D6*2)|BV(LCD_D5*2+1)|BV(LCD_D5*2)|BV(LCD_D4*2+1)|BV(LCD_D4*2));
 LCD_DATA_GPIO->PUPDR &=~(BV(LCD_D7*2+1)|BV(LCD_D7*2)|BV(LCD_D6*2+1)|BV(LCD_D6*2)|BV(LCD_D5*2+1)|BV(LCD_D5*2)|BV(LCD_D4*2+1)|BV(LCD_D4*2));//bvled
 LCD_DATA_GPIO->OTYPER &=~(BV(LCD_D7)|BV(LCD_D6)|BV(LCD_D5)|BV(LCD_D4));
  //CLEAR PINS
 LCD_DATA_GPIO->BSRR |=~(BV(LCD_D7_CLR)|BV(LCD_D6_CLR)|BV(LCD_D5_CLR)|BV(LCD_D4_CLR));
 //INITIALIZE LCD CTRL PINS

 	LCD_CTRL_GPIO->MODER &= ~(BV(LCD_RS*2+1) | BV(LCD_RW*2+1) | BV(LCD_EN*2+1));
 	LCD_CTRL_GPIO->MODER |= (BV(LCD_RS*2) | BV(LCD_RW*2) | BV(LCD_EN*2));
 	LCD_CTRL_GPIO->OSPEEDR &= ~(BV(LCD_RS*2+1) | BV(LCD_RS*2) | BV(LCD_RW*2+1) | BV(LCD_RW*2) | BV(LCD_EN*2+1) | BV(LCD_EN*2));
 	LCD_CTRL_GPIO->PUPDR &= ~(BV(LCD_RS*2+1) | BV(LCD_RS*2) | BV(LCD_RW*2+1) | BV(LCD_RW*2) | BV(LCD_EN*2+1) | BV(LCD_EN*2));
 	LCD_CTRL_GPIO->OTYPER &= ~(BV(LCD_RS) | BV(LCD_RW) | BV(LCD_EN));

 	//clear lcd ctrl pins
 	LCD_CTRL_GPIO->BSRR = (BV(LCD_RS_CLR) | BV(LCD_RW_CLR) | BV(LCD_EN_CLR));

	DelayMs(200);
	//LCD INIITIALISATION
	//4-BIN MODE,2 LINES,FONT
	LcdWrite(LCD_CMD,LCD_FN_SET);
	//DISPLAY
	LcdWrite(LCD_CMD,LCD_DISP_ON);
	//entry mode
	LcdWrite(LCD_CMD,LCD_ENTRY_MODE);
	//CLEAR DISPLAY
	LcdWrite(LCD_CMD,LCD_CLEAR);
	DelayMs(200);
}


void Lcdputs(uint8_t line,char str[])

{
	int i;
	LcdWrite(LCD_CMD,line);
	for(i=0;str[i]!='\0';i++)
		LcdWrite(LCD_DATA,str[i]);
}








