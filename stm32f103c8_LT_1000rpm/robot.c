#include "robot.h"

void DRIVERMOTOR_INIT(void)
{
	GPIO_InitTypeDef gpio_in;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	gpio_in.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_in.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	gpio_in.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio_in);
}

void BUTTON_INIT(void)
{
	GPIO_InitTypeDef gpio_in;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	gpio_in.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_in.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_12;
	gpio_in.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio_in);
}

void LCDLED_INIT(void)
{
	GPIO_InitTypeDef gpio_in;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	gpio_in.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_in.GPIO_Pin = GPIO_Pin_11;
	gpio_in.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio_in);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
}

void BUZZER_INIT(void)
{
	GPIO_InitTypeDef gpio_in;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	gpio_in.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_in.GPIO_Pin = GPIO_Pin_6;
	gpio_in.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio_in);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_6);
}

uint8_t read_startbutton(void)
{
	return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12);
}

uint8_t read_combutton(void)
{
	return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10);
}

void lcd_led(uint8_t n)
{
	if(n>0) GPIO_SetBits(GPIOA,GPIO_Pin_11);	
	else GPIO_ResetBits(GPIOA,GPIO_Pin_11);		
}

void buzzer(uint8_t n)
{
	if(n>0) GPIO_SetBits(GPIOB,GPIO_Pin_6);	
	else GPIO_ResetBits(GPIOB,GPIO_Pin_6);	
}

void motor_right_rvs(uint8_t n)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	pwma(n);
}

void motor_right_fwd(uint8_t n)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	pwma(n);
}

void motor_right_stop(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	pwma(0);
}

void motor_left_rvs(uint8_t n)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	pwmb(n);
}

void motor_left_fwd(uint8_t n)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	pwmb(n);
}

void motor_left_stop(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
	pwmb(0);
}

void ROBOT_INIT(void)
{
	DRIVERMOTOR_INIT();
	BUTTON_INIT();					//active low
	BUZZER_INIT();					//active low
	LCDLED_INIT();					//active low
}