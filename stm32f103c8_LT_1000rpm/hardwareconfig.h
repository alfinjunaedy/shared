#ifndef __HARDWARECONFIG_H
#define __HARDWARECONFIG_H
#include "stm32f10x.h"                  
#include "stdio.h"
#include "string.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC

//*************************************ADC*****************************************************************************************
void ADC_INIT(void);
uint16_t read_adc(uint8_t channel);
//=====================================adc=========================================================================================

//*************************************GY25****************************************************************************************
void USART3_INIT(void);
void putchar3(uint8_t c);
void USART_IT_Enable(void);
void NVIC_Config_Usart3(void);
void USART3_IRQHandler(void);
int read_gy25_roll(void);
int read_gy25_pitch(void);
int read_gy25_yaw(void);
void GY25_INIT(void);
//=====================================gy25========================================================================================

//*************************************PWM*****************************************************************************************
void PWM_INIT(void);
void pwma(uint8_t n);		//0(0%pwm=0v)	-	100(100%pwm=3.3v)
void pwmb(uint8_t n);
//=====================================pwm=========================================================================================

//*************************************ENCODER*************************************************************************************
void Enc_Pin_Config(void);
void Enc_Tim_Config(void);
void Enc_Nvic_Config(void);
void TIM4_IRQHandler(void);
void ENCODER_INIT(void);

int read_encoderL(void);
int read_encoderR(void);
void reset_encoderLR(void);
//=====================================encoder=====================================================================================

//*************************************LCD*****************************************************************************************
typedef enum {
  TLCD_OFF =0,    
  TLCD_ON,        
  TLCD_CURSOR,    
  TLCD_BLINK      
}TLCD_MODE_t;

#define  TLCD_INIT_PAUSE  100000  
#define  TLCD_PAUSE        50000  
#define  TLCD_CLK_PAUSE     1000  
#define  TLCD_MAXX            16  
#define  TLCD_MAXY             2  

#define  TLCD_CMD_INIT_DISPLAY  0x28   
#define  TLCD_CMD_ENTRY_MODE    0x06   
#define  TLCD_CMD_DISP_M0       0x08   
#define  TLCD_CMD_DISP_M1       0x0C   
#define  TLCD_CMD_DISP_M2       0x0E   
#define  TLCD_CMD_DISP_M3       0x0F    
#define  TLCD_CMD_CLEAR         0x01   

typedef enum 
{
  TLCD_RS = 0,  // RS-Pin
  TLCD_E  = 1,  // E-Pin
  TLCD_D4 = 2,  // DB4-Pin
  TLCD_D5 = 3,  // DB5-Pin
  TLCD_D6 = 4,  // DB6-Pin
  TLCD_D7 = 5,   // DB7-Pin
}TLCD_NAME_t;

#define  TLCD_ANZ   6 

typedef struct {
  TLCD_NAME_t TLCD_NAME;   // Name
  GPIO_TypeDef* TLCD_PORT; // Port
  const uint16_t TLCD_PIN; // Pin
  const uint32_t TLCD_CLK; // Clock
  BitAction TLCD_INIT;     // Init
}LCD_2X16_t;

void LCD_INIT(void);
void lcd_clear(void);
void lcd_SetMode(TLCD_MODE_t mode);
void lcd_putsf(char *ptr);
void lcd_gotoxy(uint8_t x, uint8_t y);
void lcd_WriteCG(uint8_t nr, uint8_t *pixeldata);
void lcd_PrintCG(uint8_t x, uint8_t y, uint8_t nr);
void lcd_int(uint32_t data);
//=====================================lcd=========================================================================================

//*************************************EEPROM**************************************************************************************
// CAN BE CHANGED
#define FEE_DENSITY_PAGES	4	    // how many pages are used 
#define FEE_PAGE_SIZE		1024	    // can be 1k or 2k check manual for used device
#define FEE_PAGE_BASE_ADDRESS 	0x0801F000  // choose location for the first EEPROMPage address on the top of flash

// DONT CHANGE
#define FEE_DENSITY_BYTES		((FEE_PAGE_SIZE / 2) * FEE_DENSITY_PAGES - 1)
#define FEE_LAST_PAGE_ADDRESS 	(FEE_PAGE_BASE_ADDRESS + (FEE_PAGE_SIZE * FEE_DENSITY_PAGES))
#define FEE_EMPTY_WORD			((uint16_t)0xFFFF)
#define FEE_ADDR_OFFSET(Address)(Address * 2) // 1Byte per Word will be saved to preserve Flash

// use this function to initialize the functionality
uint16_t FEE_Init(void);
void FEE_Erase (void);
uint16_t FEE_WriteDataByte (uint16_t Address, uint8_t DataByte);
uint8_t FEE_ReadDataByte (uint16_t Address);
//=====================================eeprom======================================================================================

//*************************************DELAY***************************************************************************************
void SYSTICK_INIT(void);
void TimeTick_Decrement(void);
void delay_us(u32 n);
void delay_1ms(void);
void delay_ms(u32 n);
void SysTick_Handler(void);
//=====================================delay=======================================================================================

void HARDWARECONFIG_INIT(void);
#endif