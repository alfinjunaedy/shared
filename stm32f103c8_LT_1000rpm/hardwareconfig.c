#include "hardwareconfig.h"

//*************************************ADC*****************************************************************************************
uint8_t adc_div=15; //3000-1500 to 0-255 range

void ADC_INIT(void)
{
	GPIO_InitTypeDef gpio_in;
	ADC_InitTypeDef adc_in;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
		
	gpio_in.GPIO_Mode = GPIO_Mode_AIN;
	gpio_in.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	gpio_in.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio_in);
	
	gpio_in.GPIO_Mode = GPIO_Mode_AIN;
	gpio_in.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	gpio_in.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio_in);
	
	ADC_StructInit(&adc_in);	
	adc_in.ADC_ContinuousConvMode = DISABLE;	
	adc_in.ADC_DataAlign = ADC_DataAlign_Right;
	adc_in.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	adc_in.ADC_Mode = ADC_Mode_Independent;
	adc_in.ADC_NbrOfChannel = 10; //channel
	adc_in.ADC_ScanConvMode = DISABLE;
	
	ADC_Init(ADC1,&adc_in);
	ADC_Cmd(ADC1,ENABLE);
}

uint16_t read_adc(uint8_t channel)
{	
	if(channel==0) ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_1Cycles5);
	if(channel==1) ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_1Cycles5);
	if(channel==2) ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_1Cycles5);
	if(channel==3) ADC_RegularChannelConfig(ADC1,ADC_Channel_3,1,ADC_SampleTime_1Cycles5);
	if(channel==4) ADC_RegularChannelConfig(ADC1,ADC_Channel_4,1,ADC_SampleTime_1Cycles5);
	if(channel==5) ADC_RegularChannelConfig(ADC1,ADC_Channel_5,1,ADC_SampleTime_1Cycles5);
	if(channel==6) ADC_RegularChannelConfig(ADC1,ADC_Channel_6,1,ADC_SampleTime_1Cycles5);
	if(channel==7) ADC_RegularChannelConfig(ADC1,ADC_Channel_7,1,ADC_SampleTime_1Cycles5);
	if(channel==8) ADC_RegularChannelConfig(ADC1,ADC_Channel_8,1,ADC_SampleTime_1Cycles5);
	if(channel==9) ADC_RegularChannelConfig(ADC1,ADC_Channel_9,1,ADC_SampleTime_1Cycles5);
	
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC1)/adc_div;
}
//=====================================adc=========================================================================================

//*************************************ENCODER*************************************************************************************
int l_count=0, r_count=0, l_temp=1, r_temp=1;

void Enc_Pin_Config(void)
{
	//left = PA15 & PB3(error-disactivate) | right = PB4 & PB5(disactivate)
	GPIO_InitTypeDef gpio_in;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	gpio_in.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_in.GPIO_Pin = GPIO_Pin_15;
	gpio_in.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio_in);
	
	gpio_in.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_in.GPIO_Pin = GPIO_Pin_4; 
	gpio_in.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio_in);
}

void Enc_Tim_Config(void)
{
	TIM_TimeBaseInitTypeDef tim_in;
	RCC_PCLK1Config(RCC_HCLK_Div1);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	//TIM4 is 36MHz on RCC_HCLK_Div1 (2.25MHz on Div16)
	//prescaler = 36, so TIM4 now is 1MHz
	//set clock = 100us = 0.1ms = 10000Hz
	//Tim_period = (timer_clock / PWm_freq) - 1
	//Tim_period = (1MHz / 10KHz) - 1 = 99
	tim_in.TIM_ClockDivision=TIM_CKD_DIV1;
	tim_in.TIM_CounterMode=TIM_CounterMode_Up;
	tim_in.TIM_Period=99; 
	tim_in.TIM_Prescaler=36;   
	tim_in.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM4,&tim_in);
	TIM_Cmd(TIM4,ENABLE);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
}

void Enc_Nvic_Config(void)
{
	NVIC_InitTypeDef nvic_in;	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	nvic_in.NVIC_IRQChannel=TIM4_IRQn;
	nvic_in.NVIC_IRQChannelCmd=ENABLE;
	nvic_in.NVIC_IRQChannelPreemptionPriority=0x00;
	nvic_in.NVIC_IRQChannelSubPriority=0x00;
	NVIC_Init(&nvic_in);
}

void ENCODER_INIT(void)
{
	Enc_Pin_Config();
	Enc_Tim_Config();
	Enc_Nvic_Config();
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		//interrupt timer program===============================================================================================
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)==1 && l_temp==1)
		{
			l_count++; l_temp=0;
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)==0)
		{
			l_temp=1;
		}
		
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==1 && r_temp==1)
		{
			r_count++; r_temp=0;
		}
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==0)
		{
			r_temp=1;
		}
		//======================================================================================================================
	}
}

int read_encoderL(void)
{
	return l_count;
}

int read_encoderR(void)
{
	return r_count;
}

void reset_encoderLR(void)
{
	l_count=0; r_count=0; l_temp=1; r_temp=1;
}
//=====================================encoder=====================================================================================

//*************************************GY25****************************************************************************************
unsigned char returnBuffgy[8], countergy=0, fullBuffgy=0;
int PRY[3];

void USART3_INIT(void)
{
	GPIO_InitTypeDef gpio_in;
	USART_InitTypeDef usart_in;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//pushpull output
	gpio_in.GPIO_Mode=GPIO_Mode_AF_PP;
	gpio_in.GPIO_Pin=GPIO_Pin_10; 
	gpio_in.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio_in);

	//floating input
	gpio_in.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	gpio_in.GPIO_Pin=GPIO_Pin_11; 
	gpio_in.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio_in);
	
	usart_in.USART_BaudRate=115200; //default baudrate of gy-25 (if soldered jumper 9600)
	usart_in.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	usart_in.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	usart_in.USART_Parity=USART_Parity_No;
	usart_in.USART_StopBits=USART_StopBits_1;
	usart_in.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART3,&usart_in); 
	USART_Cmd(USART3,ENABLE); 
	
	//GY-25 init
	delay_ms(5);//1000
	USART_SendData(USART3,0xA5);	//tilt calibration
	USART_SendData(USART3,0x54);
	delay_ms(5);//50

	USART_SendData(USART3,0xA5);	//heading calibration
	USART_SendData(USART3,0x55);
	delay_ms(5);//50

	USART_SendData(USART3,0xA5);	//ASCII calibation
	USART_SendData(USART3,0x53);
	delay_ms(5);//50

	USART_SendData(USART3,0xA5);	//start
	USART_SendData(USART3,0x52);
	delay_ms(5);//50
}

//*******************INTERUPT USART*********************************************//
void USART_IT_Enable(void)
{
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
}

void NVIC_Config_Usart3(void)
{
	NVIC_InitTypeDef nvic_in;	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	nvic_in.NVIC_IRQChannel=USART3_IRQn;
	nvic_in.NVIC_IRQChannelCmd=ENABLE;
	nvic_in.NVIC_IRQChannelPreemptionPriority=0x00;
	nvic_in.NVIC_IRQChannelSubPriority=0x00;
	NVIC_Init(&nvic_in);
}

void USART3_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE)==SET)
	{
		//***interupt_usart1_program*****************************//
		returnBuffgy[countergy] = (unsigned char) USART3->DR;
		if(countergy==0 & returnBuffgy[0]!=0xAA) return;
		countergy++;
		if(countergy==8){countergy=0; fullBuffgy=1;}
		//***********************************************//
	}
	if(fullBuffgy==1 && returnBuffgy[0]==0xAA && returnBuffgy[7]==0x55)
	{
		PRY[0] = (returnBuffgy[5]<<8|returnBuffgy[6])/100;  //pitch
		PRY[1] = (returnBuffgy[3]<<8|returnBuffgy[4])/100;  //roll
		PRY[2] = (returnBuffgy[1]<<8|returnBuffgy[2])/100;  //yaw
		if(PRY[0]>360) PRY[0]=PRY[0]-300;
		if(PRY[1]>360) PRY[1]=PRY[1]-300;
		if(PRY[2]>360) PRY[2]=PRY[2]-300;
	}
	fullBuffgy = 0;
}

int read_gy25_roll(void)
{
	return PRY[1];
}

int read_gy25_pitch(void)
{
	return PRY[0];
}

int read_gy25_yaw(void)
{	
	return PRY[2];
}

void GY25_INIT(void)
{
	USART3_INIT();
	USART_IT_Enable();
  NVIC_Config_Usart3();
}
//=====================================gy25========================================================================================

//*************************************PWM*****************************************************************************************
void PWM_INIT(void)
{
	GPIO_InitTypeDef gpio_in;
	TIM_TimeBaseInitTypeDef tim_in;
	TIM_OCInitTypeDef tim_in2;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	gpio_in.GPIO_Mode=GPIO_Mode_AF_PP;
	gpio_in.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;
	gpio_in.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio_in);
	
	//fpwm=(10.5MHz*2)/(210*100)=1000Hz
	//0=0v 100=3.3v
	TIM_TimeBaseStructInit(&tim_in);
	tim_in.TIM_ClockDivision=0;
	tim_in.TIM_CounterMode=TIM_CounterMode_Up;
	tim_in.TIM_Period=100-1;
	tim_in.TIM_Prescaler=210-1;
	TIM_TimeBaseInit(TIM1,&tim_in);
	TIM_Cmd(TIM1,ENABLE);
	
	TIM_OCStructInit(&tim_in2);
	tim_in2.TIM_OCMode=TIM_OCMode_PWM1;	//0=0v 100=3.3v
	tim_in2.TIM_OutputState=TIM_OutputState_Enable;
	tim_in2.TIM_Pulse=0;
	tim_in2.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM1,&tim_in2);
	TIM_OC2Init(TIM1,&tim_in2);
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Enable);
	TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Enable);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	TIM_Cmd(TIM1,ENABLE);
}

void pwma(uint8_t n)
{
	TIM_SetCompare1(TIM1,n);
}

void pwmb(uint8_t n)
{
	TIM_SetCompare2(TIM1,n);
}
//=====================================pwm=========================================================================================

//*************************************LCD*****************************************************************************************
void lcd_InitIO(void);
void lcd_PinLo(TLCD_NAME_t lcd_pin);
void lcd_PinHi(TLCD_NAME_t lcd_pin);
void lcd_Clk(void);
void lcd_InitSequens(void);
void lcd_Cmd(uint8_t wert);
void lcd_Data(uint8_t wert);
void lcd_gotoxy(uint8_t x, uint8_t y);
void lcd_Delay(volatile uint32_t nCount);

/***********************Pin Configuration*************************/
LCD_2X16_t lcdpins[] = {
  {TLCD_RS ,GPIOC,GPIO_Pin_13 ,RCC_APB2Periph_GPIOC,Bit_RESET},
  {TLCD_E  ,GPIOC,GPIO_Pin_14 ,RCC_APB2Periph_GPIOC,Bit_RESET},
  {TLCD_D4 ,GPIOC,GPIO_Pin_15 ,RCC_APB2Periph_GPIOC,Bit_RESET},
  {TLCD_D5 ,GPIOB,GPIO_Pin_7 ,RCC_APB2Periph_GPIOB,Bit_RESET},
  {TLCD_D6 ,GPIOB,GPIO_Pin_8 ,RCC_APB2Periph_GPIOB,Bit_RESET},
  {TLCD_D7 ,GPIOB,GPIO_Pin_9 ,RCC_APB2Periph_GPIOB,Bit_RESET},
};
/****************************************************************/

void lcd_SetMode(TLCD_MODE_t mode)
{
  if(mode==TLCD_OFF) lcd_Cmd(TLCD_CMD_DISP_M0);
  if(mode==TLCD_ON) lcd_Cmd(TLCD_CMD_DISP_M1);
  if(mode==TLCD_CURSOR) lcd_Cmd(TLCD_CMD_DISP_M2);
  if(mode==TLCD_BLINK) lcd_Cmd(TLCD_CMD_DISP_M3);
}

void lcd_clear(void)
{
  lcd_Cmd(TLCD_CMD_CLEAR);
  lcd_Delay(TLCD_PAUSE);
}

void lcd_putsf(char *ptr)
{
  //lcd_gotoxy(x,y);
  while (*ptr != 0) {
    lcd_Data(*ptr);
    ptr++;
  }
}

void lcd_WriteCG(uint8_t nr, uint8_t *pixeldata)
{
  uint8_t n;
  if(nr>7) nr=7;
  nr=(nr<<3);
  nr|=0x40;
  lcd_Cmd(nr);
  for(n=0;n<8;n++) {
    lcd_Data(pixeldata[n]);
  }
}

void lcd_PrintCG(uint8_t x, uint8_t y, uint8_t nr)
{
  if(nr>7) nr=7;
  lcd_gotoxy(x,y);
  lcd_Data(nr);
}

void lcd_InitIO(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  TLCD_NAME_t lcd_pin;
  
  for(lcd_pin=0;lcd_pin<TLCD_ANZ;lcd_pin++) {
		RCC_APB2PeriphClockCmd(lcdpins[lcd_pin].TLCD_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = lcdpins[lcd_pin].TLCD_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(lcdpins[lcd_pin].TLCD_PORT, &GPIO_InitStructure);

    if(lcdpins[lcd_pin].TLCD_INIT==Bit_RESET) {
      lcd_PinLo(lcd_pin);
    }
    else {
      lcd_PinHi(lcd_pin);
    }
  }  
}

void lcd_PinLo(TLCD_NAME_t lcd_pin)
{
	lcdpins[lcd_pin].TLCD_PORT->BRR = lcdpins[lcd_pin].TLCD_PIN;
}


void lcd_PinHi(TLCD_NAME_t lcd_pin)
{
	lcdpins[lcd_pin].TLCD_PORT->BSRR = lcdpins[lcd_pin].TLCD_PIN;
}


void lcd_Clk(void)
{
  lcd_PinHi(TLCD_E);
  lcd_Delay(TLCD_CLK_PAUSE);
  lcd_PinLo(TLCD_E);
  lcd_Delay(TLCD_CLK_PAUSE);  
}

void lcd_InitSequens(void)
{
  lcd_PinHi(TLCD_D4);
  lcd_PinHi(TLCD_D5);
  lcd_PinLo(TLCD_D6);
  lcd_PinLo(TLCD_D7);
  lcd_Clk();
  lcd_Delay(TLCD_PAUSE);
  lcd_Clk();
  lcd_Delay(TLCD_PAUSE);
  lcd_Clk();
  lcd_Delay(TLCD_PAUSE);
  lcd_PinLo(TLCD_D4);
  lcd_PinHi(TLCD_D5);
  lcd_PinLo(TLCD_D6);
  lcd_PinLo(TLCD_D7);
  lcd_Clk();
  lcd_Delay(TLCD_PAUSE);
}

void lcd_Cmd(uint8_t wert)
{
  lcd_PinLo(TLCD_RS);        
  if((wert&0x80)!=0) lcd_PinHi(TLCD_D7); else lcd_PinLo(TLCD_D7);
  if((wert&0x40)!=0) lcd_PinHi(TLCD_D6); else lcd_PinLo(TLCD_D6);
  if((wert&0x20)!=0) lcd_PinHi(TLCD_D5); else lcd_PinLo(TLCD_D5);
  if((wert&0x10)!=0) lcd_PinHi(TLCD_D4); else lcd_PinLo(TLCD_D4);
  lcd_Clk();      
  if((wert&0x08)!=0) lcd_PinHi(TLCD_D7); else lcd_PinLo(TLCD_D7);
  if((wert&0x04)!=0) lcd_PinHi(TLCD_D6); else lcd_PinLo(TLCD_D6);
  if((wert&0x02)!=0) lcd_PinHi(TLCD_D5); else lcd_PinLo(TLCD_D5);
  if((wert&0x01)!=0) lcd_PinHi(TLCD_D4); else lcd_PinLo(TLCD_D4);
  lcd_Clk();  
}

void lcd_Data(uint8_t wert)
{
  lcd_PinHi(TLCD_RS);        
  if((wert&0x80)!=0) lcd_PinHi(TLCD_D7); else lcd_PinLo(TLCD_D7);
  if((wert&0x40)!=0) lcd_PinHi(TLCD_D6); else lcd_PinLo(TLCD_D6);
  if((wert&0x20)!=0) lcd_PinHi(TLCD_D5); else lcd_PinLo(TLCD_D5);
  if((wert&0x10)!=0) lcd_PinHi(TLCD_D4); else lcd_PinLo(TLCD_D4);
  lcd_Clk();      
  if((wert&0x08)!=0) lcd_PinHi(TLCD_D7); else lcd_PinLo(TLCD_D7);
  if((wert&0x04)!=0) lcd_PinHi(TLCD_D6); else lcd_PinLo(TLCD_D6);
  if((wert&0x02)!=0) lcd_PinHi(TLCD_D5); else lcd_PinLo(TLCD_D5);
  if((wert&0x01)!=0) lcd_PinHi(TLCD_D4); else lcd_PinLo(TLCD_D4);
  lcd_Clk();  
}

void lcd_gotoxy(uint8_t x, uint8_t y)
{
  uint8_t wert;

  if(x>=TLCD_MAXX) x=0;
  if(y>=TLCD_MAXY) y=0;

  wert=(y<<6);
  wert|=x;
  wert|=0x80;
  lcd_Cmd(wert);
}

void lcd_Delay(volatile uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void lcd_int(uint32_t data)
{
	unsigned char n[16];
	sprintf(n,"%d",data);
  lcd_putsf(n);
}

void LCD_INIT(void)
{
	int i;
  lcd_InitIO();
  lcd_Delay(TLCD_INIT_PAUSE);
  lcd_InitSequens();
  lcd_Cmd(TLCD_CMD_INIT_DISPLAY);
  lcd_Cmd(TLCD_CMD_ENTRY_MODE);
  lcd_Cmd(TLCD_CMD_DISP_M1);
  lcd_Cmd(TLCD_CMD_CLEAR);
  lcd_Delay(TLCD_PAUSE);
}
//=====================================lcd=========================================================================================

//*************************************EEPROM**************************************************************************************
uint8_t DataBuf[FEE_PAGE_SIZE];
uint16_t
FEE_Init(void) {
	// unlock flash
	FLASH_Unlock();

	// Clear Flags
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);

	return FEE_DENSITY_BYTES;
}
/*****************************************************************************
*  Erase the whole reserved Flash Space used for user Data
******************************************************************************/
void
FEE_Erase (void) {

	int page_num = 0;

	// delete all pages from specified start page to the last page
	do {
		FLASH_ErasePage(FEE_PAGE_BASE_ADDRESS + (page_num * FEE_PAGE_SIZE));
		page_num++;
	} while (page_num < FEE_DENSITY_PAGES);
}

uint16_t
FEE_WriteDataByte (uint16_t Address, uint8_t DataByte) {

	FLASH_Status FlashStatus = FLASH_COMPLETE;

	uint32_t page;
	int i;

	// exit if desired address is above the limit (e.G. under 2048 Bytes for 4 pages)
	if (Address > FEE_DENSITY_BYTES) {
		return 0;
	}

	// calculate which page is affected (Pagenum1/Pagenum2...PagenumN)
	page = (FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address)) & 0x00000FFF;

	if (page % FEE_PAGE_SIZE) page = page + FEE_PAGE_SIZE;
	page = (page / FEE_PAGE_SIZE) - 1;

	// if current data is 0xFF, the byte is empty, just overwrite with the new one
	if ((*(uint16_t*)(FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address))) == FEE_EMPTY_WORD) {

		FlashStatus = FLASH_ProgramHalfWord(FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address), (uint16_t)(0x00FF & DataByte));
	}
	else {

		// Copy Page to a buffer
		memcpy(DataBuf, (uint8_t*)FEE_PAGE_BASE_ADDRESS + (page * FEE_PAGE_SIZE), FEE_PAGE_SIZE); // !!! Calculate base address for the desired page

		// check if new data is differ to current data, return if not, proceed if yes
		if (DataByte == *(uint8_t*)(FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address))) {
			return 0;
		}

		// manipulate desired data byte in temp data array if new byte is differ to the current
		DataBuf[FEE_ADDR_OFFSET(Address)] = DataByte;

		//Erase Page
		FlashStatus = FLASH_ErasePage(FEE_PAGE_BASE_ADDRESS + page);

		// Write new data (whole page) to flash if data has beed changed
		for(i = 0; i < (FEE_PAGE_SIZE / 2); i++) {
			if ((uint16_t)(0xFF00 | DataBuf[FEE_ADDR_OFFSET(i)]) != 0xFFFF) {
				FlashStatus = FLASH_ProgramHalfWord((FEE_PAGE_BASE_ADDRESS + (page * FEE_PAGE_SIZE)) + (i * 2), (uint16_t)(0xFF00 | DataBuf[FEE_ADDR_OFFSET(i)]));
			}
		}

	}
	return FlashStatus;
}
/*****************************************************************************
*  Read once data byte from a specified address.
*******************************************************************************/
uint8_t
FEE_ReadDataByte (uint16_t Address) {

	uint8_t DataByte = 0xFF;

	// Get Byte from specified address
	DataByte = (*(uint8_t*)(FEE_PAGE_BASE_ADDRESS + FEE_ADDR_OFFSET(Address)));

	return DataByte;
}
//=====================================eeprom======================================================================================

//*************************************DELAY***************************************************************************************
static __IO uint32_t sysTickCounter;			

void SYSTICK_INIT(void)
{
	while(SysTick_Config(SystemCoreClock/1000000) != 0);
}

void TimeTick_Decrement(void)
{
	if(sysTickCounter != 0x00)
	{
		sysTickCounter--;
	}
}

void delay_us(u32 n)
{
	sysTickCounter = n;
	while(sysTickCounter != 0);
}

void delay_1ms(void)
{
	sysTickCounter = 1000;
	while(sysTickCounter != 0);
}

void delay_ms(u32 n)
{
	while(n--)
	{
		delay_1ms();
	}
}

void SysTick_Handler(void)
{
	TimeTick_Decrement();
}
//=====================================delay=======================================================================================

void HARDWARECONFIG_INIT(void)
{
	SYSTICK_INIT();
	LCD_INIT();
	PWM_INIT();
	ADC_INIT();
	
	//pilih salah satu antara encoder atau gy25
	ENCODER_INIT();
	//GY25_INIT();
}