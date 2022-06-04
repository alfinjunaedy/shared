#include "movements.h"

#define bin_to_byte(b9,b8,b7,b6,b5,b4,b3,b2,b1,b0) ((b9<<9)+(b8<<8)+(b7<<7)+(b6<<6)+(b5<<5)+(b4<<4)+(b3<<3)+(b2<<2)+(b1<<1)+b0)

unsigned char buf2[16];
int i,j,count,adc[10],sp[10],ee[10];
int white[10]={0,0,0,0,0,0,0,0,0,0},black[10]={999,999,999,999,999,999,999,999,999,999};
int rate=0,error=0,last_error=0,KP,P,KI,I,KD,D,pwmka,pwmki,SP,datasensor;
int encoderL, encoderR;
int temp_gy, gy_move=0, res_gy, yaw;	
	
void calibration(void)
{
	for(i=0;i<10;i++)
	{
		adc[i] = read_adc(i);
		if(adc[i]>=white[i]) white[i]=adc[i];
		if(adc[i]<=black[i]) black[i]=adc[i];
	}
}

void start_sensor_line_fast(void)
{	
	KP = 25; 	//lurus 20 20 85
	KD = 20; 	
	KI = 0;
	SP = 80;	//set point motor
	
	for(i=0;i<10;i++)
	{
		adc[i] = read_adc(i);
		if(adc[i]<sp[i]) adc[i]=1;
		else adc[i] = 0;
	}
	
	datasensor=(adc[9]*512)+(adc[8]*256)+(adc[7]*128)+(adc[6]*64)+(adc[5]*32)+(adc[4]*16)+(adc[3]*8)+(adc[2]*4)+(adc[1]*2)+(adc[0]*1); 
	switch(datasensor)
	{
		case bin_to_byte(1,0,0,0,0,0,0,0,0,0) : error=-8;break;
		case bin_to_byte(1,1,0,0,0,0,0,0,0,0) : error=-7;break;
		case bin_to_byte(1,1,1,0,0,0,0,0,0,0) : error=-6;break;
		case bin_to_byte(0,1,1,0,0,0,0,0,0,0) : error=-5;break;
		case bin_to_byte(0,1,1,1,0,0,0,0,0,0) : error=-4;break;
		case bin_to_byte(0,0,1,1,0,0,0,0,0,0) : error=-3;break;
		case bin_to_byte(0,0,1,1,1,0,0,0,0,0) : error=-2;break;
		case bin_to_byte(0,0,0,1,1,0,0,0,0,0) : error=-1;break;
		case bin_to_byte(0,0,0,1,1,1,0,0,0,0) : error=0;break;
		
		case bin_to_byte(0,0,0,0,1,1,0,0,0,0) : error=0;break;
		case bin_to_byte(0,0,0,1,1,1,1,0,0,0) : error=0;break;
		
		case bin_to_byte(0,0,0,0,1,1,1,0,0,0) : error=0;break;
		case bin_to_byte(0,0,0,0,0,1,1,0,0,0) : error=1;break;
		case bin_to_byte(0,0,0,0,0,1,1,1,0,0) : error=2;break;
		case bin_to_byte(0,0,0,0,0,0,1,1,0,0) : error=3;break;
		case bin_to_byte(0,0,0,0,0,0,1,1,1,0) : error=4;break;
		case bin_to_byte(0,0,0,0,0,0,0,1,1,0) : error=5;break;
		case bin_to_byte(0,0,0,0,0,0,0,1,1,1) : error=6;break;
		case bin_to_byte(0,0,0,0,0,0,0,0,1,1) : error=7;break;
		case bin_to_byte(0,0,0,0,0,0,0,0,0,1) : error=8;break;
		
		case bin_to_byte(0,0,0,0,0,0,0,0,0,0) : {
																						if(error>0) {pwmka=0;pwmki=50;}
																						else if(error<0) {pwmka=50;pwmki=0;}
																						else {pwmka=50;pwmki=50;}
																						}
																						break;
		
		default : 
		{
			if(error>0) {pwmka=0;pwmki=50;}
			else if(error<0) {pwmka=50;pwmki=0;}
			else {pwmka=50;pwmki=50;}
			break;
		}
	}
	rate = error-last_error;
	
	P = KP*error;
	D = KD*rate;
	I = (KI/10)*(rate+last_error);
	
	last_error = error;
	
	pwmka = SP-(P+D);
	pwmki = SP+(P+D);
	if(pwmka>100) pwmka=100;
	if(pwmki>100) pwmki=100;
	if(pwmka<0) pwmka=0;
	if(pwmki<0) pwmki=0;
}

void start_sensor_line_slow(void)
{	
	KP = 25; 	//10 20 60
	KD = 15; 	
	KI = 0;
	SP = 40;	//37 set point motor
	
	for(i=0;i<10;i++)
	{
		adc[i] = read_adc(i);
		if(adc[i]<sp[i]) adc[i]=1;
		else adc[i] = 0;
	}
	
	datasensor=(adc[9]*512)+(adc[8]*256)+(adc[7]*128)+(adc[6]*64)+(adc[5]*32)+(adc[4]*16)+(adc[3]*8)+(adc[2]*4)+(adc[1]*2)+(adc[0]*1); 
	switch(datasensor)
	{
		case bin_to_byte(1,0,0,0,0,0,0,0,0,0) : error=-8;break;
		case bin_to_byte(1,1,0,0,0,0,0,0,0,0) : error=-7;break;
		case bin_to_byte(0,1,0,0,0,0,0,0,0,0) : error=-6;break;
		case bin_to_byte(0,1,1,0,0,0,0,0,0,0) : error=-5;break;
		case bin_to_byte(0,0,1,0,0,0,0,0,0,0) : error=-4;break;
		case bin_to_byte(0,0,1,1,0,0,0,0,0,0) : error=-3;break;
		case bin_to_byte(0,0,0,1,0,0,0,0,0,0) : error=-2;break;
		case bin_to_byte(0,0,0,1,1,0,0,0,0,0) : error=-1;break;
		case bin_to_byte(0,0,0,0,1,0,0,0,0,0) : error=0;break;
		
		case bin_to_byte(0,0,0,0,1,1,0,0,0,0) : error=0;break;
		
		case bin_to_byte(0,0,0,0,0,1,0,0,0,0) : error=0;break;
		case bin_to_byte(0,0,0,0,0,1,1,0,0,0) : error=1;break;
		case bin_to_byte(0,0,0,0,0,0,1,0,0,0) : error=2;break;
		case bin_to_byte(0,0,0,0,0,0,1,1,0,0) : error=3;break;
		case bin_to_byte(0,0,0,0,0,0,0,1,0,0) : error=4;break;
		case bin_to_byte(0,0,0,0,0,0,0,1,1,0) : error=5;break;
		case bin_to_byte(0,0,0,0,0,0,0,0,1,0) : error=6;break;
		case bin_to_byte(0,0,0,0,0,0,0,0,1,1) : error=7;break;
		case bin_to_byte(0,0,0,0,0,0,0,0,0,1) : error=8;break;
		
		case bin_to_byte(0,0,0,0,0,0,0,0,0,0) : {
																						if(error>0) {pwmka=0;pwmki=50;}
																						else if(error<0) {pwmka=50;pwmki=0;}
																						else {pwmka=50;pwmki=50;}
																						}
																						break;
		
		default : 
		{
			if(error>0) {pwmka=0;pwmki=50;}
			else if(error<0) {pwmka=50;pwmki=0;}
			else {pwmka=50;pwmki=50;}
			break;
		}
	}
	rate = error-last_error;
	
	P = KP*error;
	D = KD*rate;
	I = (KI/10)*(rate+last_error);
	
	last_error = error;
	
	pwmka = SP-(P+D);
	pwmki = SP+(P+D);
	if(pwmka>100) pwmka=100;
	if(pwmki>100) pwmki=100;
	if(pwmka<0) pwmka=0;
	if(pwmki<0) pwmki=0;
}

void forward_upto_left(void)
{
	count=0;
	while(1)
	{
		start_sensor_line_slow();

		motor_left_fwd(pwmki);
		motor_right_fwd(pwmka);
		
		//left 9 . . . 5 4 . . . 0 right
		//black=1 white=0
		if(((adc[9]==1 && adc[5]==1) || (adc[8]==1 && adc[4]==1)) && count>100)
		{
			lcd_led(0);delay_ms(20);lcd_led(1);break;
		}
		count++;
	}
}

void forward_upto_right(void)
{
	count=0;
	while(1)
	{
		start_sensor_line_slow();

		motor_left_fwd(pwmki);
		motor_right_fwd(pwmka);
		
		//left 9 . . . 5 4 . . . 0 right
		//black=1 white=0
		if(((adc[0]==1 && adc[4]==1) || (adc[1]==1 && adc[5]==1)) && count>100)
		{
			lcd_led(0);delay_ms(20);lcd_led(1);break;
		}
		count++;
	}
}

void forward_upto_leftright(void)
{
	count=0;
	while(1)
	{
		start_sensor_line_slow();

		motor_left_fwd(pwmki);
		motor_right_fwd(pwmka);
		
		//left 9 . . . 5 4 . . . 0 right
		//black=1 white=0
		if(((adc[2]==1 || adc[3]==1) && (adc[6]==1 || adc[7]==1)) && count>100)
		{
			lcd_led(0);delay_ms(20);lcd_led(1);break;
		}
		count++;
	}
}

void forward_upto_white(void)
{
	count=0;
	while(1)
	{
		start_sensor_line_slow();

		motor_left_fwd(pwmki);
		motor_right_fwd(pwmka);
		
		//left 9 . . . 5 4 . . . 0 right
		//black=1 white=0
		if(datasensor==bin_to_byte(0,0,0,0,0,0,0,0,0,0) && count>100)
		{
			lcd_led(0);delay_ms(20);lcd_led(1);break;
		}
		count++;
	}
}

void forward_upto_cm(uint32_t cm)
{
	reset_encoderLR();
	while(1)
	{
		start_sensor_line_fast();

		motor_left_fwd(pwmki); 
		motor_right_fwd(pwmka); 
		
		//encoder 52ppr
		//diameter roda = 35mm, round = 110mm = 11cm
		//1cm = ~4pulse
		
		encoderL=read_encoderL(); encoderR=read_encoderR();
		if((encoderL+encoderR)/2>cm*4)	
		{
			lcd_led(0);delay_ms(20);lcd_led(1);break;
		}
	}
}

void finish(void)
{
	buzzer(0);
	lcd_clear();
	lcd_gotoxy(0,0);
	lcd_putsf("FINISH");
	while(1)
	{
		motor_left_stop();motor_right_stop();
		lcd_led(0);delay_ms(100);
		buzzer(1);
		lcd_led(1);delay_ms(100);
	}
}

void pause(void)
{
	lcd_led(0);
	lcd_clear();
	lcd_gotoxy(0,0);
	lcd_putsf("PAUSE");
	while(1)
	{
		motor_left_stop();motor_right_stop();
		if(read_startbutton()==0) 
		{
			lcd_led(1);buzzer(0);
			delay_ms(100);
			buzzer(1);
			delay_ms(200);break;
		}
	}
	lcd_clear();
}

void pause_ms(uint32_t ms)
{
	int t=0;
	lcd_led(0);
	lcd_clear();
	lcd_gotoxy(0,0);
	lcd_putsf("PAUSE");
	while(1)
	{
		motor_left_stop();motor_right_stop();
		if(t>=ms) break;		
		delay_ms(1);
		t++;
	}
	lcd_clear();
}

void turn_left(void)
{
	lcd_led(0);
	motor_left_rvs(50);motor_right_fwd(50);
	delay_ms(50);
	while(1)
	{
		start_sensor_line_slow();
		if((adc[8]==1 && adc[7]==1) || (adc[7]==1 && adc[6]==1)) break;
	}
	lcd_led(1);
	motor_left_stop();motor_right_stop();
}

void turn_right(void)
{
	lcd_led(0);
	motor_left_fwd(50);motor_right_rvs(50);
	delay_ms(50);
	while(1)
	{
		start_sensor_line_slow();
		if((adc[1]==1 && adc[2]==1) || (adc[2]==1 && adc[3]==1)) break;
	}
	lcd_led(1);
	motor_left_stop();motor_right_stop();
}

void left(void)
{
	lcd_led(0);
	motor_left_stop();motor_right_fwd(50);
	delay_ms(50);
	while(1)
	{
		start_sensor_line_fast();
		if((adc[8]==1 && adc[7]==1) || (adc[7]==1 && adc[6]==1)) break;
	}
	lcd_led(1);
	motor_left_stop();motor_right_stop();
}

void right(void)
{
	lcd_led(0);
	motor_left_fwd(50);motor_right_stop();
	delay_ms(50);
	while(1)
	{
		start_sensor_line_fast();
		if((adc[1]==1 && adc[2]==1) || (adc[2]==1 && adc[3]==1)) break;
	}
	lcd_led(1);
	motor_left_stop();motor_right_stop();
}

void turn_enc_right(uint8_t deg)
{
	reset_encoderLR();
	lcd_led(0);
	motor_left_fwd(75);motor_right_rvs(75);
	delay_ms(50);
	while(1)
	{
		encoderL=read_encoderL(); encoderR=read_encoderR();
		if((encoderL+encoderR)/2>(deg/90)*52)			
		{
			delay_ms(20);break;
		}
	}
	lcd_led(1);
	motor_left_stop();motor_right_stop();
}

void turn_enc_left(uint8_t deg)
{
	reset_encoderLR();
	lcd_led(0);
	motor_left_rvs(75);motor_right_fwd(75);
	delay_ms(50);
	while(1)
	{
		encoderL=read_encoderL(); encoderR=read_encoderR();
		if((encoderL+encoderR)/2>(deg/90)*52)		
		{
			delay_ms(20);break;
		}
	}
	lcd_led(1);
	motor_left_stop();motor_right_stop();
}

void forward_enc_only_cm(uint32_t cm)
{
	reset_encoderLR();
	while(1)
	{
		motor_left_fwd(50); 
		motor_right_fwd(50); 
		
		//encoder 52ppr
		//diameter roda = 35mm, round = 110mm = 11cm
		//1cm = ~4pulse
		
		encoderL=read_encoderL(); encoderR=read_encoderR();
		if((encoderL+encoderR)/2>cm*4)	
		{
			lcd_led(0);delay_ms(20);lcd_led(1);break;
		}
	}
}

void turn_gy25_right(uint8_t deg)
{
	lcd_led(0);
	if(deg>180) deg=180;
	temp_gy = read_gy25_yaw(); gy_move=0;
	motor_left_fwd(50);motor_right_rvs(50);
	while(1)
	{
		yaw = read_gy25_yaw();
		if(temp_gy<180 && yaw>180 && gy_move==0) {temp_gy = temp_gy + 355; gy_move++;} 
		
		res_gy = temp_gy - yaw;		
		if(res_gy >= (deg-30))			
		{
			delay_ms(20);break;
		}
	}
	lcd_led(1);
	motor_left_stop();motor_right_stop();
}

void turn_gy25_left(uint8_t deg)
{
	lcd_led(0);
	if(deg>180) deg=180;
	temp_gy=read_gy25_yaw(); gy_move=0;
	motor_left_rvs(50);motor_right_fwd(50);
	while(1)
	{
		yaw = read_gy25_yaw();
		if(temp_gy>175 && yaw<175 && gy_move==0) {temp_gy = temp_gy - 355; gy_move++;} 
		
		res_gy = abs(yaw-temp_gy);
		if(res_gy >= (deg-30))			
		{
			delay_ms(20);break;
		}
	}
	lcd_led(1);
	motor_left_stop();motor_right_stop();
}

void calib_sensor_line(void)
{
	while(1)	//calibration
		{
			lcd_gotoxy(0,0);lcd_putsf("  CALIBRATION   ");
			lcd_gotoxy(0,1);lcd_putsf("black and white");
			calibration();
			if(!read_startbutton()) {lcd_clear();delay_ms(500);break;}
		}
		FEE_Init();
		FEE_Erase();
		for(i=0;i<10;i++)	//calculate set point and save to eeprom
		{
			ee[i] = ((white[i]-black[i])/2)+black[i]; 
			FEE_WriteDataByte(i,ee[i]);
		}
		lcd_clear();lcd_gotoxy(0,0);lcd_putsf("Calibration done");
		delay_ms(500);
		lcd_clear();
}

void read_eeprom(void)
{
	for(i=0;i<10;i++)
	{
		sp[i] = FEE_ReadDataByte(i);
	}
}

uint8_t read_adc_dig(uint8_t n)
{
	for(i=0;i<10;i++)
	{
		adc[i] = read_adc(i);
		if(adc[i]<sp[i]) adc[i]=1;
		else adc[i] = 0;
	}
	return adc[n];
}

