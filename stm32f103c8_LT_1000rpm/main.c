/*====================================DAFTAR PERGERAKAN YANG BISA DIGUNAKAN=====================================================

forward_upto_left();							= maju ikuti garis sampai sensor kiri deteksi garis
forward_upto_right();							= maju ikuti garis sampai sensor kanan deteksi garis
forward_upto_leftright();					= maju ikuti garis sampai sensor kiri dan kanan deteksi garis
forward_upto_white();							= maju ikuti garis sampai semua sensor tidak mendeteksi garis
forward_upto_cm(10);							= maju ikuti garis sampai jarak tertentu (cm)

finish();													= berhenti total
pause();													= berhenti sampai tombol start ditekan
pause_ms(1000);										= berhenti dengan jeda waktu tertentu (ms)

turn_left();											= putar kiri dengan poros tengah
turn_right();											= putar kanan dengan poros tengah
left();														= putar kiri dengan poros roda kiri
right();													= putar kanan dengan poros roda kanan

turn_enc_right(90);								= putar kanan dengan poros tengah hingga sudut tertentu dari encoder (deg)
turn_enc_left(90);								= putar kiri dengan poros tengah hingga sudut tertentu dari encoder (deg)
forward_enc_only_cm(10);					= maju tanpa pengaruh garis hingga jarak tertentu (cm)

turn_gy25_right(90);							= putar kanan dengan poros tengah hingga sudut tertentu dari gy25 (deg)
turn_gy25_left(90);								= putar kiri dengan poros tengah hingga sudut tertentu dari gy25 (deg)

==============================================================================================================================*/

#include "stm32f10x.h"                  
#include "hardwareconfig.h"
#include "robot.h"
#include "movements.h"

unsigned char buf[16];

int main(void)
{
	HARDWARECONFIG_INIT();
	ROBOT_INIT();

	//line sensor calibration
	while(!read_combutton())
	{
		buzzer(0);
		lcd_gotoxy(0,0);
		lcd_putsf("StartCalibration");
		delay_ms(500);
		buzzer(1);
		lcd_clear();
		
		calib_sensor_line();
	}
	
	//read memory
	read_eeprom();
	
	//opening
	lcd_gotoxy(0,0);lcd_putsf("Jangan Lupa Doa");
	buzzer(0);lcd_led(0);delay_ms(50);
	buzzer(1);lcd_led(1);delay_ms(50);
	buzzer(0);lcd_led(0);delay_ms(50);
	buzzer(1);lcd_led(1);delay_ms(50);
	lcd_led(0);
	delay_ms(1000);lcd_clear();
	
	//waiting to be started
	while(read_startbutton())
	{		
		lcd_gotoxy(0,0);
		sprintf(buf,"  %d%d%d%d%d %d%d%d%d%d   ",read_adc_dig(9),read_adc_dig(8),read_adc_dig(7),read_adc_dig(6),read_adc_dig(5),read_adc_dig(4),read_adc_dig(3),read_adc_dig(2),read_adc_dig(1),read_adc_dig(0));
		lcd_putsf(buf);
		
		lcd_gotoxy(2,1);
		sprintf(buf,"%d",read_encoderL());
		lcd_putsf(buf);
		lcd_gotoxy(8,1);
		sprintf(buf,"%d",read_encoderR());
		lcd_putsf(buf);
		
		lcd_gotoxy(13,1);
		sprintf(buf,"%d",read_gy25_yaw());
		lcd_putsf(buf);
		
		delay_ms(100);
		lcd_clear();
	}
	
	buzzer(0);
	lcd_clear();
	lcd_led(1);
	delay_ms(300);
	buzzer(1);
			
	while(1)
	{
		//main program, write your code here
		forward_upto_white();
		
		finish();
		//end of main program, don't forget to put "finish()" to stop the robot
	}
}