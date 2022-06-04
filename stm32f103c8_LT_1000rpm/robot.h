#ifndef __ROBOT_H
#define __ROBOT_H

#include "hardwareconfig.h"

void DRIVERMOTOR_INIT(void);
void BUTTON_INIT(void);
void LCDLED_INIT(void);
void BUZZER_INIT(void);

uint8_t read_startbutton(void);
uint8_t read_combutton(void);
void lcd_led(uint8_t n);
void buzzer(uint8_t n);

void motor_right_rvs(uint8_t n);
void motor_right_fwd(uint8_t n);
void motor_right_stop(void);
void motor_left_rvs(uint8_t n);
void motor_left_fwd(uint8_t n);
void motor_left_stop(void);

void ROBOT_INIT(void);
#endif