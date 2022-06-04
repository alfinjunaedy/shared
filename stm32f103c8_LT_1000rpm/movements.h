#ifndef __MOVEMENTS_H
#define __MOVEMENTS_H

#include "hardwareconfig.h"
#include "robot.h"

void calibration(void);
void start_sensor_line_fast(void);
void start_sensor_line_slow(void);

void forward_upto_left(void);
void forward_upto_right(void);
void forward_upto_leftright(void);
void forward_upto_white(void);
void forward_upto_cm(uint32_t cm);

void finish(void);
void pause(void);
void pause_ms(uint32_t ms);

void turn_left(void);
void turn_right(void);
void left(void);
void right(void);

void turn_enc_right(uint8_t deg);
void turn_enc_left(uint8_t deg);
void forward_enc_only_cm(uint32_t cm);

void turn_gy25_right(uint8_t deg);
void turn_gy25_left(uint8_t deg);

void calib_sensor_line(void);
void read_eeprom(void);
uint8_t read_adc_dig(uint8_t n);

#endif