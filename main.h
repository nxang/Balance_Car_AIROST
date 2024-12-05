#ifndef __MAIN_H
#define __MAIN_H

#define PB1 21
#define PB2 47

#define L_ENC_B 35
#define L_ENC_A 36
#define L_IN2 37
#define L_IN1 38  
#define R_IN2 39
#define R_IN1 40  
#define R_ENC_B 41
#define R_ENC_A 42  

#define PB3 2
#define PB4 1
#define left_L2 6
#define PWM_L 9

#define I2C_SCL 10
#define I2C_SDA 9
#define F_RGB 3
#define LED1 20
#define LED2 19
#define TX 8
#define RX 18
#define BUZZER 17

#define G16 16
#define G15 15

#define SIGNAL1 12
#define SIGNAL2 11
#define SIGNAL3 7
#define SIGNAL4 6

#define CHANNEL 1



void IMU(void *pvParameters);
void Main(void *pvParameters);

#include <esp_now.h>
#include <WiFi.h>
#include "ps4.h"
#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "motor.h"
#include "cam.h"


#endif /* __MAIN_H */
