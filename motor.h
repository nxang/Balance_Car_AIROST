#include "esp32-hal.h"
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "main.h"
// #include "arduino.h"
#include "imu.h"
#include "ESP32Encoder.h"

//////////////////////PID parameters///////////////////////////////
double kp = 8.18, ki = 0.00, kd = -0.9;                   // angle loop parameter
double kp_speed = 2.757, ki_speed = 0.00, kd_speed = 0.0; // speed loop parameter

double kp_turn = 2.00, ki_turn = 0, kd_turn = 0.08; // steering loop parameter
double kp_pitch = 0.00;

double setp0 = 0; // angle balance point
int PD_pwm;       // angle output
float pwm1 = 0, pwm2 = 0;
// int motordead_val = 35;
int motordead_val = 28;

float previous_yaw = 0;
float yaw_speed;
float previous_pitch = 0;
float gyroy;
float gyroz;

int Flag_Left, Flag_Right, Flag_Front, Flag_Back;

int angle0 = 5.0; // Actual measured angle (ideally 0 degrees)

float speeds_filterold = 0;
float positions = 0;
int flag1;
double PI_pwm;
int cc;
int speedout;
float speeds_filter;

// Bluetooth//
int setspeed = 0;
int front = 0; // backward variable
int back = 0;  // forward variable
int left = 0;  // turn left
int right = 0; // turn right
float Turn_Amplitude = 0;
float Turn_speed = 0;
char val;

//////////////////////////////steering PD///////////////////
int turnmax, turnmin, turnout;
float Turn_pwm = 0;
int zz = 0;
int turncc = 0;

ESP32Encoder enc1;
ESP32Encoder enc2;

typedef struct
{
  uint8_t pinA;
  uint8_t pinB;
  uint8_t stateA;
  uint8_t stateB;
  uint8_t lastStateA;
  int64_t encoderPos;
} Enc;

volatile long count_right = 0; // Used to calculate the pulse value calculated by the Hall encoder (the volatile long type is to ensure that the value is valid）
volatile long count_left = 0;
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int pulseright, pulseleft;

// void CountPulse(Enc *encoder);
void motor_init();
void LeftEncoderHandler();
void RightEncoderHandler();
Enc Left_Enc;
Enc Right_Enc;

void motor_init()
{
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc1.attachFullQuad(L_ENC_A, L_ENC_B);
  delay(10);
  enc2.attachFullQuad(R_ENC_B, R_ENC_A);
}

void PD()
{
  PD_pwm = kp * (pitch + angle0) + kd * gyroy; // Use pitch and its rate of change
}

void speedpiout()
{
  static int64_t lastpulseleft = 0, lastpulseright = 0;

  pulseleft = enc1.getCount() - lastpulseleft;
  pulseright = enc2.getCount() - lastpulseright;
  lastpulseleft = enc1.getCount();
  lastpulseright = enc2.getCount();
  float speeds = (pulseleft + pulseright) * 2.0; // speed; pulse value
  speeds_filterold *= 0.7;                       // first-order complementary filtering
  speeds_filter = speeds_filterold + speeds * 0.3 + setp0;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions = constrain(positions, -2000, 2000); // Anti-integral saturation
  ki_speed = kp_speed / 200.0;

  PI_pwm = ki_speed * (0 - positions) + kp_speed * (0 - speeds_filter); // speed loop control PI
}

void turn() // 转向控制
{
  static float Turn_Target, Encoder_temp, Turn_Convert = 0.9, Turn_Count;
  float Kp = 2, Kd = 0.2;
  //=============遥控左右旋转部分=======================//
  // 这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
  if (1 == Flag_Left || 1 == Flag_Right)
  {
    if (++Turn_Count == 1)
      Encoder_temp = abs(pulseleft + pulseright) * 2.0;
    Turn_Convert = 55 / Encoder_temp;
    if (Turn_Convert < 0.6)
      Turn_Convert = 0.6;
    if (Turn_Convert > 3)
      Turn_Convert = 3;
  }
  else
  {
    Turn_Convert = 0.9;
    Turn_Count = 0;
    Encoder_temp = 0;
  }
  if (1 == Flag_Left)
    Turn_Target -= Turn_Convert;
  else if (1 == Flag_Right)
    Turn_Target += Turn_Convert;
  else
    Turn_Target = 0;
  if (Turn_Target > Turn_Amplitude)
    Turn_Target = Turn_Amplitude; //===转向	速度限幅
  if (Turn_Target < -Turn_Amplitude)
    Turn_Target = -Turn_Amplitude;
  // if(Flag_Front==1||Flag_Back==1)  Kd=0.2;
  if (1 == Flag_Left || 1 == Flag_Right)
    Kd = 0;
  else
    Kd = 0.2; // 转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  //=============转向PD控制器=======================//
  Turn_pwm = -Turn_Amplitude * Kp - gyroz * Kd; //===结合Z轴陀螺仪进行PD控制
}

void anglePWM()
{
  pwm2 = -PD_pwm - PI_pwm + Turn_pwm;
  pwm1 = -PD_pwm - PI_pwm - Turn_pwm;

  if (pwm1 > 0)
  {
    pwm1 += motordead_val;
  }
  else if (pwm1 < 0)
  {
    pwm1 -= motordead_val;
  }

  if (pwm2 > 0)
  {
    pwm2 += motordead_val;
  }
  else if (pwm2 < 0)
  {
    pwm2 -= motordead_val;
  }

  if (pwm1 > 255) // limit PWM value not more than 255
  {
    pwm1 = 255;
  }
  if (pwm1 < -255)
  {
    pwm1 = -255;
  }
  if (pwm2 > 255)
  {
    pwm2 = 255;
  }
  if (pwm2 < -255)
  {
    pwm2 = -255;
  }

  if (pwm2 >= 0) // determine the motor’s turning and speed by the negative and positive of PWM
  {
    analogWrite(L_IN1, 0);
    analogWrite(L_IN2, pwm2);
  }
  else
  {

    analogWrite(L_IN1, abs(pwm2));
    analogWrite(L_IN2, 0);
  }

  if (pwm1 >= 0)
  {
    analogWrite(R_IN1, pwm1);
    analogWrite(R_IN2, 0);
  }
  else
  {
    analogWrite(R_IN1, 0);

    analogWrite(R_IN2, abs(pwm1));
  }
}

#endif