#ifndef IMU_H
#define IMU_H

#include <MPU6050.h>      //MPU6050 library
#include <Wire.h>        //IIC communication library
#include "MPU9250.h"

MPU9250 mpu;
float yaw;
float pitch;
float roll;

int16_t ax, ay, az, gx, gy, gz;     //Define three-axis acceleration, three-axis gyroscope variables
float Angle;
float Gyro_x,Gyro_y,Gyro_z;  //calculate angular velocity of each axis by gyroscope 

///////////////////////Kalman_Filter////////////////////////////
float Q_angle = 0.1;  //Covariance of gyroscope noise
float Q_gyro = 0.02;    //Covariance of gyroscope drift noise
float R_angle = 0.5;    //Covariance of accelerometer

char C_0 = 1;
float dt = 0.001; //The value of dt is the filter sampling time.
float K1 = 0.05; // a function containing the Kalman gain is used to calculate the deviation of the optimal estimate.
float K_0,K_1,t_0,t_1;
float angle_err;
float q_bias;    //gyroscope drift 

float accelz = 0;
float angle;
float angle_speed;

float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1, E;


void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;          //Prior estimate
  angle_err = angle_m - angle;
  
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    //Differential of azimuth error covariance
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  
  P[0][0] += Pdot[0] * dt;    //The integral of the covariance differential of the prior estimate error
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  
  //Intermediate variable of matrix multiplication
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  //Denominator
  E = R_angle + C_0 * PCt_0;
  //Gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;  //Intermediate variable of matrix multiplication
  t_1 = C_0 * P[0][1];
  
  P[0][0] -= K_0 * t_0;    //Posterior estimation error covariance 
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  q_bias += K_1 * angle_err;    //Posterior estimation
  angle_speed = gyro_m - q_bias;   //The differential value of the output value; work out the optimal angular velocity
  angle += K_0 * angle_err; ////Posterior estimation; work out the optimal angle
}


void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1)
{
  Angle = -atan2(ay , az) * (180/ PI);           //Radial rotation angle calculation formula; negative sign is direction processing
  Gyro_x = -gx / 131;              //The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Kalman_Filter(Angle, Gyro_x);            //KalmanFilter 
}

#endif