#include <sys/_stdint.h>
#include <Pixy2I2C.h>
#include "MA.h"
#include <stdint.h>

typedef struct Pixy_Position {
  uint8_t x_chosen, y_chosen;  //used for invert vector detection and selection
  uint8_t x_out, y_out;        //will be sent to mainboard
} Pixy_Position_t;

float cam_angle;
float prev_angle = 0;
float real_angle = 0;

uint8_t cam_count, cam_flag;
int constant;
uint8_t ending_x;
uint8_t ending_y;
uint8_t starting_x;
uint8_t starting_y;
uint8_t X1;
uint8_t X2;
uint8_t Y1;
uint8_t Y2;

enum pixy_cam2_instruction_list {  //copy this into mainboard code (in pixy_cam2_ESP32.h)
  PIXY_0x54_GET_VECTOR = 1,
  PIXY_0x55_GET_VECTOR,
  PIXY_0x56_GET_VECTOR,
  PIXY_0x57_GET_VECTOR,
  PIXY_ALL_GET_VECTOR,

  PIXY_0x54_TOGGLE_LAMP,
  PIXY_0x55_TOGGLE_LAMP,
  PIXY_0x56_TOGGLE_LAMP,
  PIXY_0x57_TOGGLE_LAMP,
  PIXY_ALL_TOGGLE_LAMP,
} pixy_instruction;

Pixy2I2C pixy1;
// Moving_Average_c Mov1, Mov2, Mov3, Mov4;
// Pixy_Position_t pos1, pos2, pos3, pos4;
uint8_t cam_output;
MovingAverage MA1;
MovingAverage MA2;
MovingAverage MA3;
MovingAverage MA4;
MovingAverage MA5;



void Cam(Pixy2I2C &pixy) {
  int8_t i;
  char buf[128];
  uint8_t mid_x;
  uint8_t mid_y;

  pixy1.line.getAllFeatures();
  Serial.print("cam");

  // print all vectors
  if (pixy.line.numVectors) {
    for (i = 0; i < pixy.line.numVectors; i++) {
      if (pixy.line.vectors[i].m_flags == 0 && i == 0) {
        X1 = MA1.calculate(pixy.line.vectors[i].m_x0);
        Y1 = MA2.calculate(pixy.line.vectors[i].m_y0);
        X2 = MA3.calculate(pixy.line.vectors[i].m_x1);
        Y2 = MA4.calculate(pixy.line.vectors[i].m_y1);
        if (Y2 > Y1) {
          starting_x = X2;
          starting_y = Y2;
          ending_x = X1;
          ending_y = Y1;
        } else {
          starting_x = X1;
          starting_y = Y1;
          ending_x = X2;
          ending_y = Y2;
        }
        float angle_xrad = atan2((-ending_x + 39), (-ending_y + 51));
        float angle_x = constrain(degrees(angle_xrad), -50, 50);
        cam_angle = MA5.calculate(angle_x);

        mid_x = (starting_x + ending_x) / 2;
        mid_y = (starting_y + ending_y) / 2;
        cam_count=0;
        // cam_flag=0;
        // printf("\n angle: %f", angle_x);

        // Serial.println();
        // sprintf(buf, "line %d: ", i);
        // Serial.print(buf);
        // pixy.line.vectors[i].print();
      }
    }
  }
  else{
    cam_count++;
  }

  if(cam_count>=15){
    cam_flag=1;
  }
  else{
        cam_flag=0;
  }
}