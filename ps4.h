#ifndef PS4_H
#define PS4_H

#pragma once

#include <stdint.h>
#include <stdint.h>
#include "motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define T_SHARE 0
#define T_L3 1
#define T_R3 2
#define T_UP 4
#define T_RIGHT 5
#define T_DOWN 6
#define T_LEFT 7
#define T_L2 8
#define T_R2 9
#define T_L1 10
#define T_R1 11
#define T_TRIANGLE 12
#define T_CIRCLE 13
#define T_CROSS 14
#define T_SQUARE 15
#define T_OPTION 0
#define T_TOUCH 1

  enum DPADEnum
  {
    DPAD_UP = 0x0,
    DPAD_UP_RIGHT = 0x1,
    DPAD_RIGHT = 0x2,
    DPAD_RIGHT_DOWN = 0x3,
    DPAD_DOWN = 0x4,
    DPAD_DOWN_LEFT = 0x5,
    DPAD_LEFT = 0x6,
    DPAD_LEFT_UP = 0x7,
    DPAD_OFF = 0x8,
  };

  enum ButtonEnum
  {
    /*@{/
        /** Directional Pad Buttons - available on most controllers */
    UP = 0,
    RIGHT = 1,
    DOWN = 2,
    LEFT = 3,
    /*@}/

        /*@{/
        /** Playstation buttons */
    TRIANGLE,
    CIRCLE,
    CROSS,
    SQUARE,

    SELECT,
    START,

    L3,
    R3,
  };

  typedef union
  {
    unsigned int picdata;
    struct
    {
      unsigned char buff1;
      unsigned char buff2;
    };
  } bits;
  bits bits1, bits2;

  volatile unsigned int leftjoy_x;
  volatile unsigned int leftjoy_y;
  volatile unsigned int rightjoy_x;
  volatile unsigned int rightjoy_y;

  volatile unsigned int an_L2;
  volatile unsigned int an_R2;

  volatile float joyR_y;
  volatile float joyR_x;
  volatile float joyL_y;
  volatile float joyL_x;
  volatile float joyR_2;
  volatile float joyL_2;

  static const char *PMK_KEY_STR = "pmk1234567890123";
  static const char *LMK_KEY_STR = "lmk1234567890123";

  uint8_t receiverAddress[] = {0xEC, 0xDA, 0x3B, 0x67, 0x72, 0x88};
  unsigned int ps4datatemp, ps4data;
  unsigned char ps4data1;
  ps4_msg_t *buf;
  esp_now_peer_info_t peerInfo;
  boolean toggle = false; // Initialize to false (0)
  uint8_t count = 0;

  bool checkDpad(ButtonEnum b)
  {
    switch (b)
    {
    case UP:
      return buf->buttons_1.dpad == DPAD_LEFT_UP || buf->buttons_1.dpad == DPAD_UP || buf->buttons_1.dpad == DPAD_UP_RIGHT;
    case RIGHT:
      return buf->buttons_1.dpad == DPAD_UP_RIGHT || buf->buttons_1.dpad == DPAD_RIGHT || buf->buttons_1.dpad == DPAD_RIGHT_DOWN;
    case DOWN:
      return buf->buttons_1.dpad == DPAD_RIGHT_DOWN || buf->buttons_1.dpad == DPAD_DOWN || buf->buttons_1.dpad == DPAD_DOWN_LEFT;
    case LEFT:
      return buf->buttons_1.dpad == DPAD_DOWN_LEFT || buf->buttons_1.dpad == DPAD_LEFT || buf->buttons_1.dpad == DPAD_LEFT_UP;
    default:
      return false;
    }
  }

  void OnDataRecv(const esp_now_recv_info_t *mac_addr, const uint8_t *incomingData, int len)
  {
    toggle = !toggle;
    digitalWrite(LED1, toggle);

    buf = (ps4_msg_t *)incomingData;
    // Serial.println( buf->buttons_1.val);
    ps4datatemp = 0;
    ps4datatemp |= ((buf->buttons_2.share << T_SHARE) | (buf->buttons_2.L3 << T_L3) | (buf->buttons_2.R3 << T_R3) | (checkDpad(UP) << T_UP) | (checkDpad(RIGHT) << T_RIGHT) | (checkDpad(DOWN) << T_DOWN) | (checkDpad(LEFT) << T_LEFT) | (0 << T_L2) | (0 << T_R2) | (buf->buttons_2.L1 << T_L1) | (buf->buttons_2.R1 << T_R1) | (buf->buttons_1.triangle << T_TRIANGLE) | (buf->buttons_1.circle << T_CIRCLE) | (buf->buttons_1.cross << T_CROSS) | (buf->buttons_1.square << T_SQUARE));
    ps4data = ps4datatemp;
    leftjoy_x = buf->left_joy_x;
    leftjoy_y = buf->left_joy_y;
    rightjoy_x = buf->right_joy_x;
    rightjoy_y = buf->right_joy_y;
    an_L2 = buf->left_trigger;
    an_R2 = buf->right_trigger;
    ps4data1 = 0;
    ps4data1 |= ((buf->buttons_2.option << T_OPTION) | (buf->buttons_3.Tpad << T_TOUCH));
  }

#ifdef __cplusplus
}
#endif //__cplusplus
#endif