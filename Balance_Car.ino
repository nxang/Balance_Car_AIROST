#include "main.h"
#include "index.h"


//sda 9
//scl 10
uint8_t follow_flag = 0;
uint8_t state = 0;
uint8_t pb;
float delta_time = 0.001;  // 5 milliseconds = 0.005 seconds
uint8_t L, LM, RM, R;
void setup() {

  Wire.begin(9, 10);  //Join the I2C bus sequence
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("There was an error initializing ESP-NOW");
    return;
  }

  // Set PMK key
  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);

  // Register the receiver board as peer
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = CHANNEL;
  // Set the receiver device LMK key
  for (uint8_t i = 0; i < 16; i++) {
    peerInfo.lmk[i] = LMK_KEY_STR[i];
  }

  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  delay(50);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      esp_restart();  // Trigger a hard reset
    }
  }
  delay(100);
  pixy1.init(0x56, Wire, 9, 10);
  pixy1.changeProg("line");

  delay(100);

  pinMode(F_RGB, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  pinMode(PB1, INPUT_PULLUP);  // set the pulse pin to INPUT
  pinMode(PB2, INPUT_PULLUP);  // set the pulse pin to INPUT
  pinMode(PB3, INPUT_PULLUP);  // set the pulse pin to INPUT
  pinMode(PB4, INPUT_PULLUP);  // set the pulse pin to INPUT

  pinMode(L_ENC_A, INPUT);  // set the pulse pin to INPUT
  pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT);  // set the pulse pin to INPUT
  pinMode(R_ENC_B, INPUT);

  motor_init();

  Serial.print("nope ");

  xTaskCreate(
    Cam,
    "Cam Task",
    20480,
    NULL,
    4,
    NULL);

  xTaskCreate(
    Motor,
    "Motor Task",
    20480,
    NULL,
    3,
    NULL);


  xTaskCreate(
    IMU,
    "IMU Task",
    20480,
    NULL,
    2,
    NULL);

  xTaskCreate(
    Main,
    "Main Task",
    2048,
    NULL,
    1,
    NULL);
}

void loop() {
}


float follow_lasterror = 0;
void Motor(void *pvParameters) {
  while (1) {
    digitalWrite(LED1, follow_flag);
    if (follow_flag == 1) {
      if (cam_flag == 0) {
        setp0 = 4.0;
      } else {
        setp0 = 0.0;
      }
      Turn_speed = cam_angle * -0.12 + (cam_angle - follow_lasterror) * -0.05;
      follow_lasterror = cam_angle;
    } else {
      setp0 = joyL_y * 15.0;
      Turn_speed = (-joyL_2 + joyR_2) * 30.0 - joyL_x * 20.0;
    }

    if (setp0 > 0) {
      Flag_Front = 1;
      Flag_Back = 0;
    } else if (setp0 < 0) {
      Flag_Front = 0;
      Flag_Back = 1;
    } else {
      Flag_Front = 0;
      Flag_Back = 0;
    }

    if (Turn_speed < 0) {
      Turn_Amplitude = Turn_speed;
      Flag_Left = 1;
      Flag_Right = 0;
    } else if (Turn_speed > 0) {
      Turn_Amplitude = Turn_speed;
      Flag_Left = 0;
      Flag_Right = 1;
    } else {
      Turn_Amplitude = 0;
      Flag_Left = 0;
      Flag_Right = 0;
    }

    if (abs(pitch) < 40) {
      PD();  // PD control of angle loop
      speedpiout();
      turn();
      anglePWM();
    } else {
      analogWrite(L_IN1, 0);
      analogWrite(L_IN2, 0);
      analogWrite(R_IN1, 0);
      analogWrite(R_IN2, 0);
    }
    vTaskDelay(5);
  }
}

void IMU(void *pvParameters) {
  while (1) {
    if (mpu.update()) {
      yaw = mpu.getYaw();      // Z-axis rotation
      pitch = mpu.getPitch();  // X-axis rotation
      roll = mpu.getRoll();    // Y-axis rotation
      gyroy = mpu.getGyroY();
      gyroz = mpu.getGyroZ();
    }
  
       Serial.print(Turn_speed);
      Serial.print(",");
    Serial.print(cam_angle);
    Serial.print(",");
    Serial.print(kp, 4);
    Serial.print(",");
    Serial.print(kd, 5);
    Serial.print(",");
    Serial.println(kp_speed, 4);
    vTaskDelay(1);
  }
}

void Main(void *pvParameters) {

  while (1) {
    switch (ps4data) {
      case UP:
        kp += 0.02;
        while (ps4data == UP) {
          vTaskDelay(5);

        };  // Wait until the button is released
        break;

      case DOWN:
        kp -= 0.02;
        while (ps4data == DOWN) {
          vTaskDelay(5);
        };  // Wait until the button is released

        break;

      case LEFT:
        // Serial.println("left");
        kd -= 0.005;
        // Flag_Left = 1;
        while (ps4data == LEFT) {
          vTaskDelay(5);
        };  // Wait until the button is released
        break;

      case RIGHT:
        kd += 0.005;
        while (ps4data == RIGHT) {
          vTaskDelay(5);
        };  // Wait until the button is released
        // Flag_Right = 0;
        break;

      case TRIANGLE:
        kp_speed += 0.1;
        while (ps4data == TRIANGLE) {
          vTaskDelay(5);
        };  // Wait until the button is released
        break;

      case SQUARE:
        kp_speed -= 0.001;
        while (ps4data == SQUARE) {
          vTaskDelay(5);
        };  // Wait until the button is released
        break;


      case CIRCLE:
        kp_speed += 0.001;
        while (ps4data == CIRCLE) {
          vTaskDelay(5);
        };  // Wait until the button is released
        break;

      case CROSS:
        kp_speed -= 0.1;
        while (ps4data == CROSS) {
          vTaskDelay(5);
        };  // Wait until the button is released
        break;

      case R1:
        follow_flag = !follow_flag;
        while (ps4data == R1) {
          vTaskDelay(5);
        };  // Wait until the button is released
        break;

      default:
        break;
    }
    vTaskDelay(5);
  }
}

void Cam(void *pvParameters) {
  while (1) {
    Cam(pixy1);
    vTaskDelay(5);

  }
}