/**
* Author: Yunpeng Song <413740951@qq.com>
* Author: Ye Tian <flymaxty@foxmail.com>
* Copyright (c) 2016 Maker Collider Corporation.
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*/

#include <Arduino.h>

#include "xpider_pin.h"
#include "xpider_control.h"

// #define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTF(x, y) Serial.print(x, y)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTF(x, y)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTLNF(x, y)
#endif

#define OPERATION_TIMEOUT 1500

XpiderControl* XpiderControl::instance_ = NULL;

XpiderControl::XpiderControl() {
  voltage_sum_ = 0;
  distance_sum_ = 0;
  step_counter_ = 0;
  voltage_counter_ = 0;
  distance_counter_ = 0;
  walk_status_ = 0;
  rotate_status_ = 0;
}

XpiderControl::~XpiderControl() {
  
}

XpiderControl* XpiderControl::instance() {
  if(instance_ == NULL) {
    instance_ = new XpiderControl();
  }
  return instance_;
}

void XpiderControl::Initialize() {
  front_leds_ = new Adafruit_NeoPixel(2, LED_FRONT, NEO_GRB + NEO_KHZ800);
  front_leds_->begin();
  front_leds_->clear();
  front_leds_->show();

  pinMode(MOTOR_F_1, OUTPUT);
  pinMode(MOTOR_F_2, OUTPUT);

  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(ROTATE_HALL, INPUT);
  InitializeRotate();

  pinMode(IR_DIST, INPUT);
  InitializeObstacleDistance();

  pinMode(SOUND_SENSOR, INPUT);

  pinMode(CAMERA_1, OUTPUT);
  pinMode(CAMERA_2, OUTPUT);
  pinMode(CAMERA_READY, INPUT);
  pinMode(CAMERA_ENCODER, INPUT);
  InitializeCameraAngle();

  pinMode(BATTERY_VOLT, INPUT);
  InitializeVoltageBuffer();

  // Initialize hall sensor 
  pinMode(STEP_HALL_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(STEP_HALL_INT), StepCounterHandler, FALLING);

  StopWalking();
  StopRotating();
}

void XpiderControl::StepCounterHandler() {
  static unsigned long s_delay = 0;

  if(instance_->stop_flag == true) {
    instance_->BrakeWalking();
    instance_->stop_flag = false;
  }

  if(millis() - s_delay > 50) {
    instance_->step_counter_ += 1;
    s_delay = millis();
  }
}

void XpiderControl::StopAll() {
  StopEye();
  StopWalking();
  StopRotating();
  ClearFrontLeds();
}

void XpiderControl::BrakeAll() {
  BrakeEye();
  BrakeWalking();
  BrakeRotating();
  ClearFrontLeds();
}

void XpiderControl::Step(const int8_t &count_speed, const uint8_t &count){
  uint16_t start = 0;  
  start = step_counter();
  while(step_counter()-start<count) {
    Walk(count_speed);
  }
  StopWalkOnHigh();
}

void XpiderControl::Walk(const int8_t &speed) {
  int8_t temp = constrain(speed, -100, 100);
  int v = map(abs(temp), 0, 100, 255, 0);

  stop_flag = false;

  if(v > 255) {
    DEBUG_PRINTLN(v);
    return;
  }
  if(temp > 0) {
    walk_status_ = 1;
    digitalWrite(MOTOR_F_1, 1);
    analogWrite(MOTOR_F_2, v);
  }else if(temp < 0) {
    walk_status_ = -1;
    digitalWrite(MOTOR_F_2, 1);
    analogWrite(MOTOR_F_1, v);
  } else {
    StopWalking();
  }

  DEBUG_PRINT("walk speed: ");
  DEBUG_PRINTLN(speed);
}

void XpiderControl::Rotate(const int8_t &speed) {
  int8_t temp = constrain(speed, -100, 100);
  int v = map(abs(temp), 0, 100, 255, 0);

  if(v > 255) {
    DEBUG_PRINTLN(v);
    return;
  }
  if(temp > 0) {
    rotate_status_ = 1;
    digitalWrite(MOTOR_R_2, 1);
    analogWrite(MOTOR_R_1, v);
  }else if(temp < 0) {
    rotate_status_ = -1;
    digitalWrite(MOTOR_R_1, 1);
    analogWrite(MOTOR_R_2, v);
  } else {
    StopRotating();
  }

  DEBUG_PRINT("rotate in speed: ");
  DEBUG_PRINTLN(speed);
}

/*
 * ^ CAMERA_MAX_ANGLE (camera_min_value_)
 * |
 * | 
 * + - - - > CAMERA_MIN_ANGLE (camera_max_value_)
 * 
 * when encoder is on camera_min_value_,
 * the head is on CAMERA_MAX_ANGLE
 */
bool XpiderControl::SetCameraAngle(const uint8_t &angle) {
  bool success = false;
  int target_value;
  unsigned long time_start = millis();

  uint8_t t_angle = constrain(angle, CAMERA_MIN_ANGLE, CAMERA_MAX_ANGLE);

  target_value = map(t_angle, CAMERA_MAX_ANGLE, CAMERA_MIN_ANGLE, camera_min_value_, camera_max_value_);
  while(millis()-time_start < 1000) {
    if(GetCameraAngle() < target_value-CAMERA_ENCODER_MIN_ERROR) {
      MoveEye(90);
    } else if(GetCameraAngle() > target_value+CAMERA_ENCODER_MIN_ERROR) {
      MoveEye(-90);
    } else {
      success = true;
      break;
    }
  }

  BrakeEye();

  DEBUG_PRINT("angle: ");
  DEBUG_PRINT(angle);
  DEBUG_PRINT(", target_value: ");
  DEBUG_PRINT(target_value);
  DEBUG_PRINT(", ");
  DEBUG_PRINTLN(GetCameraAngle());

  return success;
}

void XpiderControl::StopWalkOnHigh() {
  unsigned long start_time = millis();

  if(walk_status_ != 0) {
    stop_flag = true;
    while(stop_flag == true && (millis()-start_time)<OPERATION_TIMEOUT) {
      delay(1);
    }

    stop_flag = false;

    /* 
     * TODO: may cause delay on different task
     */
    delay(40);

    StopWalking();
  }
}

void XpiderControl::StopWalking() {
  digitalWrite(MOTOR_F_1, 0);
  digitalWrite(MOTOR_F_2, 0);

  walk_status_ = 0;

  DEBUG_PRINTLN("walk stop");
}

void XpiderControl::StopRotating() {  
  digitalWrite(MOTOR_R_1, 0);
  digitalWrite(MOTOR_R_2, 0);

  rotate_status_ = 0;

  DEBUG_PRINTLN("rotate stop");
}

void XpiderControl::StopEye() {  
  digitalWrite(CAMERA_1, 0);
  digitalWrite(CAMERA_2, 0);

  DEBUG_PRINTLN("camera stop");
}

void XpiderControl::BrakeWalking() {
  digitalWrite(MOTOR_F_1, 1);
  digitalWrite(MOTOR_F_2, 1);
  
  walk_status_ = 0;

  DEBUG_PRINTLN("walk brake");
}

void XpiderControl::BrakeRotating() {
  digitalWrite(MOTOR_R_1, 1);
  digitalWrite(MOTOR_R_2, 1);

  rotate_status_ = 0;

  DEBUG_PRINTLN("rotate brake");
}

void XpiderControl::BrakeEye() {  
  digitalWrite(CAMERA_1, 1);
  digitalWrite(CAMERA_2, 1);

  DEBUG_PRINTLN("camera brake");
}

uint16_t XpiderControl::GetCameraAngle() {
  return analogRead(CAMERA_ENCODER);
}

void XpiderControl::MoveEye(int8_t speed) {
  int8_t temp = constrain(speed, -100, 100);
  int v = map(abs(temp), 0, 100, 255, 0);

  if(v > 255) {
    DEBUG_PRINTLN(v);
    return;
  }
  if(temp > 0) {
    digitalWrite(CAMERA_1, 1);
    analogWrite(CAMERA_2, v);
  }else if(temp < 0) {
    digitalWrite(CAMERA_2, 1);
    analogWrite(CAMERA_1, v);
  } else {
    StopEye();
  }

  DEBUG_PRINT("camera move in speed: ");
  DEBUG_PRINTLN(speed);
}

bool XpiderControl::InitializeCameraAngle() {
  MoveEye(-70);
  delay(600);
  camera_min_value_ = GetCameraAngle() + CAMERA_VALUE_OFFSET;
  MoveEye(70);
  delay(600);
  camera_max_value_ = GetCameraAngle() - CAMERA_VALUE_OFFSET;
  SetCameraAngle(45);
  return true;
}

bool XpiderControl::InitializeRotate() {
  bool is_success = false;
  unsigned long start_time = millis();

  Rotate(65);
  delay(500);

  while(millis()-start_time < 3000) {
    if(digitalRead(ROTATE_HALL) == 0) {
      is_success = true;
      break;
    }
  }

  StopRotating();
  return is_success;
}

void XpiderControl::ClearFrontLeds() {
  front_leds_->clear();
}

uint8_t XpiderControl::GetMicSoundLevel() {
  return map(analogRead(SOUND_SENSOR), 0, 1024, 0, 255);
}

void XpiderControl::InitializeVoltageBuffer() {
  // 电压平均值初始化
  for(uint8_t i=0; i<20; i++) {
    voltage_buffer_[i] = analogRead(BATTERY_VOLT);
    voltage_sum_ += voltage_buffer_[i];
  }

  voltage_buffer_initialized_ = true;
}

float XpiderControl::GetVoltage() {
  voltage_sum_ -= voltage_buffer_[voltage_counter_];
  voltage_buffer_[voltage_counter_] = analogRead(BATTERY_VOLT);
  voltage_sum_ += voltage_buffer_[voltage_counter_];

  float voltage = map(voltage_sum_ / 20.0f , 0, 1024, 0, IO_REFERENCE*100) /100.0;

  // 如果电压低于阈值，则停止活动
  if(voltage < BATTERY_THRESHOLD) {
    is_low_battery_ = true;
  }

  voltage_counter_ = voltage_counter_>=19 ? 0 : voltage_counter_ + 1;

  return voltage;
}

void XpiderControl::InitializeObstacleDistance() {
  int analog_value;
  int distance, voltage_timed;

  // 平均值初始化
  /* TODO: use define or const value to provide buffer size */
  for(uint8_t i=0; i<10; i++) {
    analog_value = analogRead(IR_DIST);
    voltage_timed = map(analog_value, 0, 1023, 0, IO_REFERENCE*100);
    if(voltage_timed < 50 || voltage_timed > 220) {
      distance = 500;
    } else {
      distance = map(voltage_timed, 55, 220, 500, 40); 
    }

    distance_buffer_[i] = distance;
    distance_sum_ += distance_buffer_[i];
  }

  DEBUG_PRINT("init dist: ");
  DEBUG_PRINTLN(distance_sum_);

  distance_buffer_initialized_ = true;
}

uint16_t XpiderControl::GetObstacleDistance() {
  int analog_value;
  int distance, voltage_timed;
  
  analog_value = analogRead(IR_DIST);
  voltage_timed = map(analog_value, 0, 1023, 0, IO_REFERENCE*100);
  if(voltage_timed < 50 || voltage_timed > 220) {
    distance = 500;
  } else {
    distance = map(voltage_timed, 55, 220, 500, 40); 
  }

  distance_sum_ -= distance_buffer_[distance_counter_];
  distance_buffer_[distance_counter_] = distance;
  distance_sum_ += distance_buffer_[distance_counter_];
  distance_counter_ = distance_counter_>=9 ? 0 : distance_counter_ + 1;

  // for(int i=0; i<10;i++) {
  //   DEBUG_PRINT(distance_buffer_[i]);
  //   DEBUG_PRINT(", ");
  // }
  // DEBUG_PRINTLN("");

  DEBUG_PRINT("voltage_timed: ");
  DEBUG_PRINT(voltage_timed);
  DEBUG_PRINT(", Get distance: ");
  DEBUG_PRINT(distance);
  DEBUG_PRINT(", avg: ");
  DEBUG_PRINT(distance_sum_/10);
  DEBUG_PRINT(", size: ");
  DEBUG_PRINTLN(sizeof(distance_sum_));

  return static_cast<uint16_t>(distance_sum_/10);
}

void XpiderControl::SetFrontLeds(const uint8_t leds[6]) {  
  front_leds_->clear();
  front_leds_->setPixelColor(0, leds[0], leds[1], leds[2]);
  front_leds_->setPixelColor(1, leds[3], leds[4], leds[5]);
  front_leds_->show();

  DEBUG_PRINTLN("set led");
}
