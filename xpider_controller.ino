/**
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

#include "Arduhdlc.h"

#include "xpider_task.h"
#include "xpider_pin.h"
#include "xpider_imu.h"
#include "xpider_control.h"
#include "xpider_inside_protocol.h"

#define CONTROLLER_VERSION "3.1.0"

Arduhdlc *g_hdlc;
XpiderIMU *g_xpider_imu;
XpiderControl *g_xpider_control;
XpiderInsideProtocol g_xpider_inside_protocol;

void serialEvent() {
  while(Serial.available()) {
    g_hdlc->charReceiver(Serial.read());
  }
}

void frameHandler(const uint8_t *data, uint16_t length) {
  g_xpider_inside_protocol.Decode(data, length);
}

void sendframe(const uint8_t *buffer, uint16_t length) {
  Serial.write(buffer, length);
}

void InsideSendData(const uint8_t* buffer, uint8_t length) {
  g_hdlc->frameDecode(buffer, length);
}

void SetMove(int8_t speed) {
  KillAutoMove();
  g_xpider_control->Walk(speed);
}

void SetStep(int8_t count_speed, uint8_t count) {
  KillAutoMove();
  g_xpider_control->Step(count_speed, count);
}

void SetAutoMove(uint8_t rotate_speed, float rotate_rad, uint8_t walk_speed, int8_t walk_step) {
  SetTaskAutoMove(rotate_speed, rotate_rad, walk_speed, walk_step);
}

void SetRotate(int8_t speed) {
  KillAutoMove();
  g_xpider_control->Rotate(speed);
}

void SetEye(uint8_t angle) {
  KillAutoMove();
  g_xpider_control->SetCameraAngle(angle);
}

void SetFrontLeds(const uint8_t leds[6]) {
  KillAutoMove();
  g_xpider_control->SetFrontLeds(leds);
}

void GetRegister(XpiderInsideProtocol::RegisterIndex register_index) {
  switch(register_index) {
    case XpiderInsideProtocol::kControllerVersion: {
      uint8_t led_a[6] = {20, 20, 20, 0, 20, 20};
      g_xpider_control->SetFrontLeds(led_a);
      String temp = CONTROLLER_VERSION;
      g_xpider_inside_protocol.RegisterResponse(register_index, temp.c_str(), temp.length()+1);
      break;
    }
    default: {
      break;
    }
  }
}

void WaitNetWithBlink() {
  bool rak_starting = true;
  uint8_t led_a[6] = {0, 20, 0, 0, 20, 0};
  uint8_t led_b[6] = {0, 0, 20, 0, 0, 20};
  uint8_t led_off[6] = {0, 0, 0, 0, 0, 0};
  unsigned long start_time = millis();

  while(rak_starting) {
    if(digitalRead(CAMERA_READY)) {
      g_xpider_control->SetFrontLeds(led_a);
      if(millis() - start_time > 3000){
        rak_starting = false;
      }
    } else {
      g_xpider_control->SetFrontLeds(led_b);
      start_time = millis();
    }
  }

  g_xpider_control->SetFrontLeds(led_off);
}

void setup() {
  g_hdlc = new Arduhdlc(&sendframe, &frameHandler, 256);

  g_xpider_imu = XpiderIMU::instance();
  g_xpider_imu->Initialize();

  g_xpider_control = XpiderControl::instance();
  g_xpider_control->Initialize();

  XpiderInsideProtocol::CallbackListStruct callback_list;
  callback_list.move = &SetMove;
  callback_list.step = &SetStep;
  callback_list.rotate = &SetRotate;
  callback_list.set_eye = &SetEye;
  callback_list.auto_move = &SetAutoMove;
  callback_list.set_front_leds = &SetFrontLeds;
  callback_list.get_register = &GetRegister;
  g_xpider_inside_protocol.Initialize(&InsideSendData, callback_list);

  TaskInitialize(&g_xpider_inside_protocol, g_xpider_control, g_xpider_imu, g_hdlc);

  WaitNetWithBlink();

  Serial.begin(9600);
}

void loop() {
  /* 
   * Execute each task with defined period
   */
  TaskUpdate();
}
