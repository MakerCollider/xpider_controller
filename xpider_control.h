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

#ifndef XPIDER_CONTROL_H_
#define XPIDER_CONTROL_H_

#include <Adafruit_NeoPixel.h>

#include "xpider_pin.h"

class XpiderControl{
public:
  static XpiderControl* instance();

  int walk_status() { return walk_status_; }
  int rotate_status() { return rotate_status_; }
  bool is_low_battery() { return is_low_battery_; }
  uint16_t step_counter() { return step_counter_; }

  void Initialize();

  void StopAll();
  void BrakeAll();

  void Walk(const int8_t&);
  void Step(const int8_t&, const uint8_t&);
  void Rotate(const int8_t&);

  void StopWalking();
  void StopWalkOnHigh();
  void StopRotating();

  void BrakeWalking();
  void BrakeRotating();

  uint16_t GetObstacleDistance();

  uint8_t GetMicSoundLevel();

  uint16_t GetCameraAngle();
  bool SetCameraAngle(const uint8_t&);

  void SetFrontLeds(const uint8_t[6]);

  void ClearFrontLeds();

  float GetVoltage();

  bool InitializeRotate();
  bool InitializeCameraAngle();

private:
  static XpiderControl* instance_;

  int8_t walk_status_;
  int8_t rotate_status_;

  bool stop_flag;
  bool is_low_battery_;
  uint16_t step_counter_;

  uint16_t camera_min_value_, camera_max_value_;

  float voltage_sum_;
  int voltage_buffer_[20];
  uint8_t voltage_counter_;
  bool voltage_buffer_initialized_;

  int32_t distance_sum_;
  int distance_buffer_[20];
  uint8_t distance_counter_;
  bool distance_buffer_initialized_;

  Adafruit_NeoPixel *front_leds_;

  XpiderControl();
  ~XpiderControl();

  void BrakeEye();
  void StopEye();
  void MoveEye(int8_t speed);

  static void StepCounterHandler();

  void InitializeVoltageBuffer();
  void InitializeObstacleDistance();
};

#endif // XPIDER_CONTROL_H_
