/*
 * Xpider controller software, running on Atmel MEGA328p
 * Copyright (C) 2015-2017  Roboeve, MakerCollider
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
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
