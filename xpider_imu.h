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

#ifndef XPIDER_IMU_H_
#define XPIDER_IMU_H_

#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

class XpiderIMU {
private:
  static XpiderIMU* instance_;

  MPU6050 mpu_;
  uint16_t fifo_count_;        // count of all bytes currently in FIFO
  uint8_t fifo_buffer_[64];   // FIFO storage buffer
  bool dmp_ready_;            // set true if DMP init was successful
  uint8_t mpu_int_status_;    // holds actual interrupt status byte from MPU
  uint8_t device_status_;     // return status after each device operation (0 = success, !0 = error)
  uint16_t packet_size_;      // expected DMP packet size (default is 42 bytes)
  Quaternion quaternion_;     // [w, x, y, z]         quaternion container
  VectorFloat gravity_;       // [x, y, z]            gravity vector

private:
  XpiderIMU();
  ~XpiderIMU();

public:
  static XpiderIMU* instance();

  bool Initialize();
  void GetYawPitchRoll(float in_data[3]);
};

#endif // XPIDER_IMU_H_


