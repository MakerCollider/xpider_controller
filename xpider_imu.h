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


