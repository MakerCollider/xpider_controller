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

#ifndef PID_H_
#define PID_H_

#include <Arduino.h>

class PID {
public:
  PID(float kp, float ki, float kd, float out_max, float out_min);
  ~PID();

  float k_p() { return k_p_; }
  float k_i() { return k_i_; }
  float k_d() { return k_d_; }

  void set_k_p(float value) { k_p_ = value; }
  void set_k_i(float value) { k_i_ = value; }
  void set_k_d(float value) { k_d_ = value; }

  bool clear();
  float update(float target, float actual);

private:
  float k_p_;                // 比例系数
  float k_i_;                // 积分系数
  float k_d_;                // 微分系数

  float integral_;           // 积分值
  int8_t i_enable_;          // 积分项使能

  float delta_value_;        // 偏差值
  float last_delta_value_;   // 上一个偏差值

  float output_;             // 输出值
  float last_output_;        // 上一次输出值
  float output_max_;         // 最大输出值
  float output_min_;         // 最小输出值
};

#endif // PID_H_


