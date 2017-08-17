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

#include "PID.h"

PID::PID(float kp, float ki, float kd, float out_max, float out_min) {
  k_p_ = kp;
  k_i_ = ki;
  k_d_ = kd;

  output_max_ = out_max;
  output_min_ = out_min;
}

PID::~PID() {

}

float PID::update(float target_value, float actual_value) {
  delta_value_ = target_value - actual_value;

  if(abs(delta_value_) < 0.8) {
    i_enable_ = 1;
    if((last_output_ > output_max_ && delta_value_ < 0) ||
       (last_output_ < output_min_ && delta_value_ > 0) ||
       (last_output_ < output_max_ && actual_value > output_min_)) {
        // TODO: add description
      integral_ += delta_value_;
    }
  } else {
    i_enable_ = 0;
  }

  output_ = k_p_ * delta_value_;
  output_ += i_enable_ * k_i_ * integral_;
  output_ += k_d_ * (delta_value_ - last_delta_value_);

  last_output_ = output_;
  last_delta_value_ = delta_value_;

  return output_;
}

bool PID::clear() {
  last_output_ = 0;
  integral_ = 0;
  return true;
}


