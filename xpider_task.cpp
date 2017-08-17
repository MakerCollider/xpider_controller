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

#include "xpider_task.h"

#include <math.h>

// #include "pid.h"
#include "task.h"
#include "Arduhdlc.h"
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

#define ROTATE_MIN_ERROR 0.26f

Task task_heartbeat;
Task task_automove;

uint8_t task_rotate_speed;
float task_rotate_rad;
int8_t task_walk_speed_direction;
int32_t task_walk_step;

Arduhdlc *task_hdlc;
XpiderIMU *task_xpider_imu;
XpiderControl *task_xpider_control;
XpiderInsideProtocol *task_xpider_inside_protocol;

void TaskUpdate() {
  task_heartbeat.trigger(micros());
  task_automove.trigger(micros());
}

void SetTaskAutoMove(uint8_t rotate_speed, float rotate_rad, uint8_t walk_speed, int8_t walk_step) {
  task_rotate_speed = rotate_speed;
  task_walk_speed_direction = walk_step>0 ? walk_speed : walk_speed*(-1);

  task_walk_step = static_cast<int32_t>(task_xpider_control->step_counter()) +
                   static_cast<int32_t>(abs(walk_step))-1+abs(static_cast<int8_t>(task_xpider_control->walk_status()));

  float ypr[3];
  task_xpider_imu->GetYawPitchRoll(ypr);
  task_rotate_rad = fmod(ypr[0]+rotate_rad, 2*PI);

  task_automove.setEnabled(true);

  DEBUG_PRINTLN("=== Set automove ===");
  DEBUG_PRINT("Rotate: speed ");
  DEBUG_PRINT(static_cast<int>(rotate_speed));
  DEBUG_PRINT(", target_rad ");
  DEBUG_PRINT(task_rotate_rad);
  DEBUG_PRINT(", Walk: speed_direction ");
  DEBUG_PRINT(task_walk_speed_direction);
  DEBUG_PRINT("delta_step: ");
  DEBUG_PRINT(walk_step);
  DEBUG_PRINTLN("=== Set automove finish ===");
}

void TaskHeartBeat() {
  XpiderInsideProtocol::HeartBeatStruct heartbeat;

  task_xpider_imu->GetYawPitchRoll(heartbeat.yaw_pitch_roll);
  heartbeat.obstacle_distance = task_xpider_control->GetObstacleDistance();
  heartbeat.battery_voltage = task_xpider_control->GetVoltage();
  heartbeat.step_counter = task_xpider_control->step_counter();

  task_xpider_inside_protocol->SetHeartBeat(heartbeat);

  DEBUG_PRINT("=== Update HeartBeat ===");
  DEBUG_PRINT("voltage: ");
  DEBUG_PRINT(heartbeat.battery_voltage);
  DEBUG_PRINT(", step_data: ");
  DEBUG_PRINT(heartbeat.step_counter);
  DEBUG_PRINT(", obstacle_distance: ");
  DEBUG_PRINT(heartbeat.obstacle_distance);
  DEBUG_PRINT(", ypr: ");
  DEBUG_PRINT(heartbeat.yaw_pitch_roll[0]);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(heartbeat.yaw_pitch_roll[1]);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(heartbeat.yaw_pitch_roll[2]);
  DEBUG_PRINT("=== Update heartbeat finish ===");
}

void KillAutoMove() {
  if(task_automove.enabled()) {
    task_xpider_control->StopWalkOnHigh();
    task_xpider_control->StopRotating();
    task_automove.setEnabled(false);
  }
}

void TaskAutoMove() {
  float ypr[3], delta_rad1, delta_rad2;
  task_xpider_imu->GetYawPitchRoll(ypr);
  delta_rad1 = task_rotate_rad-ypr[0];
  delta_rad2 = delta_rad1<0 ? PI*2.0+delta_rad1 : delta_rad1-PI*2.0;
  delta_rad1 = abs(delta_rad1)-abs(delta_rad2)>0 ? delta_rad2 : delta_rad1;

  /* 
   * check if angle is not correct
   */
  if(abs(delta_rad1) > ROTATE_MIN_ERROR) {
    int direction = delta_rad1>0 ? 1 : -1;
    task_xpider_control->StopWalkOnHigh();
    task_xpider_control->Rotate(direction*task_rotate_speed);
  } else {
    /*
     * stop rotate, and check if step is enough
     */
    task_xpider_control->BrakeRotating();
    if(static_cast<int32_t>(task_xpider_control->step_counter()) < task_walk_step) {
      task_xpider_control->Walk(task_walk_speed_direction);
    } else {
      task_xpider_control->StopWalkOnHigh();
      task_xpider_control->StopRotating();
      /*
       * finish automove
       */
      task_automove.setEnabled(false);
    }
  }
}

void TaskInitialize(XpiderInsideProtocol *xpider_inside_protocol, XpiderControl *xpider_control,
                    XpiderIMU *xpider_imu, Arduhdlc *hdlc) {
  task_hdlc = hdlc;
  task_xpider_imu = xpider_imu;
  task_xpider_control = xpider_control;
  task_xpider_inside_protocol = xpider_inside_protocol;

  task_automove.init(TASK_AUTOMOVE_PERIOD, TaskAutoMove);
  task_automove.setEnabled(false);
  task_heartbeat.init(TASK_HEARTBEAT_PERIOD, TaskHeartBeat);
  task_heartbeat.setEnabled(true);
}