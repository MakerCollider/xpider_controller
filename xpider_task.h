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

#ifndef XPIDER_TASK_H_
#define XPIDER_TASK_H_

#include "Arduhdlc.h"
#include "xpider_imu.h"
#include "xpider_inside_protocol.h"
#include "xpider_control.h"

#define TASK_AUTOMOVE_PERIOD    10000       /* 10ms */
#define TASK_HEARTBEAT_PERIOD   50000       /* 50ms */

extern void TaskInitialize(XpiderInsideProtocol *xpider_inside_protocol, XpiderControl *xpider_control,
                           XpiderIMU *xpider_imu, Arduhdlc *hdlc);
extern void TaskUpdate();
extern void KillAutoMove();
extern void SetTaskAutoMove(uint8_t rotate_speed, float rotate_rad, uint8_t walk_speed, int8_t walk_step);

#endif // XPIDER_TASK_H_
