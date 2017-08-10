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
