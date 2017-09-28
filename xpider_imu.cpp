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

#include "xpider_imu.h"

XpiderIMU* XpiderIMU::instance_ = NULL;

XpiderIMU::XpiderIMU() {

}

XpiderIMU::~XpiderIMU() {

}

XpiderIMU* XpiderIMU::instance() {
  if(instance_ == NULL) {
    instance_ = new XpiderIMU();
  }

  return instance_;
}

bool XpiderIMU::Initialize() {
  dmp_ready_ = false;

  Wire.begin();
  Wire.setClock(400000);

  // initialize device
  // Serial.println(F("Initializing I2C devices..."));
  mpu_.initialize();

  // verify connection
  // Serial.println(F("Testing device connections..."));
  // Serial.println(mpu_.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Serial.println(F("Initializing DMP..."));
  device_status_ = mpu_.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu_.setXGyroOffset(220);
  mpu_.setYGyroOffset(76);
  mpu_.setZGyroOffset(-85);
  mpu_.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (device_status_ == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu_.setDMPEnabled(true);

    mpu_int_status_ = mpu_.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmp_ready_ = true;

    // get expected DMP packet size for later comparison
    packet_size_ = mpu_.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    DEBUG_PRINT(F("DMP Initialization failed (code "));
    DEBUG_PRINT(device_status_);
    DEBUG_PRINTLN(F(")"));
  }
  return true;
}

void XpiderIMU::GetYawPitchRoll(float in_data[3]) {
  // if programming failed, don't try to do anything
  if (dmp_ready_ == false) {
    DEBUG_PRINTLN("DMP not ready");
    return;
  }

  while(1) {
    DEBUG_PRINT("IMU: get int status...");
    mpu_int_status_ = mpu_.getIntStatus();
    DEBUG_PRINTLN("Done");
    
    // get current FIFO count
    DEBUG_PRINT("IMU: get fifo count...");
    fifo_count_ = mpu_.getFIFOCount();
    DEBUG_PRINTLN("Done");
  
//    Serial.print("count: ");
//    Serial.print(fifo_count_);
//    Serial.print("\t");
//    Serial.print("status: ");
//    Serial.print(mpu_int_status_);
//    Serial.print("\t");
  
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpu_int_status_ & 0x10) || fifo_count_ == 1024) {
      // reset so we can continue cleanly
      mpu_.resetFIFO();
      DEBUG_PRINTLN(F("FIFO overflow!"));
      memset(in_data, 0, sizeof(float)*3);
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpu_int_status_ & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      DEBUG_PRINT("IMU: Wait for enough bytes...");
      while (fifo_count_ < packet_size_) fifo_count_ = mpu_.getFIFOCount();
      DEBUG_PRINTLN("Done");
  
      // read a packet from FIFO
      DEBUG_PRINT("IMU: read packet...");
      mpu_.getFIFOBytes(fifo_buffer_, packet_size_);
      DEBUG_PRINTLN("Done");
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifo_count_ -= packet_size_;
  
      DEBUG_PRINT("IMU: Calculate YPR");
      mpu_.dmpGetQuaternion(&quaternion_, fifo_buffer_);
      mpu_.dmpGetGravity(&gravity_, &quaternion_);
      mpu_.dmpGetYawPitchRoll(in_data, &quaternion_, &gravity_);
      DEBUG_PRINTLN("Done");

      for(int i=0; i<3; i++) {
        in_data[i] = in_data[i]<0 ? in_data[i]+PI*2.0 : in_data[i];
      }
      break;
    } else {
      DEBUG_PRINTLN("IMU: unknown");
    }
  }
}
