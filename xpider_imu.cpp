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
    // Serial.print(F("DMP Initialization failed (code "));
    // Serial.print(device_status_);
    // Serial.println(F(")"));
  }
  return true;
}

void XpiderIMU::GetYawPitchRoll(float in_data[3]) {
  // if programming failed, don't try to do anything
  if (!dmp_ready_) return;

  while(1) {
    mpu_int_status_ = mpu_.getIntStatus();
    
    // get current FIFO count
    fifo_count_ = mpu_.getFIFOCount();
  
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
      // Serial.println(F("FIFO overflow!"));
      memset(in_data, 0, sizeof(float)*3);
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpu_int_status_ & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifo_count_ < packet_size_) fifo_count_ = mpu_.getFIFOCount();
  
      // read a packet from FIFO
      mpu_.getFIFOBytes(fifo_buffer_, packet_size_);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifo_count_ -= packet_size_;
  
      mpu_.dmpGetQuaternion(&quaternion_, fifo_buffer_);
      mpu_.dmpGetGravity(&gravity_, &quaternion_);
      mpu_.dmpGetYawPitchRoll(in_data, &quaternion_, &gravity_);

      for(int i=0; i<3; i++) {
        in_data[i] = in_data[i]<0 ? in_data[i]+PI*2.0 : in_data[i];
      }
      break;
    }
  }
}
