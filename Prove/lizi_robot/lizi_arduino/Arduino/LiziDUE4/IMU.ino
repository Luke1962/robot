void  setup_imu() {
  Wire.begin();
  dueMPU.selectDevice(DEVICE_TO_USE);                        // only really necessary if using device 1
  imu_fault = !dueMPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE); // start the MPU
  if (imu_fault == false)   nh.loginfo("IMU ready");
  else   nh.loginfo("IMU fault");
  CHECK_IMU_INTERVAL = (unsigned long)(1000.0 / MPU_UPDATE_RATE * 50.0);

  loopState = LOOPSTATE_NORMAL;
  pollInterval = (1000 / MPU_UPDATE_RATE) - 1; // a bit less than the minimum interval
  lastPollTime = millis();
}


boolean duePoll()
{
  if ((millis() - lastPollTime) < pollInterval)
    return false; // not time yet
  if (dueMPU.read()) {
    lastPollTime = millis();
    return true;
  }
  return false; // try again next time round
}

void read_imu() {



  switch (loopState) {
    case LOOPSTATE_NORMAL:
      if (duePoll()) { // get the latest data if ready yet
        qw = dueMPU.m_fusedQuaternion[0];
        qx = dueMPU.m_fusedQuaternion[1];
        qy = dueMPU.m_fusedQuaternion[2];
        qz = dueMPU.m_fusedQuaternion[3];
        imu_t = millis();
      }
      break;

    case LOOPSTATE_MAGCAL:
      magCalLoop();
       imu_t = millis();
      break;

    case LOOPSTATE_ACCELCAL:
      accelCalLoop();
       imu_t = millis();
      break;
  }




}


void imu_calibCb(const imu_calib::Request & req, imu_calib::Response & res) {

  switch (loopState) {
    case LOOPSTATE_NORMAL:
      switch (req.com) {
        case 1:
          magCalStart();
          return;

        case 2:
          accelCalStart();
          return;
      }
      break;

    case LOOPSTATE_MAGCAL:
      switch (req.com)  {
        case 3:
          calData.magValid = true;
          calLibWrite(DEVICE_TO_USE, &calData);
          nh.loginfo("Mag cal data saved");
          break;

        case 4:
          loopState = LOOPSTATE_NORMAL;
          nh.loginfo("\n\n *** restart to use calibrated data ***");
          break;
      }
      break;

    case LOOPSTATE_ACCELCAL:
      switch (req.com) {
        case 3:

          calData.accelValid = true;
          calLibWrite(DEVICE_TO_USE, &calData);
          nh.loginfo("Accel cal data saved");
          break;

        case 4:

          loopState = LOOPSTATE_NORMAL;
          nh.loginfo("\n\n *** restart to use calibrated data ***");
          break;
      }
      break;
  }

}

void magCalStart(void)
{
  calLibRead(DEVICE_TO_USE, &calData); // pick up existing accel data if there

  calData.magValid = false;
  calData.magMinX = 0x7fff; // init mag cal data
  calData.magMaxX = 0x8000;
  calData.magMinY = 0x7fff;
  calData.magMaxY = 0x8000;
  calData.magMinZ = 0x7fff;
  calData.magMaxZ = 0x8000;

  nh.loginfo("\n\nEntering mag calibration mode");
  loopState = LOOPSTATE_MAGCAL;
}

void magCalLoop()
{
  boolean changed;

  if (duePoll()) { // get the latest data
    changed = false;
    if (dueMPU.m_rawMag[VEC3_X] < calData.magMinX) {
      calData.magMinX = dueMPU.m_rawMag[VEC3_X];
      changed = true;
    }
    if (dueMPU.m_rawMag[VEC3_X] > calData.magMaxX) {
      calData.magMaxX = dueMPU.m_rawMag[VEC3_X];
      changed = true;
    }
    if (dueMPU.m_rawMag[VEC3_Y] < calData.magMinY) {
      calData.magMinY = dueMPU.m_rawMag[VEC3_Y];
      changed = true;
    }
    if (dueMPU.m_rawMag[VEC3_Y] > calData.magMaxY) {
      calData.magMaxY = dueMPU.m_rawMag[VEC3_Y];
      changed = true;
    }
    if (dueMPU.m_rawMag[VEC3_Z] < calData.magMinZ) {
      calData.magMinZ = dueMPU.m_rawMag[VEC3_Z];
      changed = true;
    }
    if (dueMPU.m_rawMag[VEC3_Z] > calData.magMaxZ) {
      calData.magMaxZ = dueMPU.m_rawMag[VEC3_Z];
      changed = true;
    }

    if (changed) {
      nh.loginfo("-------");

      sprintf(temp_msg, "minX:  %d", calData.magMinX); nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxX:  %d", calData.magMaxX); nh.loginfo(temp_msg);
      sprintf(temp_msg, "minY:  %d", calData.magMinY); nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxY:  %d", calData.magMaxY); nh.loginfo(temp_msg);
      sprintf(temp_msg, "minZ:  %d", calData.magMinZ); nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxZ:  %d", calData.magMaxZ); nh.loginfo(temp_msg);


    }
  }


}

void accelCalStart(void)
{
  calLibRead(DEVICE_TO_USE, &calData); // pick up existing accel data if there

  calData.accelValid = false;
  calData.accelMinX = 0x7fff; // init accel cal data
  calData.accelMaxX = 0x8000;
  calData.accelMinY = 0x7fff;
  calData.accelMaxY = 0x8000;
  calData.accelMinZ = 0x7fff;
  calData.accelMaxZ = 0x8000;

  nh.loginfo("\n\nEntering accel calibration mode");
  loopState = LOOPSTATE_ACCELCAL;
  dueMPU.disableAccelCal();
}

void accelCalLoop()
{
  boolean changed;

  if (duePoll()) { // get the latest data
    changed = false;
    if (dueMPU.m_rawAccel[VEC3_X] < calData.accelMinX) {
      calData.accelMinX = dueMPU.m_rawAccel[VEC3_X];
      changed = true;
    }
    if (dueMPU.m_rawAccel[VEC3_X] > calData.accelMaxX) {
      calData.accelMaxX = dueMPU.m_rawAccel[VEC3_X];
      changed = true;
    }
    if (dueMPU.m_rawAccel[VEC3_Y] < calData.accelMinY) {
      calData.accelMinY = dueMPU.m_rawAccel[VEC3_Y];
      changed = true;
    }
    if (dueMPU.m_rawAccel[VEC3_Y] > calData.accelMaxY) {
      calData.accelMaxY = dueMPU.m_rawAccel[VEC3_Y];
      changed = true;
    }
    if (dueMPU.m_rawAccel[VEC3_Z] < calData.accelMinZ) {
      calData.accelMinZ = dueMPU.m_rawAccel[VEC3_Z];
      changed = true;
    }
    if (dueMPU.m_rawAccel[VEC3_Z] > calData.accelMaxZ) {
      calData.accelMaxZ = dueMPU.m_rawAccel[VEC3_Z];
      changed = true;
    }

    if (changed) {
      nh.loginfo("-------");

      sprintf(temp_msg, "minX:  %d", calData.accelMinX); nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxX:  %d", calData.accelMaxX); nh.loginfo(temp_msg);
      sprintf(temp_msg, "minY:  %d", calData.accelMinY); nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxY:  %d", calData.accelMaxY); nh.loginfo(temp_msg);
      sprintf(temp_msg, "minZ:  %d", calData.accelMinZ); nh.loginfo(temp_msg);
      sprintf(temp_msg, "maxZ:  %d", calData.accelMaxZ); nh.loginfo(temp_msg);

    }
  }


}
