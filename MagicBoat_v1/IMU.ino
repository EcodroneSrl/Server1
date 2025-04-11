void imuInit() {
  imu.begin();

#ifdef BNO_055
  imu.configure(NORMAL_PWR_MODE, NDOF, BNO055_ADDRESS);
  imu.set_interrupt();
  imu.setCalAll();
  // Serialprintln(F("BNO_055_Init"));
#elif defined(BNO_086)
  // imu.enableDebMes();
  imu.configCalibSettings(true, true, true, true, true);
  imu.periodicCalibSave(true);
  delay(1000);
  imu.setReports();
  // Serialprintln(F("BNO_086_Init"));
#endif
}

// DEBUG ZONE ####################
unsigned long lastPrint = 0;
unsigned long is_entered = 0;
// ###############################

uint32_t imuTime = 0;
void computeIMU() {
  imuTime = millis();
  if (imu.update()) {
    declination = AP_Declination::get_declination(43.835651, 11.196260);  // TOFIX: TOGLIERE COORD HARDCODED
    // declination = AP_Declination::get_declination(boatLat, boatLon);
    TetaB_IMU = imu.EulerAccData.Yaw - declination;

    sendJetson(SENSOR_REPORTID_ROTATION_VECTOR);

    // is_entered = millis();  //TOFIX: Remove after debug
  }

  // DEBUG ZONE ####################
  // if (millis() - lastPrint > 200) {
  //   lastPrint = millis();
  //   Serial.printf("UPDATE: %u - DEC: %3.4f - YAW: %3.4f - TB: %3.4f\n", (millis() - is_entered) / 1000, declination, imu.EulerAccData.Yaw, TetaB_IMU);
  // }
  // ###############################

  imuTime = millis() - imuTime;
}

/*-------------- FUNCTIONS TO PRINT IMU DEBUG ------------------*/

void IMUprintCalibStat(uint8_t description_on_off, uint8_t update_on_off, uint8_t head_on_off) {
  if (update_on_off == UPDATE_ON) {
    imu.update_calib_stat();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("M:"));
  }
  Serialprint(imu.mag_cal);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("A:"));
  }
  Serialprint(imu.acc_cal);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("G:"));
  }
  Serialprint(imu.gyr_cal);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("S:"));
  }
  Serialprint(imu.sys_cal);
  if (head_on_off == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintEuler(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Euler();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Y:"));
  }
  Serialprint(imu.EulerAccData.Yaw, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("R:"));
  }
  Serialprint(imu.EulerAccData.Roll, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("P:"));
  }
  Serialprint(imu.EulerAccData.Pitch, 6);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintAcc(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Acc();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Ax:"));
  }
  Serialprint(imu.EulerAccData.Ax, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Ay:"));
  }
  Serialprint(imu.EulerAccData.Ay, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Az:"));
  }
  Serialprint(imu.EulerAccData.Az, 6);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintMag(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Mag();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Mx:"));
  }
  Serialprint(imu.Mx, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("My:"));
  }
  Serialprint(imu.My, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Mz:"));
  }
  Serialprint(imu.Mz, 6);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintGyr(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Gyr();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Gx:"));
  }
  Serialprint(imu.Gx, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Gy:"));
  }
  Serialprint(imu.Gy, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Gz:"));
  }
  Serialprint(imu.Gz, 6);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintQua(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Qua();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Qw:"));
  }
  Serialprint(imu.q0, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Qx:"));
  }
  Serialprint(imu.q1, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Qy:"));
  }
  Serialprint(imu.q2, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Qz:"));
  }
  Serialprint(imu.q3, 6);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintDataX(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Gyr();
    imu.update_Mag();
    imu.update_Acc();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Ax:"));
  }
  Serialprint(imu.EulerAccData.Ax, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Mx:"));
  }
  Serialprint(imu.Mx, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Gx:"));
  }
  Serialprint(imu.Gx, 6);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintDataY(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Gyr();
    imu.update_Mag();
    imu.update_Acc();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Ay:"));
  }
  Serialprint(imu.EulerAccData.Ay, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("My:"));
  }
  Serialprint(imu.My, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Gy:"));
  }
  Serialprint(imu.Gy, 6);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintDataZ(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Gyr();
    imu.update_Mag();
    imu.update_Acc();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Az:"));
  }
  Serialprint(imu.EulerAccData.Az, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Mz:"));
  }
  Serialprint(imu.Mz, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Gz:"));
  }
  Serialprint(imu.Gz, 6);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

#ifdef BNO_055

void IMUprintCalibOffset(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.getCalAll();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("AOx:"));
  }
  Serialprint(imu.cal_values.acc_off_x);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("AOy:"));
  }
  Serialprint(imu.cal_values.acc_off_y);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("AOz:"));
  }
  Serialprint(imu.cal_values.acc_off_z);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("GOx:"));
  }
  Serialprint(imu.cal_values.gyr_off_x);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("GOy:"));
  }
  Serialprint(imu.cal_values.gyr_off_y);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("GOz:"));
  }
  Serialprint(imu.cal_values.gyr_off_z);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("MOx:"));
  }
  Serialprint(imu.cal_values.mag_off_x);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("MOy:"));
  }
  Serialprint(imu.cal_values.mag_off_y);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("MOz:"));
  }
  Serialprint(imu.cal_values.mag_off_z);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Ar:"));
  }
  Serialprint(imu.cal_values.acc_rad);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Mr:"));
  }
  Serialprint(imu.cal_values.mag_rad);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUQua_to_Eul(uint8_t update_on_off) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Qua();
  }
  imu.Yq = (atan2(2 * ((imu.q0) * (imu.q3) + (imu.q1) * (imu.q2)), 1 - 2 * (pow((imu.q2), 2) + pow((imu.q3), 2)))) * 180 / PI;
  imu.Rq = -(asin(2 * ((imu.q0) * (imu.q2) - (imu.q3) * (imu.q1)))) * 180 / PI;
  imu.Pq = -(atan2(2 * ((imu.q0) * (imu.q1) + (imu.q2) * (imu.q3)), 1 - 2 * (pow((imu.q1), 2) + pow((imu.q2), 2)))) * 180 / PI;
}

void IMUprintEulerQua(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  IMUQua_to_Eul(update_on_off);
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Yq:"));
  }
  Serialprint(imu.Yq, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Rq:"));
  }
  Serialprint(imu.Rq, 6);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Pq:"));
  }
  Serialprint(imu.Pq, 6);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintAccConfig(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Acc_conf();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Acc G range:"));
  }
  Serialprint(imu.acc_range);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Acc BW:"));
  }
  Serialprint(imu.acc_bw);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Acc OpMode:"));
  }
  Serialprint(imu.acc_op_mode);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintGyrConfig(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Gyr_conf();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Gyr range:"));
  }
  Serialprint(imu.gyr_range);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Gyr BW:"));
  }
  Serialprint(imu.gyr_bw);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Gyr OpMode:"));
  }
  Serialprint(imu.gyr_op_mode);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

void IMUprintMagConfig(uint8_t description_on_off, uint8_t update_on_off, uint8_t head) {
  if (update_on_off == UPDATE_ON) {
    imu.update_Mag_conf();
  }

  if (description_on_off == DESCR_ON) {
    Serialprint(F("Mag data rate:"));
  }
  Serialprint(imu.mag_data_rate);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Mag OpMode:"));
  }
  Serialprint(imu.mag_op_mode);
  Serialprint(F(","));
  if (description_on_off == DESCR_ON) {
    Serialprint(F("Mag PwMode:"));
  }
  Serialprint(imu.mag_pw_mode);
  if (head == HEAD_ON) {
    Serialprintln(F(","));
  } else {
    Serialprint(F(","));
  }
}

#endif
