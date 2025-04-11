uint8_t psram_ind = 0;
#define psram_ind_max 10
#define DEBUG_COLLECT true
uint16_t timer_deb_collect = 10;
uint32_t time_deb_collect = 0;

EXTMEM PSRAM PSRAMData[psram_ind_max];

//void mediaInit()
//{
//  mediaIMU[0].setNum(NUM_MEDIA_IMU);
//  mediaMotor.setNum(NUM_MEDIA_MOTOR);
//}

void collectIMU() {
  mediaIMU[0].putValues(imu.EulerAccData.Yaw);
  mediaIMU[1].putValues(imu.EulerAccData.Pitch);
  mediaIMU[2].putValues(imu.EulerAccData.Roll);
  mediaIMU[3].putValues(imu.EulerAccData.Ax);
  mediaIMU[4].putValues(imu.EulerAccData.Ay);
  mediaIMU[5].putValues(imu.EulerAccData.Az);
  mediaIMU[6].putValues(imu.Gx);
  mediaIMU[7].putValues(imu.Gy);
  mediaIMU[8].putValues(imu.Gz);
}

void collectMotor() {
  mediaMotor[0].putValues(motors[INDEX_MOT_DD].bufferTelData.temp_motor1);
  mediaMotor[1].putValues(motors[INDEX_MOT_CD].bufferTelData.temp_motor1);
  mediaMotor[2].putValues(motors[INDEX_MOT_SS].bufferTelData.temp_motor1);
  mediaMotor[3].putValues(motors[INDEX_MOT_CS].bufferTelData.temp_motor1);
  mediaMotor[4].putValues(motors[INDEX_MOT_DD].bufferTelData.avg_motor_current);
  mediaMotor[5].putValues(motors[INDEX_MOT_CD].bufferTelData.avg_motor_current);
  mediaMotor[6].putValues(motors[INDEX_MOT_SS].bufferTelData.avg_motor_current);
  mediaMotor[7].putValues(motors[INDEX_MOT_CS].bufferTelData.avg_motor_current);
  mediaMotor[8].putValues((float)motors[INDEX_MOT_DD].bufferTelData.rpm);
  mediaMotor[9].putValues((float)motors[INDEX_MOT_CD].bufferTelData.rpm);
  mediaMotor[10].putValues((float)motors[INDEX_MOT_SS].bufferTelData.rpm);
  mediaMotor[11].putValues((float)motors[INDEX_MOT_CS].bufferTelData.rpm);
}

uint32_t psramTime = 0;
void storePSRAM() {
  psramTime = millis();
  if (DEBUG_COLLECT) {
    if (millis() - time_deb_collect > timer_deb_collect) {
      time_deb_collect = millis();
      collectIMU();
      collectMotor();
    }
  }


  if (mediaIMU[0].put_end) {

    PSRAMData[psram_ind].store_index = psram_ind;
    PSRAMData[psram_ind].time_past = millis();

    //Storing IMU
    PSRAMData[psram_ind].imuPSRAMData.Yaw_max = mediaIMU[IMU_YAW_INDEX].getMax();
    PSRAMData[psram_ind].imuPSRAMData.Yaw_min = mediaIMU[IMU_YAW_INDEX].getMin();
    PSRAMData[psram_ind].imuPSRAMData.Yaw_med = mediaIMU[IMU_YAW_INDEX].getMed();

    PSRAMData[psram_ind].imuPSRAMData.Pitch_max = mediaIMU[IMU_PITCH_INDEX].getMax();
    PSRAMData[psram_ind].imuPSRAMData.Pitch_min = mediaIMU[IMU_PITCH_INDEX].getMin();
    PSRAMData[psram_ind].imuPSRAMData.Pitch_med = mediaIMU[IMU_PITCH_INDEX].getMed();

    PSRAMData[psram_ind].imuPSRAMData.Roll_max = mediaIMU[IMU_ROLL_INDEX].getMax();
    PSRAMData[psram_ind].imuPSRAMData.Roll_min = mediaIMU[IMU_ROLL_INDEX].getMin();
    PSRAMData[psram_ind].imuPSRAMData.Roll_med = mediaIMU[IMU_ROLL_INDEX].getMed();

    PSRAMData[psram_ind].imuPSRAMData.Ax_max = mediaIMU[IMU_AX_INDEX].getMax();
    PSRAMData[psram_ind].imuPSRAMData.Ax_min = mediaIMU[IMU_AX_INDEX].getMin();
    PSRAMData[psram_ind].imuPSRAMData.Ax_med = mediaIMU[IMU_AX_INDEX].getMed();

    PSRAMData[psram_ind].imuPSRAMData.Ay_max = mediaIMU[IMU_AY_INDEX].getMax();
    PSRAMData[psram_ind].imuPSRAMData.Ay_min = mediaIMU[IMU_AY_INDEX].getMin();
    PSRAMData[psram_ind].imuPSRAMData.Ay_med = mediaIMU[IMU_AY_INDEX].getMed();

    PSRAMData[psram_ind].imuPSRAMData.Az_max = mediaIMU[IMU_AZ_INDEX].getMax();
    PSRAMData[psram_ind].imuPSRAMData.Az_min = mediaIMU[IMU_AZ_INDEX].getMin();
    PSRAMData[psram_ind].imuPSRAMData.Az_med = mediaIMU[IMU_AZ_INDEX].getMed();

    PSRAMData[psram_ind].imuPSRAMData.Gx_max = mediaIMU[IMU_GX_INDEX].getMax();
    PSRAMData[psram_ind].imuPSRAMData.Gx_min = mediaIMU[IMU_GX_INDEX].getMin();
    PSRAMData[psram_ind].imuPSRAMData.Gx_med = mediaIMU[IMU_GX_INDEX].getMed();

    PSRAMData[psram_ind].imuPSRAMData.Gy_max = mediaIMU[IMU_GY_INDEX].getMax();
    PSRAMData[psram_ind].imuPSRAMData.Gy_min = mediaIMU[IMU_GY_INDEX].getMin();
    PSRAMData[psram_ind].imuPSRAMData.Gy_med = mediaIMU[IMU_GY_INDEX].getMed();

    PSRAMData[psram_ind].imuPSRAMData.Gz_max = mediaIMU[IMU_GZ_INDEX].getMax();
    PSRAMData[psram_ind].imuPSRAMData.Gz_min = mediaIMU[IMU_GZ_INDEX].getMin();
    PSRAMData[psram_ind].imuPSRAMData.Gz_med = mediaIMU[IMU_GZ_INDEX].getMed();

    // Storing Motors
    PSRAMData[psram_ind].motorPSRAMDataDD.temp_motor1_max = mediaMotor[MOTOR_DD_TEMP_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataDD.temp_motor1_min = mediaMotor[MOTOR_DD_TEMP_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataDD.temp_motor1_med = mediaMotor[MOTOR_DD_TEMP_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataCD.temp_motor1_max = mediaMotor[MOTOR_CD_TEMP_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataCD.temp_motor1_min = mediaMotor[MOTOR_CD_TEMP_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataCD.temp_motor1_med = mediaMotor[MOTOR_CD_TEMP_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataSS.temp_motor1_max = mediaMotor[MOTOR_SS_TEMP_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataSS.temp_motor1_min = mediaMotor[MOTOR_SS_TEMP_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataSS.temp_motor1_med = mediaMotor[MOTOR_SS_TEMP_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataCS.temp_motor1_max = mediaMotor[MOTOR_CS_TEMP_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataCS.temp_motor1_min = mediaMotor[MOTOR_CS_TEMP_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataCS.temp_motor1_med = mediaMotor[MOTOR_CS_TEMP_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataDD.avg_motor_current_max = mediaMotor[MOTOR_DD_CURR_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataDD.avg_motor_current_min = mediaMotor[MOTOR_DD_CURR_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataDD.avg_motor_current_med = mediaMotor[MOTOR_DD_CURR_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataCD.avg_motor_current_max = mediaMotor[MOTOR_CD_CURR_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataCD.avg_motor_current_min = mediaMotor[MOTOR_CD_CURR_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataCD.avg_motor_current_med = mediaMotor[MOTOR_CD_CURR_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataSS.avg_motor_current_max = mediaMotor[MOTOR_SS_CURR_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataSS.avg_motor_current_min = mediaMotor[MOTOR_SS_CURR_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataSS.avg_motor_current_med = mediaMotor[MOTOR_SS_CURR_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataCS.avg_motor_current_max = mediaMotor[MOTOR_CS_CURR_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataCS.avg_motor_current_min = mediaMotor[MOTOR_CS_CURR_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataCS.avg_motor_current_med = mediaMotor[MOTOR_CS_CURR_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataDD.rpm_max = mediaMotor[MOTOR_DD_RPM_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataDD.rpm_min = mediaMotor[MOTOR_DD_RPM_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataDD.rpm_med = mediaMotor[MOTOR_DD_RPM_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataCD.rpm_max = mediaMotor[MOTOR_CD_RPM_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataCD.rpm_min = mediaMotor[MOTOR_CD_RPM_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataCD.rpm_med = mediaMotor[MOTOR_CD_RPM_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataSS.rpm_max = mediaMotor[MOTOR_SS_RPM_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataSS.rpm_min = mediaMotor[MOTOR_SS_RPM_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataSS.rpm_med = mediaMotor[MOTOR_SS_RPM_INDEX].getMed();

    PSRAMData[psram_ind].motorPSRAMDataCS.rpm_max = mediaMotor[MOTOR_CS_RPM_INDEX].getMax();
    PSRAMData[psram_ind].motorPSRAMDataCS.rpm_min = mediaMotor[MOTOR_CS_RPM_INDEX].getMin();
    PSRAMData[psram_ind].motorPSRAMDataCS.rpm_med = mediaMotor[MOTOR_CS_RPM_INDEX].getMed();

    //Storing BMS
    PSRAMData[psram_ind].bmsPSRAMData.PackCurrent = bms.BasicData.PackCurrent;
    PSRAMData[psram_ind].bmsPSRAMData.PackCurrent = bms.EepromData.dsg_rate;

    for (uint8_t j = 0; j < bms.num_cell_set; j++) {
      PSRAMData[psram_ind].bmsPSRAMData.vcell[j] = bms.vcell[j];
    }

    //Storing GPS
    combinaStruct(&PSRAMData[psram_ind].pvtPSRAMData, &gps.pvt1, sizeof(PSRAMData[psram_ind].pvtPSRAMData));
    combinaStruct(&PSRAMData[psram_ind].relposnedPSRAMData, &gps.relposned1, sizeof(PSRAMData[psram_ind].relposnedPSRAMData));


    psram_ind++;

    for (uint8_t i = 0; i < sizeof(mediaIMU) / sizeof(mediaIMU[0]); i++) {
      mediaIMU[i].put_end = 0;
    }

    for (uint8_t i = 0; i < sizeof(mediaMotor) / sizeof(mediaMotor[0]); i++) {
      mediaMotor[i].put_end = 0;
    }
  }
  if (psram_ind >= psram_ind_max) {
    //printLog();
    writeStruct();
    psram_ind = 0;
  }
  psramTime = millis() - psramTime;
}

void printLog() {
  Serialprint(F("------------------- START BUFFER ---------------------"));
  Serialprintln();
  for (uint8_t sd_ind = 0; sd_ind < psram_ind_max; sd_ind++) {
    Serialprint(F("N:"));
    Serialprint(sd_ind + 1);
    Serialprintln();

    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Yaw_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Yaw_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Yaw_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Pitch_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Pitch_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Pitch_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Roll_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Roll_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Roll_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Ax_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Ax_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Ax_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Ay_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Ay_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Ay_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Az_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Az_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Az_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Gx_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Gx_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Gx_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Gy_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Gy_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].imuPSRAMData.Gy_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataDD.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataDD.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataDD.rpm_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCD.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCD.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCD.rpm_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataSS.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataSS.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataSS.rpm_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCS.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCS.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCS.rpm_med);
    Serialprint(F(","));
    Serialprintln();

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataDD.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataDD.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataDD.rpm_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCD.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCD.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCD.rpm_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataSS.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataSS.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataSS.rpm_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCS.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCS.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCS.rpm_med);
    Serialprint(F(","));
    Serialprintln();

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataDD.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataDD.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataDD.rpm_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCD.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCD.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCD.rpm_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataSS.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataSS.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataSS.rpm_med);
    Serialprint(F(","));

    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCS.rpm_max);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCS.rpm_min);
    Serialprint(F(","));
    Serialprint(PSRAMData[sd_ind].motorPSRAMDataCS.rpm_med);
    Serialprint(F(","));
    Serialprintln();
  }
  Serialprint(F("------------------- END BUFFER ---------------------"));
  Serialprintln();
}
