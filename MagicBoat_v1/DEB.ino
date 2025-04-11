// define necessarie per abilitare la print dei motori
#define UPDATE_CD_OFF 0
#define UPDATE_CD_ON 1
#define UPDATE_CS_OFF 0
#define UPDATE_CS_ON 1
#define UPDATE_DD_OFF 0
#define UPDATE_DD_ON 1
#define UPDATE_SS_OFF 0
#define UPDATE_SS_ON 1

#define PRINT_LORA_INERVAL 1000


#define DEBUG_LOOP_TIMINGS true

#define DEBUG_LORA false
#define DUMMY_VAR false


extern uint32_t imuTime, psramTime, gpsTime, motorTime, sendMuxTime, nextMissionTime, ser485Time;
void debugPrint() {

  if (DEBUG_LOOP_TIMINGS && loopTime > 75) {
    uint32_t timings[] = { imuTime, psramTime, gpsTime, motorTime, boatTime, bmsTime, coolTime, buttonTime, nextMissionTime, cmdRwTime, ser485Time, sendCostructBuffTime };

    Serial.println("################# TIMINGS #################");

    //COMPUTE IMU
    Serial.print(F("IMU time:\t"));
    Serial.print(imuTime);

    //storePSRAM (psramTime)
    Serial.print("\tPSRAM time:\t");
    Serial.println(psramTime);

    //COMPUTE GPS
    Serial.print("GPS time:\t");
    Serial.print(gpsTime);

    //COMPUTE ALL MOTORS
    Serial.print("\tMOTOR time:\t");
    Serial.println(motorTime);

    //BOAT CONTROL
    Serial.print("BOAT CTRL:\t");
    Serial.print(boatTime);

    //COMPUTE BMS
    Serial.print("\tBMS time:\t");
    Serial.println(bmsTime);

    //COMPUTE COOL
    Serial.print("COOL time:\t");
    Serial.print(coolTime);

    //COMPUTE BUTTON
    Serial.print("\tBUTTON time:\t");
    Serial.println(buttonTime);

    //TIMER NEXT MISSION
    Serial.print("NXT MISSION:\t");
    Serial.print(nextMissionTime);

    //CMD RW
    Serial.print("\tCMD RW time:\t");
    Serial.println(cmdRwTime);

    //COMPUTE 485
    Serial.print("485 time:\t");
    Serial.print(ser485Time);


    //SEND CONSTRUCT BUFFER
    Serial.print("\tSND.COS.BUFF:\t");
    Serial.println(sendCostructBuffTime);


    //WHOLE MAIN LOOP
    Serial.print("\nLOOP time:\t");
    Serial.print(loopTime);

    uint8_t longestTask = 0;
    for (uint8_t i = 0; i < sizeof(timings) / sizeof(timings[0]); i++)
      if (timings[longestTask] < timings[i]) longestTask = i;

    String longestTaskName = "CASE ERROR";
    switch (longestTask) {
      case 0:
        longestTaskName = "IMU time";
        break;
      case 1:
        longestTaskName = "PSRAM time";
        break;
      case 2:
        longestTaskName = "GPS time";
        break;
      case 3:
        longestTaskName = "MOTOR time";
        break;
      case 4:
        longestTaskName = "BOAT CTRL";
        break;
      case 5:
        longestTaskName = "BMS time";
        break;
      case 6:
        longestTaskName = "COOL time";
        break;
      case 7:
        longestTaskName = "BUTTON time";
        break;
      case 8:
        longestTaskName = "NXT MISSION";
        break;
      case 9:
        longestTaskName = "CMD RW time";
        break;
      case 10:
        longestTaskName = "485 time";
        break;
      case 11:
        longestTaskName = "SND COS BUFF";
        break;

      default:
        longestTaskName = "ERRORE";
    }

    Serial.print("\tLONGEST: ");
    Serial.printf("%s\n\n", longestTaskName.c_str());
  }

  if (DUMMY_VAR) dummyVar();
  if (DEBUG_LORA) debugLoRa();

  if (loop_en_data.debugPrintFlag) {
    debugImu();
    debugGps();
    // debugMot();
    debugBms();
    debugEcho();
    debugMPPT();
    debugJS();
  }
}

/*------------------- GENERA VARIABILI DUMMY -----------------------*/
uint16_t randomMinMax = 100;
void dummyVar() {
  float randomNumMill = ((float)(random(-randomMinMax, randomMinMax))) / 1000;
  int8_t randomNumDec = random(-10, 10);

  imu.EulerAccData.Yaw = 45 + randomNumMill;
  imu.EulerAccData.Roll = (-30) + randomNumMill;
  imu.EulerAccData.Pitch = (-100) + randomNumMill;
  imu.EulerAccData.Ax = 1.1;  // 1 + randomNumMill;
  imu.EulerAccData.Ay = (-3) + randomNumMill;
  imu.EulerAccData.Az = (-6) + randomNumMill;
  imu.Gx = 7 + randomNumMill;
  imu.Gy = (-8) + randomNumMill;
  imu.Gz = (23) + randomNumMill;

  motors[INDEX_MOT_DD].bufferTelData.rpm = 1000 + (randomNumMill * 1000);
  motors[INDEX_MOT_CD].bufferTelData.rpm = 1000 + (randomNumMill * 1000);
  motors[INDEX_MOT_SS].bufferTelData.rpm = 1000 + (randomNumMill * 1000);
  motors[INDEX_MOT_CS].bufferTelData.rpm = 1000 + (randomNumMill * 1000);

  gps.pvt1.day = 18;
  gps.pvt1.month = 10;
  gps.pvt1.year = 2022;

  gps.pvt1.hour = 15;
  gps.pvt1.min = 00;
  gps.pvt1.sec = 00;

  boatLon = 437762349 + randomNumDec;
  boatLat = 112637266 + randomNumDec;

  for (int i = 0; i < 16; i++) {
    bms.BasicData.balance_status[i] = i;
  }

  bms.BasicData.cell_ov = 1;
  bms.BasicData.cell_uv = 2;
  bms.BasicData.pack_ov = 3;
  bms.BasicData.pack_uv = 4;
  bms.BasicData.charge_ot = 5;
  bms.BasicData.charge_ut = 6;
  bms.BasicData.discharge_ot = 7;
  bms.BasicData.discharge_ut = 8;
  bms.BasicData.charge_oc = 9;
  bms.BasicData.discharge_oc = 10;
  bms.BasicData.short_circuit = 11;
  bms.BasicData.frontend_error = 12;
  bms.BasicData.locked_FET = 13;

  for (int i = 0; i < 16; i++) {
    bms.vcell[i] = 3.7 + randomNumMill;
  }
}

/*------------------------- DEBUG LORA -----------------------------*/
uint32_t loraTiming = 0;
void debugLoRa() {
  if (millis() - loraTiming > PRINT_LORA_INERVAL) {
    loraTiming = millis();
    Serial7.printf("Cupolino uptime: %us", millis() / 1000);
  }
}

/*------------------------- DEBUG MPPT -----------------------------*/
void printMPPT(uint8_t description, uint8_t head) {
  for (uint8_t i = 0; i < 4; i++) {
    if (description) {
      Serialprint(F("MPPT"));
      Serialprint(i);
      Serialprint(F(": "));
    }
    if (description) {
      Serialprint(F("Iin:"));
    }
    Serialprint(hfrIntData[i].Iin / 100.0);
    Serialprint(F(","));

    if (description) {
      Serialprint(F("Vin:"));
    }
    Serialprint(hfrIntData[i].Vin / 100.0);
    Serialprint(F(","));

    if (description) {
      Serialprint(F("Iout:"));
    }
    Serialprint(hfrIntData[i].Iout / 100.0);
    Serialprint(F(","));

    if (description) {
      Serialprint(F("Vout:"));
    }
    Serialprint(hfrIntData[i].Vout / 100.0);
    Serialprint(F(","));

    if (description) {
      Serialprint(F("DutyPWM:"));
    }
    Serialprint(hfrIntData[i].dutyPWM);
    Serialprint(F(","));

    if (head) {
      Serialprintln();
    }
  }
}

/*------------------------- DEBUG IMU ------------------------------*/
void debugImu() {
  for (uint8_t i = 0; i < sizeof(imu.debug_par.debug_enable); i++) {
    if (imu.debug_par.debug_enable[i]) {
      debug_switch_imu(i, imu.debug_par.debug_timer[i]);
    }
  }
}
// **** Ho dichiarato nella libreria array per enable della descrizione, ****
// **** enable dell'update variabili ed enable a capo per ogni variabile ****
void debug_switch_imu(uint8_t i, uint16_t debug_timer) {
  if ((millis() - imu.debug_time[i]) > debug_timer) {
    imu.debug_time[i] = millis();
    switch (i) {
#ifdef BNO_055

      case DEB_CALIB_OFFSET:
        IMUprintCalibOffset(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_ACC_CONF:
        IMUprintAccConfig(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_GYR_CONF:
        IMUprintGyrConfig(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_MAG_CONF:
        IMUprintMagConfig(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_QUA:
        IMUprintQua(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_EUL_QUA:
        IMUprintEulerQua(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

#endif

      case DEB_CALIB_STAT:
        IMUprintCalibStat(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_X:
        IMUprintDataX(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_Y:
        IMUprintDataY(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_Z:
        IMUprintDataZ(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_EULER:
        IMUprintEuler(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_ACC:
        IMUprintAcc(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_GYR:
        IMUprintGyr(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;

      case DEB_MAG:
        IMUprintMag(imu.debug_par.debug_en_descr[i], imu.debug_par.debug_en_update[i], imu.debug_par.debug_en_head[i]);
        break;
    }
  }
}

/*-------------------------- DEBUG GPS -----------------------------*/
void debugGps() {
  for (uint8_t i = 0; i < sizeof(gps.debug_par.debug_enable); i++) {
    if (gps.debug_par.debug_enable[i]) {
      debug_switch_gps(i, gps.debug_par.debug_timer[i]);
    }
  }
}

void debug_switch_gps(uint8_t i, uint16_t debug_timer) {
  if ((millis() - gps.debug_time[i]) > debug_timer) {
    gps.debug_time[i] = millis();
    switch (i) {
      case DEB_GPS_PVT:
        printParsePVT(gps.debug_par.debug_en_head[i]);
        break;

      case DEB_GPS_RELPOSNED:
        printParseRELPOSNED(gps.debug_par.debug_en_head[i]);
        break;
    }
  }
}

/*-------------------------- DEBUG MOTORS --------------------------*/
void debugMot() {
  printCAN_Buffer(DESCR_OFF, 100, UPDATE_CD_OFF, UPDATE_CS_OFF, UPDATE_DD_OFF, UPDATE_SS_OFF, HEAD_ON);
  // printCAN_Status(DESCR_ON, 100, UPDATE_CD_OFF, UPDATE_CS_OFF, UPDATE_DD_OFF, UPDATE_SS_OFF, HEAD_ON);
}

/*-------------------------- DEBUG BMS -----------------------------*/
void debugBms() {
  for (uint8_t i = 0; i < sizeof(bms.debug_par.debug_enable); i++) {
    if (bms.debug_par.debug_enable[i]) {
      debug_switch_bms(i, bms.debug_par.debug_timer[i]);
    }
  }
}

void debug_switch_bms(uint8_t i, uint16_t debug_timer) {
  if ((millis() - bms.debug_time[i]) > debug_timer) {
    bms.debug_time[i] = millis();
    switch (i) {
      case DEB_BMS_VCELL:
        printBmsVcell(bms.debug_par.debug_en_descr[i], bms.debug_par.debug_en_head[i]);
        break;

      case DEB_BMS_BASIC:
        printBmsBasic(bms.debug_par.debug_en_descr[i], bms.debug_par.debug_en_head[i]);
        break;

      case DEB_BMS_EEPROM:
        printBmsEeprom(bms.debug_par.debug_en_descr[i], bms.debug_par.debug_en_head[i]);
        break;
    }
  }
}

/*-------------------------- DEBUG ECHO ---------------------------------*/
void debugEcho() {
  if (echo_debug_par.debug_en) {
    if (millis() - time_print_echo > echo_debug_par.timer_print_echo) {
      time_print_echo = millis();
      printEcho(echo_debug_par.descr_en, echo_debug_par.head_en);
    }
  }
}

/*-------------------------- DEBUG MPPT ---------------------------------*/
void debugMPPT() {
  if (mppt_debug_par.debug_en) {
    if (millis() - time_print_mppt > mppt_debug_par.timer_print_mppt) {
      time_print_mppt = millis();
      printMPPT(mppt_debug_par.descr_en, mppt_debug_par.head_en);
    }
  }
}

/*-------------------------- DEBUG MISC ---------------------------------*/
void debugJS() {
  if (js_debug_par.debug_en) {
    if (millis() - time_print_js > js_debug_par.timer_print_js) {
      time_print_js = millis();
      printJS(js_debug_par.descr_en, js_debug_par.head_en, js_debug_par.index);
    }
  }
}

uint16_t timer_erpm_deb = 100;
uint32_t time_erpm_deb = 0;

void debugERPM() {
  if (millis() - time_erpm_deb > timer_erpm_deb) {
    time_erpm_deb = millis();
    Serialprint(motorSetData.erpm_driving[INDEX_MOT_DD]);
    Serialprint(F(","));
    Serialprint(motorSetData.erpm_driving[INDEX_MOT_CD]);
    Serialprint(F(","));
    Serialprint(motorSetData.erpm_driving[INDEX_MOT_CS]);
    Serialprint(F(","));
    Serialprint(motorSetData.erpm_driving[INDEX_MOT_SS]);
    Serialprintln();
  }
}

void debugRadioChValue() {
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(radioChValue[i]);
    Serial.print(",");
  }
  Serial.println();
}
void debugJoyControl()
{
      Serialprint(controlCmdData.base_cmd);
        Serialprint(",");
      Serialprint(controlCmdData.boost_cmd);
       Serialprint(",");
      Serialprint(controlCmdData.diff_cmd);
       Serialprint(",");
      Serialprint(controlCmdData.wheel_cmd);
       Serialprintln(",");
}
