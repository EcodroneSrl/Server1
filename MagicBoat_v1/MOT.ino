unsigned long time_set = 0;
unsigned long timer_set = 10;
unsigned long time_CANBuffer = 0;
unsigned long time_CAN_stat = 0;

uint8_t motorAddr[4] = { MOTOR_DD, MOTOR_SS, MOTOR_CD, MOTOR_CS };
uint8_t pinEnCan = 38;

void motorInit() {
  // pinMode(pinEnCan,INPUT);
  motors[INDEX_MOT_DD].initCAN();  // Da chiamare solo una volta perchè setta la porta CAN sulla Teensy

  for (uint8_t i = 0; i < sizeof(motors) / sizeof(MagicBoat_VESC_CAN); i++) {
    motors[i].begin(motorAddr[i]);  // Inizializzo tutti i motori
    motors[i].setTelTime(100);      // Setto il tempo di richiesta della telemetria per ogni motore
  }
}

void miscApplyParam(float base_cmd, float boost_cmd, float diff_cmd, float wheel_cmd, uint8_t boatNavMode, uint8_t boatNavSubMode) {
  int16_t p_base = 1;
  int16_t p_boost = 1;
  int16_t p_diff = 1;
  int16_t p_wheel = 1;

  switch (boatNavMode) {
    case NAV_TEL:
      switch (boatNavSubMode) {
        case TEL_MODE_1:

          p_base = miscParamData.base;
          p_boost = miscParamData.boost;
          p_diff = miscParamData.motorControlData.kDiff;
          p_wheel = miscParamData.wheel;
          // miscInLinea(base_erpm * miscParamData.base,erpm_boost * miscParamData.boost,erpm_diff * miscParamData.motorControlData.max_min_diff,erpm_wheel * miscParamData.base);
          break;

        case TEL_MODE_2:
          // erpm_base & ThetaD
          p_base = miscParamData.base;
          p_boost = miscParamData.boost;
          p_wheel = 0;
          break;

        case TEL_MODE_3:
          p_boost = miscParamData.boost;
          p_diff = miscParamData.motorControlData.kDiff;
          p_wheel = miscParamData.wheel;
          break;

        case TEL_MODE_4:
          p_boost = miscParamData.boost;
          p_wheel = 0;
          break;
      }
      break;

    case NAV_AUT:
      switch (boatNavSubMode) {
        case AUT_MODE_1:
          // erpm base + ThetaD da waypoint
          break;

        case AUT_MODE_2:
          // TODO
          break;

        case AUT_MODE_3:
          // TODO
          break;

        case AUT_MODE_4:
          // TODO
          break;

        case AUT_MODE_5:
          // TODO
          break;
      }
      break;
  }

  int16_t base_erpm = base_cmd * p_base;
  int16_t erpm_boost = boost_cmd * p_boost;
  int16_t erpm_diff = diff_cmd * p_diff;
  int16_t erpm_wheel = wheel_cmd * p_wheel;

  miscInLinea(base_erpm, erpm_boost, erpm_diff, erpm_wheel);
}

void miscInLinea(int16_t base_erpm, int16_t erpm_boost, int16_t erpm_diff, int16_t erpm_wheel) {
  int16_t erpmSS = base_erpm + erpm_boost + erpm_diff * miscParamData.motorControlData.kDiff + erpm_wheel;
  int16_t erpmCS = base_erpm + erpm_boost + erpm_diff * ((miscParamData.motorControlData.kDiff) / 2);
  int16_t erpmCD = base_erpm + erpm_boost - erpm_diff * ((miscParamData.motorControlData.kDiff) / 2);
  int16_t erpmDD = base_erpm + erpm_boost - erpm_diff * miscParamData.motorControlData.kDiff - erpm_wheel;

  setMotorsErpm(erpmSS, erpmCS, erpmCD, erpmDD);
  // debugERPM();
}

// Parte di messaggio fisico di controllo motori su CAN bus

void motorDrive() {
  if (millis() - time_set > timer_set) {
    time_set += timer_set;
    switch (driveMode) {
      case MOTOR_ERPM_CMD3:
        erpmDriveMotors();
        break;

      case MOTOR_CURRENT_CMD3:
        currentDriveMotor();
        break;
    }
  }
}

void setMotorsErpm(int16_t erpmSS, int16_t erpmCS, int16_t erpmCD, int16_t erpmDD) {
  motorSetData.erpm_driving[INDEX_MOT_DD] = erpmDD;
  motorSetData.erpm_driving[INDEX_MOT_CD] = erpmCD;
  motorSetData.erpm_driving[INDEX_MOT_CS] = erpmCS;
  motorSetData.erpm_driving[INDEX_MOT_SS] = erpmSS;
}

void printMotorErpm() {
  for (uint8_t i = 0; i < sizeof(motorSetData.erpm_driving) / sizeof(motorSetData.erpm_driving[0]); i++) {
    Serialprint(motorSetData.erpm_driving[i]);
    Serialprint(",");
  }
  Serialprintln();
}

void erpmDriveMotors() {
  for (uint8_t motor_index = 0; motor_index < sizeof(motors) / sizeof(MagicBoat_VESC_CAN); motor_index++) {
    int16_t erpmLimited = erpmLimit(motorSetData.erpm_driving[motor_index], motor_index);
    erpmSetRawMotor(&motors[motor_index], erpmLimited, motor_index);
  }
}

int16_t erpmLimit(int16_t erpm, uint8_t motor_index) {
  if (erpm >= miscParamData.motorControlData.max_erpm) {
    erpm = miscParamData.motorControlData.max_erpm;
  }
  if (erpm <= -(miscParamData.motorControlData.max_erpm)) {
    erpm = -(miscParamData.motorControlData.max_erpm);
  }
  return erpm;
}

uint8_t erpmSetRawMotor(MagicBoat_VESC_CAN *motorPtr, int16_t erpm, uint8_t motor_index) {
  uint8_t ok = 0;
  if (!motorSetData.drive_off[motor_index]) {
    motorPtr->setERPM(erpm);
    ok = 1;
  }
  return ok;
}

void currentDriveMotor() {
  // TODO
}

uint32_t motorTime = 0;
void computeAllMotor() {
  motorTime = millis();
  for (uint8_t i = 0; i < sizeof(motors) / sizeof(MagicBoat_VESC_CAN); i++) {
    motors[i].updateCAN_Tel();
    // motors[i].updateCANStatus();
  }
  readMotorTel();
  motorTime = millis() - motorTime;
}

void readMotorTel() {
  CAN_message_t msg;
  if (motors[INDEX_MOT_DD].Can1.read(msg)) {
    for (uint8_t i = 0; i < sizeof(motors) / sizeof(MagicBoat_VESC_CAN); i++) {
      motors[i].parseLongBuffer(msg);
    }
  }
}

// Print personalizzabile dei messaggi di stato 1,2,3,4,5

void printCAN_Status(MagicBoat_VESC_CAN *motorPtr, uint8_t description, uint16_t timer_CAN_stat, uint8_t update_mot, uint8_t head) {
  if (update_mot == UPDATE_ON) {
    motorPtr->updateCANStatus();
  }

  if ((millis() - time_CAN_stat) > timer_CAN_stat) {
    time_CAN_stat = millis();

    if (description == DESCR_ON) {
      Serialprint(F("RPM:"));
    }
    Serialprint(motorPtr->RPM);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Curr:"));
    }
    Serialprint(motorPtr->current);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Duty:"));
    }
    Serialprint(motorPtr->duty);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Amp_h:"));
    }
    Serialprint(motorPtr->amp_hours);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Amp_h_ch:"));
    }
    Serialprint(motorPtr->amp_hours_charged);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Watt_h:"));
    }
    Serialprint(motorPtr->watt_hours);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Watt_h_ch:"));
    }
    Serialprint(motorPtr->watt_hours_charged);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Temp_FET:"));
    }
    Serialprint(motorPtr->temp_fet);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Temp_motor:"));
    }
    Serialprint(motorPtr->temp_motor);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Curr_in:"));
    }
    Serialprint(motorPtr->current_in);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("PID_pos:"));
    }
    Serialprint(motorPtr->pid_pos_now);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("Tacho:"));
    }
    Serialprint(motorPtr->tacho_value);
    Serialprint(F(","));
    if (description == DESCR_ON) {
      Serialprint(F("V_in:"));
    }
    Serialprint(motorPtr->v_in);
    Serialprint(F(","));

    if (head == HEAD_ON) {
      Serialprintln(F(","));
    } else {
      Serialprint(F(","));
    }
  }
}

// Test dei motori in ERPM con comando inviato ogni tot tempo
void testMotor() {
  int32_t rpms = 1000;
  // int16_t curr = 2000;
  uint8_t rpm = 1;
  if (millis() - time_set > timer_set) {
    time_set = millis();
    if (rpm) {
      for (uint8_t i = 0; i < sizeof(motors) / sizeof(MagicBoat_VESC_CAN); i++) {
        motors[i].setERPM(rpms);
      }
    } else {
      for (uint8_t i = 0; i < sizeof(motors) / sizeof(MagicBoat_VESC_CAN); i++) {
        // motors[i].setCurrent(curr);
      }
    }
  }
}

// Print personalizzabile del buffer più accurato

void printCAN_Buffer(uint8_t description, uint16_t timer_CANBuffer, uint8_t update_motCD, uint8_t update_motCS, uint8_t update_motDD, uint8_t update_motSS, uint8_t head) {
  if (update_motCD == UPDATE_ON) {
    motors[INDEX_MOT_CD].updateCAN_Tel();
  }
  if (update_motCS == UPDATE_ON) {
    motors[INDEX_MOT_CS].updateCAN_Tel();
  }
  if (update_motDD == UPDATE_ON) {
    motors[INDEX_MOT_DD].updateCAN_Tel();
  }
  if (update_motSS == UPDATE_ON) {
    motors[INDEX_MOT_SS].updateCAN_Tel();
  }

  if ((millis() - time_CANBuffer) > timer_CANBuffer) {
    time_CANBuffer = millis();

    /* ----- Print Motori ----- */
    for (uint8_t i = 0; i < sizeof(motors) / sizeof(MagicBoat_VESC_CAN); i++) {
      if (description == DESCR_ON) {
        Serialprint(F("ID:"));
      }
      Serialprint(motors[i].motorID);
      Serialprint(F(","));

      if (description == DESCR_ON) {
        Serialprint(F("RPM:"));
      }
      Serialprint(motors[i].bufferTelData.rpm);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Curr:"));
      }
      Serialprint(motors[i].bufferTelData.avg_motor_current);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Id:"));
      }
      Serialprint(motors[i].bufferTelData.i_D);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Iq:"));
      }
      Serialprint(motors[i].bufferTelData.i_Q);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("pid_pos:"));
      }
      Serialprint(motors[i].bufferTelData.pid_pos);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Mostemp1:"));
      }
      Serialprint(motors[i].bufferTelData.mostemp1);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Mostemp2:"));
      }
      Serialprint(motors[i].bufferTelData.mostemp2);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Mostemp3:"));
      }
      Serialprint(motors[i].bufferTelData.mostemp3);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Duty:"));
      }
      Serialprint(motors[i].bufferTelData.duty_cycle);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Amp_h:"));
      }
      Serialprint(motors[i].bufferTelData.amp_h);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Amp_h_ch:"));
      }
      Serialprint(motors[i].bufferTelData.amp_h_ch);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Watt_h:"));
      }
      Serialprint(motors[i].bufferTelData.watt_h);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Watt_h_ch:"));
      }
      Serialprint(motors[i].bufferTelData.watt_h_ch);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Temp_FET:"));
      }
      Serialprint(motors[i].bufferTelData.temp_fet1);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Temp_motor:"));
      }
      Serialprint(motors[i].bufferTelData.temp_motor1);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Curr_in:"));
      }
      Serialprint(motors[i].bufferTelData.avg_input_current);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Odo:"));
      }
      Serialprint(motors[i].bufferTelData.odometer);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("Odo abs:"));
      }
      Serialprint(motors[i].bufferTelData.odometer_abs);
      Serialprint(F(","));
      if (description == DESCR_ON) {
        Serialprint(F("V:"));
      }
      Serialprint(motors[i].bufferTelData.voltage);
      Serialprint(F(","));

      if (head == HEAD_ON) {
        Serialprintln(F(","));
      } else {
        Serialprint(F(","));
      }
    }
  }
}
