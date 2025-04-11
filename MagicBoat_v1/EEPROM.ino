uint8_t Avvio = 13;  // Se cambio questo numero il sistema riparte con le variabili di default e le rimette nella EEPROM
uint8_t AvvioPower = 3;

uint8_t AvvioCell = 1;
uint8_t AvvioCellPower = 2;

struct __attribute__((packed)) cellEprom {
  uint16_t startStruct = 10;

  uint16_t Nimu_cal_values = startStruct;
  uint16_t Nimu_update_par = Nimu_cal_values + sizeof(imu.cal_values);
  uint16_t Nimu_debug_par = Nimu_update_par + sizeof(imu.update_par);
  uint16_t Nbms_start_par = Nimu_debug_par + sizeof(imu.debug_par);
  uint16_t Nbms_debug_par = Nbms_start_par + sizeof(bms.start_par);
  uint16_t Nloop_en_data = Nbms_debug_par + sizeof(bms.debug_par);
  uint16_t Nip = Nloop_en_data + sizeof(loop_en_data);
  uint16_t Ntcp_port = Nip + sizeof(SerialRWSocket.ip);

  // Controllo
  uint16_t NtelInputDev = Ntcp_port + sizeof(tcp_port);
  uint16_t NgpsSource = NtelInputDev + sizeof(telInputDev);
  uint16_t NthetaBSource = NgpsSource + sizeof(gpsSource);
  uint16_t NboatNavMode = NthetaBSource + sizeof(thetaBSource);
  uint16_t NboatNavSubMode = NboatNavMode + sizeof(boatNavMode);
  uint16_t NcontrolModeRoute = NboatNavSubMode + sizeof(boatNavSubMode);
  uint16_t NpidTheta = NcontrolModeRoute + sizeof(controlModeRoute);
  uint16_t NcontrolModeVel = NpidTheta + sizeof(pidTheta);
  uint16_t NpidVel = NcontrolModeVel + sizeof(controlModeVel);
  uint16_t NmiscParamData = NpidVel + sizeof(pidVel);
  uint16_t NequipData = NmiscParamData + sizeof(miscParamData);

  // ON-OFF Potenza
  uint16_t Npower_en = NequipData + sizeof(equipData);
  // Variabile per salvare lo stato si sleep
  uint16_t Nsleep_on = Npower_en + sizeof(power_en);
} pos;

void EEPROMInit() {
  delay(100); // Per far avviare bene la EEPROM
  if (EEPROM.read(AvvioCell) != Avvio) {
    toEpromAll();
    EEPROM.write(AvvioCell, Avvio);
  }

  if (EEPROM.read(AvvioCellPower) != AvvioPower) {
    setPowerOnOff();
    EEPROM.write(AvvioCellPower, AvvioPower);
  }

  fromEEpromAll();
}

void toEpromAll() {
  Serial.println("toEpromAll()");
  setIP();
  setIMU();
  setBMS();
  setLoopEn();
  setControl();
  setEquipment();
}

void fromEEpromAll() {
  Serial.println("fromEEpromAll()");
  getIP();
  getIMU();
  getBMS();
  getLoopEn();
  getControl();
  getEquipment();
  getPowerOnOff();
}

/*++++++++++++++ IMU +++++++++++++++*/

//Funzioni di set

void setIMU() {
  setCalIMU();
  setConfigIMU();
  setDebConfigIMU();
}

void setCalIMU() {
  to_eeprom(&imu.cal_values, sizeof(imu.cal_values), pos.Nimu_cal_values);
}

void setConfigIMU() {
  to_eeprom(&imu.update_par, sizeof(imu.update_par), pos.Nimu_update_par);
}

void setDebConfigIMU() {
  to_eeprom(&imu.debug_par, sizeof(imu.debug_par), pos.Nimu_debug_par);
}

//Funzioni di get

void getIMU() {
  getCalIMU();
  getConfigIMU();
  getDebConfigIMU();
}

void getCalIMU() {
  from_eeprom(&imu.cal_values, sizeof(imu.cal_values), pos.Nimu_cal_values);
}

void getConfigIMU() {
  from_eeprom(&imu.update_par, sizeof(imu.update_par), pos.Nimu_update_par);
}

void getDebConfigIMU() {
  from_eeprom(&imu.debug_par, sizeof(imu.debug_par), pos.Nimu_debug_par);
}

/*++++++++++++ END IMU +++++++++++++++*/

/*++++++++++++++ BMS +++++++++++++++++*/
//Funzioni di set
void setBMS() {
  setConfigBMS();
  setDebConfigBMS();
}

void setConfigBMS() {
  to_eeprom(&bms.start_par, sizeof(bms.start_par), pos.Nbms_start_par);
}

void setDebConfigBMS() {
  to_eeprom(&bms.debug_par, sizeof(bms.debug_par), pos.Nbms_debug_par);
}

//Funzioni di get

void getBMS() {
  getConfigBMS();
  getDebConfigBMS();
}

void getConfigBMS() {
  from_eeprom(&bms.start_par, sizeof(bms.start_par), pos.Nbms_start_par);
}

void getDebConfigBMS() {
  from_eeprom(&bms.debug_par, sizeof(bms.debug_par), pos.Nbms_debug_par);
}

/*++++++++++++ END BMS +++++++++++++++*/

/*++++++++++++++ LOOP EN +++++++++++++++++*/

void setLoopEn() {
  to_eeprom(&loop_en_data, sizeof(loop_en_data), pos.Nloop_en_data);
}

void getLoopEn() {
  from_eeprom(&loop_en_data, sizeof(loop_en_data), pos.Nloop_en_data);
}

/*++++++++++++ END LOOP EN +++++++++++++++*/

/*++++++++++++++ CONTROL +++++++++++++++++*/
//Funzioni di set

void setControl() {
  setRemoteControl();
  setNavGPS();
  setNavHead();
  setNavMode();
  setControlTeta();
  setControlVel();
  setMiscParam();
}

void setRemoteControl() {
  to_eeprom(&telInputDev, sizeof(telInputDev), pos.NtelInputDev);
}

void setNavGPS() {
  to_eeprom(&gpsSource, sizeof(gpsSource), pos.NgpsSource);
}

void setNavHead() {
  to_eeprom(&thetaBSource, sizeof(thetaBSource), pos.NthetaBSource);
}

void setNavMode() {
  to_eeprom(&boatNavMode, sizeof(boatNavMode), pos.NboatNavMode);
  to_eeprom(&boatNavSubMode, sizeof(boatNavSubMode), pos.NboatNavSubMode);
}

void setControlTeta() {
  to_eeprom(&controlModeRoute, sizeof(controlModeRoute), pos.NcontrolModeRoute);
  to_eeprom(&pidTheta, sizeof(pidTheta), pos.NpidTheta);
}

void setControlVel() {
  to_eeprom(&controlModeVel, sizeof(controlModeVel), pos.NcontrolModeVel);
  to_eeprom(&pidVel, sizeof(pidVel), pos.NpidVel);
}

void setMiscParam() {
  to_eeprom(&miscParamData, sizeof(miscParamData), pos.NmiscParamData);
}

//Funzioni di get

void getControl() {
  getRemoteControl();
  getNavGPS();
  getNavHead();
  getNavMode();
  getControlTeta();
  getControlVel();
  getMiscParam();
}

void getRemoteControl() {
  from_eeprom(&telInputDev, sizeof(telInputDev), pos.NtelInputDev);
}

void getNavGPS() {
  from_eeprom(&gpsSource, sizeof(gpsSource), pos.NgpsSource);
}

void getNavHead() {
  from_eeprom(&thetaBSource, sizeof(thetaBSource), pos.NthetaBSource);
}

void getNavMode() {
  from_eeprom(&boatNavMode, sizeof(boatNavMode), pos.NboatNavMode);
  from_eeprom(&boatNavSubMode, sizeof(boatNavSubMode), pos.NboatNavSubMode);
}

void getControlTeta() {
  from_eeprom(&controlModeRoute, sizeof(controlModeRoute), pos.NcontrolModeRoute);
  from_eeprom(&pidTheta, sizeof(pidTheta), pos.NpidTheta);
}

void getControlVel() {
  from_eeprom(&controlModeVel, sizeof(controlModeVel), pos.NcontrolModeVel);
  from_eeprom(&pidVel, sizeof(pidVel), pos.NpidVel);
}

void getMiscParam() {
  from_eeprom(&miscParamData, sizeof(miscParamData), pos.NmiscParamData);
}

/*++++++++++++ END CONTROL +++++++++++++++*/

/*++++++++++++ EQUIPMENT +++++++++++++++++*/

void setEquipment() {
  to_eeprom(&equipData, sizeof(equipData), pos.NequipData);
}

void getEquipment() {
  from_eeprom(&equipData, sizeof(equipData), pos.NequipData);
}

/*++++++++++ END EQUIPMENT +++++++++++++*/

/*++++++++++++++ IP +++++++++++++++++*/

void setIP() {
  to_eeprom(&SerialRWSocket.ip, sizeof(SerialRWSocket.ip), pos.Nip);
  to_eeprom(&tcp_port, sizeof(tcp_port), pos.Ntcp_port);
}

void getIP() {
  from_eeprom(&SerialRWSocket.ip, sizeof(SerialRWSocket.ip), pos.Nip);
  from_eeprom(&tcp_port, sizeof(tcp_port), pos.Ntcp_port);
}

/*++++++++++ END IP +++++++++++++++++*/

/*++++++++++++++ POWER ON/OFF +++++++++++++++++*/

void setPowerOnOff() {
  to_eeprom(&power_en, sizeof(power_en), pos.Npower_en);
}

void getPowerOnOff() {
  from_eeprom(&power_en, sizeof(power_en), pos.Npower_en);
}

/*++++++++++ END POWER ON/OFF +++++++++++++++++*/

/*************** EEPROM BASIC FUNCTIONS ***********************/

void to_eeprom(void *ptr, uint32_t dim, uint32_t start_position) {
  unsigned char mom;
  for (uint16_t k = 0; k < dim; k++) {
    mom = ((unsigned char *)ptr)[k];
    EEPROM.write(start_position + k, mom);
  }
  wdtReset();
  return;
}

void from_eeprom(void *ptr, uint32_t dim, uint32_t start_position) {
  unsigned char mom;
  uint16_t k;
  for (k = 0; k < dim; k++) {
    mom = EEPROM.read(start_position + k);
    ((unsigned char *)ptr)[k] = mom;
  }
  wdtReset();
  return;
}
