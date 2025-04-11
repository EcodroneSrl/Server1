uint8_t AvvioFRAM = 3;  // Se cambio questo numero il sistema riparte con le variabili di default e le rimette nella EEPROM
uint8_t AvvioFRAMRead = 0;

uint32_t AvvioCellFRAM = 1;

struct __attribute__((packed)) cellFRAM {
  uint8_t startStruct = 10;
  uint16_t NmissionParam = startStruct;
  uint16_t NmissionCycle = NmissionParam + sizeof(missionParamDataCurrent);
  uint16_t NmissionCycleNum = NmissionCycle + sizeof(missionCycle);
  uint16_t NgoBack = NmissionCycleNum + sizeof(missionCycleNum);
  uint16_t NindexWPNow = NgoBack + sizeof(goBack);
  uint16_t Nindex_step = NindexWPNow + sizeof(indexWPNow);
  uint16_t NrifLatTrue = Nindex_step + sizeof(index_step);
  uint16_t NrifLonTrue = NrifLatTrue + sizeof(rifLatTrue);
  uint16_t NrifLatMissionOld = NrifLonTrue + sizeof(rifLonTrue);
  uint16_t NrifLonMissionOld = NrifLatMissionOld + sizeof(rifLatMissionOld);
  uint16_t NTetaD = NrifLonMissionOld + sizeof(rifLonMissionOld);
  uint16_t Nmission_active = NTetaD + sizeof(TetaD);
} posFRAM;

void FRAMInit() {
  if (FRAM.begin()) {
    FRAM.read(AvvioCellFRAM, AvvioFRAMRead);

    if (AvvioFRAMRead != AvvioFRAM) {
      toFRAMAll();
      FRAM.write(AvvioCellFRAM, AvvioFRAM);
    }

    fromFRAMAll();

    restoreNavState(index_step);
  }
}

void toFRAMAll() {
  setFRAMmissionParam();
  setFRAMmissionCycle();
  setFRAMmissionCycleNum();
  setFRAMgoBack();
  setFRAMindexWPNow();
  setFRAMindexStep();
  setFRAMrifTrue();
  setFRAMrifOld();
  setFRAMTetaD();
  setFRAMmissionActive();
}

void fromFRAMAll() {
  getFRAMmissionParam();
  getFRAMmissionCycle();
  getFRAMmissionCycleNum();
  getFRAMgoBack();
  getFRAMindexWPNow();
  getFRAMindexStep();
  getFRAMrifTrue();
  if (goBack) {
    getFRAMrifOld();
  }
  getFRAMTetaD();
  getFRAMmissionActive();
}

/*++++++++++++++ NAV +++++++++++++++*/

//Funzioni di set

void setFRAMmissionParam() {
  to_fram(missionParamDataCurrent, posFRAM.NmissionParam);
}

void setFRAMmissionCycle() {
  to_fram(missionCycle, posFRAM.NmissionCycle);
  //Serial.println("Set missionCycle");
}

void setFRAMmissionCycleNum() {
  to_fram(missionCycleNum, posFRAM.NmissionCycleNum);
  //Serial.println("Set missionCycleNum");
}

void setFRAMgoBack() {
  to_fram(goBack, posFRAM.NgoBack);
  //Serial.println("Set goBack");
}

void setFRAMindexWPNow() {
  to_fram(indexWPNow, posFRAM.NindexWPNow);
  //Serial.println("Set indexWPNow");
}

void setFRAMindexStep() {
  to_fram(index_step, posFRAM.Nindex_step);
  //Serial.println("Set index_step");
}

void setFRAMrifTrue() {
  to_fram(rifLatTrue, posFRAM.NrifLatTrue);
  to_fram(rifLonTrue, posFRAM.NrifLonTrue);
  //Serial.println("Set coordinate True");
}

void setFRAMrifOld() {
  to_fram(rifLatMissionOld, posFRAM.NrifLatMissionOld);
  to_fram(rifLonMissionOld, posFRAM.NrifLonMissionOld);
  //Serial.println("Set coordinate Old");
}

void setFRAMTetaD() {
  to_fram(TetaD, posFRAM.NTetaD);
  //Serial.println("Set TetaD");
}

void setFRAMmissionActive() {
  to_fram(mission_active, posFRAM.Nmission_active);
  //Serial.println("Set mission_active");
}

//Funzioni di get

void getFRAMmissionParam() {
  from_fram(missionParamDataCurrent, posFRAM.NmissionParam);
}

void getFRAMmissionCycle() {
  from_fram(missionCycle, posFRAM.NmissionCycle);
}

void getFRAMmissionCycleNum() {
  from_fram(missionCycleNum, posFRAM.NmissionCycleNum);
}

void getFRAMgoBack() {
  from_fram(goBack, posFRAM.NgoBack);
}

void getFRAMindexWPNow() {
  from_fram(indexWPNow, posFRAM.NindexWPNow);
}

void getFRAMindexStep() {
  from_fram(index_step, posFRAM.Nindex_step);
}

void getFRAMrifTrue() {
  from_fram(rifLatTrue, posFRAM.NrifLatTrue);
  from_fram(rifLonTrue, posFRAM.NrifLonTrue);
}

void getFRAMrifOld() {
  from_fram(rifLatMissionOld, posFRAM.NrifLatMissionOld);
  from_fram(rifLonMissionOld, posFRAM.NrifLonMissionOld);
}

void getFRAMTetaD() {
  from_fram(TetaD, posFRAM.NTetaD);
}

void getFRAMmissionActive() {
  from_fram(mission_active, posFRAM.Nmission_active);
}

/*++++++++++++ END +++++++++++++++*/

/*************** FRAM BASIC FUNCTIONS ***********************/

template<typename T>
void to_fram(T &value, uint32_t address) {
  FRAM.write(address, value);
  wdtReset();
  return;
}

template<typename T>
void from_fram(T &value, uint32_t address) {
  FRAM.read(address, value);
  wdtReset();
  return;
}

/****************** PRINT FRAM *************************/

void printFRAM() {
  Serialprint(F("getFRAMmissionParam"));
  printMissionParamData(&missionParamDataCurrent);
  Serialprint(F("missionCycle:"));
  Serialprint(missionCycle);
  Serialprint(F(","));
  Serialprint(F("missionCycleNum:"));
  Serialprint(missionCycleNum);
  Serialprint(F(","));
  Serialprint(F("goBack:"));
  Serialprint(goBack);
  Serialprint(F(","));
  Serialprint(F("indexWPNow:"));
  Serialprint(indexWPNow);
  Serialprint(F(","));
  Serialprint(F("index_step:"));
  Serialprint(index_step);
  Serialprint(F(","));
  Serialprint(F("rifLatMission:"));
  Serialprint(rifLatMission);
  Serialprint(F(","));
  Serialprint(F("rifLonMission:"));
  Serialprint(rifLonMission);
  Serialprint(F(","));
  Serialprint(F("rifLatMissionOld:"));
  Serialprint(rifLatMissionOld);
  Serialprint(F(","));
  Serialprint(F("rifLonMissionOld:"));
  Serialprint(rifLonMissionOld);
  Serialprint(F(","));
  Serialprint(F("rifLatTrue:"));
  Serialprint(rifLatTrue);
  Serialprint(F(","));
  Serialprint(F("rifLonTrue:"));
  Serialprint(rifLonTrue);
  Serialprint(F(","));
  Serialprint(F("TetaD:"));
  Serialprint(TetaD);
  Serialprint(F(","));
  Serialprint(F("mission_active:"));
  Serialprint(mission_active);
  Serialprintln();
}
