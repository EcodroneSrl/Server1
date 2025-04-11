void flashInit() {
  if (!extFlash.begin()) {
    Serialprint(F("Error starting QSPI"));
    Serialprintln();
    delay(1);
  } else {
    delay(1);
    scheduleRead();
  }
}

void createMissionFile(char *flashFilenameNow, missionParam *missionParamPtr) {
  createFlashFolder(missionParamPtr);

  if (extFlash.exists(flashFilenameNow)) {
    Serialprint(F("Rimosso!"));
    Serialprintln();
    extFlash.remove(flashFilenameNow);
  }
  flashFile = extFlash.open(flashFilenameNow, FILE_WRITE);
  flashFile.close();
}

void createFlashFolder(missionParam *missionParamPtr) {
  if (!extFlash.exists(missionParamPtr->idMission)) {
    extFlash.mkdir(missionParamPtr->idMission);
  }
}

void createFlashFilename(missionParam *missionParamPtr) {
  char nMission_ch[5];
  sprintf(nMission_ch, "%u", missionParamPtr->nMission);
  memset(flashFilename, 0, sizeof(flashFilename));
  char slash[] = "/";
  strcpy(flashFilename, slash);
  strcat(flashFilename, missionParamPtr->idMission);
  strcat(flashFilename, slash);
  strcat(flashFilename, nMission_ch);
  strcat(flashFilename, ".bin");
}

void saveHeader(char *flashFilenameNow, missionParam *missionParamPtr) {
  flashFile = extFlash.open(flashFilenameNow, FILE_WRITE);
  flashFile.write(missionParamPtr, sizeof(missionParam));
  flashFile.close();
}

void saveWP(char *flashFilenameNow, uint16_t waypointIndex, wayPoint *wayPointPtr) {
  flashFile = extFlash.open(flashFilenameNow, FILE_WRITE);
  flashFile.seek(sizeof(missionParamData) + ((waypointIndex) * sizeof(wayPointDataCksum)), SeekSet);
  flashFile.write(wayPointPtr, sizeof(wayPoint));
  flashFile.close();
}

void cksumTestFlash(char *flashFilenameNow) {
  File flashFileRead = extFlash.open(flashFilenameNow, FILE_READ);
  uint32_t mission_cksum_read = 0;

  readMissionParamData(&missionParamDataCksum, sizeof(missionParamDataCksum), flashFilenameNow);

  mission_cksum_read += sumStructBytes(&missionParamDataCksum, sizeof(missionParamDataCksum));

  for (uint16_t k = 0; k < missionParamDataCksum.total_mission_nWP; k++) {

    readMissionWaypointData(&wayPointDataCksum, sizeof(wayPointDataCksum), flashFilenameNow, k);
    mission_cksum_read += sumStructBytes(&wayPointDataCksum, sizeof(wayPointDataCksum));
  }

  if (mission_cksum_read != mission_cksum) {
    Serialprint(F("checksum diverso!!"));
    Serialprintln();
    extFlash.remove(flashFilenameNow);
  }
  flashFileRead.close();
}

bool readMissionParamData(missionParam *missionParamPtr, uint32_t dim, char *flashFilenameNow) {
  if (!extFlash.exists(flashFilenameNow)) return false;

  unsigned char mom;
  File flashFileRead = extFlash.open(flashFilenameNow, FILE_READ);
  for (uint16_t i = 0; i < dim; i++) {
    mom = flashFileRead.read();
    ((unsigned char *)missionParamPtr)[i] = mom;
  }
  flashFileRead.close();

  Serialprint(F(" PARAM_DATA: "));
  Serialprint(missionParamPtr->idMission);  // Mission ID
  Serialprint(F(","));
  Serialprint(missionParamPtr->nMission);  // Mission number
  Serialprint(F(","));
  Serialprint(missionParamPtr->total_mission_nWP);  // Number of waypoints of current mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->wpStart);  // Start waypoint of current mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->cycles);  // How many execution cycles of the current mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->wpEnd);  // Final waypoint of current mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->NMmode);  // What to do next (0,1,2)
  Serialprint(F(","));
  Serialprint(missionParamPtr->NMnum);  // Number of the next mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->NMStartInd);  // Start waypoint of next mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->idMissionNext);  // String with the path of the mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->standRadius);  // String with the path of the mission
  Serialprintln();
  return true;
}

void printMissionParamData(missionParam *missionParamPtr) {
  Serialprint(F(" PARAM_DATA: "));
  Serialprint(missionParamPtr->idMission);  // Mission ID
  Serialprint(F(","));
  Serialprint(missionParamPtr->nMission);  // Mission number
  Serialprint(F(","));
  Serialprint(missionParamPtr->total_mission_nWP);  // Number of waypoints of current mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->wpStart);  // Start waypoint of current mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->cycles);  // How many execution cycles of the current mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->wpEnd);  // Final waypoint of current mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->NMmode);  // What to do next (0,1,2)
  Serialprint(F(","));
  Serialprint(missionParamPtr->NMnum);  // Number of the next mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->NMStartInd);  // Start waypoint of next mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->idMissionNext);  // String with the path of the mission
  Serialprint(F(","));
  Serialprint(missionParamPtr->standRadius);  // String with the path of the mission
  Serialprintln();
}

bool readMissionWaypointData(wayPoint *wayPointPtr, uint32_t dim, char *flashFilenameNow, uint16_t waypointIndex) {
  if (!extFlash.exists(flashFilenameNow)) return false;

  unsigned char mom;
  File flashFileRead = extFlash.open(flashFilenameNow, FILE_READ);
  flashFileRead.seek(sizeof(missionParamDataCksum) + ((waypointIndex) * sizeof(wayPointDataCksum)), SeekSet);
  for (uint16_t i = 0; i < dim; i++) {
    mom = flashFileRead.read();
    ((unsigned char *)wayPointPtr)[i] = mom;
  }
  flashFileRead.close();

  Serialprint(F(" WP"));
  Serialprint(waypointIndex);
  Serialprint(F(": "));
  Serialprint(wayPointPtr->Nmissione);
  Serialprint(F(","));
  Serialprint(wayPointPtr->IndexWP);
  Serialprint(F(","));
  Serialprint(wayPointPtr->Latitude, 6);
  Serialprint(F(","));
  Serialprint(wayPointPtr->Longitude, 6);
  Serialprint(F(","));
  Serialprint(wayPointPtr->NavMode);
  Serialprint(F(","));
  Serialprint(wayPointPtr->PointType);
  Serialprint(F(","));
  Serialprint(wayPointPtr->MonitoringOp);
  Serialprint(F(","));
  Serialprint(wayPointPtr->ArriveMode);
  Serialprint(F(","));
  Serialprint(wayPointPtr->WaypointRadius, 6);
  Serialprintln();
  return true;
}

//*********** MISSION SCHEDULE FUNCTIONS ***********

void scheduleWrite(schedule *schedulePtr) {
  schedule_cksum = 0;
  schedule_cksum += sumStructBytes(schedulePtr, sizeof(schedule));

  File scheduleFile = extFlash.open("schedule.bin", FILE_WRITE);

  if (scheduleFile) {
    scheduleFile.seek((schedulePtr->schedule_index) * sizeof(schedule), SeekSet);
    scheduleFile.write(schedulePtr, sizeof(schedule));
  }
  scheduleFile.close();

  unsigned char mom;
  scheduleFile = extFlash.open("schedule.bin", FILE_READ);
  scheduleFile.seek((schedulePtr->schedule_index) * sizeof(schedule), SeekSet);
  for (uint16_t i = 0; i < sizeof(schedule); i++) {
    mom = scheduleFile.read();
    ((unsigned char *)&scheduleDataCksum)[i] = mom;
  }
  scheduleFile.close();

  uint32_t schedule_cksum_read = 0;
  schedule_cksum_read += sumStructBytes(&scheduleDataCksum, sizeof(scheduleDataCksum));

  if (schedule_cksum_read != schedule_cksum) {
    Serialprint(F("checksum diverso!!!"));
    Serialprintln();
  }
}

void scheduleRead() {
  File scheduleFile = extFlash.open("schedule.bin", FILE_READ);
  uint16_t nByteNow = scheduleFile.available();
  uint16_t nSchedule = nByteNow / sizeof(schedule);
  uint16_t schedule_count = 0;
  unsigned char mom;
  deltaTime = 0xFFFFFFFF;
  for (uint16_t j = 0; j < nSchedule; j++) {
    scheduleFile.seek(j * sizeof(schedule), SeekSet);
    for (uint16_t i = 0; i < sizeof(schedule); i++) {
      mom = scheduleFile.read();
      ((unsigned char *)&scheduleData)[i] = mom;
    }
    if (debug_schedule) {
      printSchedule(&scheduleData);
    }
    if (scheduleData.isEnabled) {
      schedule_count++;
      scheduleNextMission();
    }
  }
  scheduleFile.close();

  if (schedule_count) {
    // Serialprint(F("Next Mission is "));
    printSchedule(&scheduleDataNext);
  } else {
    // Serialprint(F("No scheduled missions\n"));
  }
  DateNextMission = date_to_FAT32(scheduleDataNext.year, scheduleDataNext.month, scheduleDataNext.day, scheduleDataNext.hour, scheduleDataNext.min);
  debug_schedule = 0;
}

void scheduleNextMission() {
  uint32_t DateNow = gps.Date; /*date_to_FAT32(gps.pvt1.year,gps.pvt1.month,gps.pvt1.day,gps.pvt1.hour,gps.pvt1.min);*/
  uint32_t DateMission = date_to_FAT32(scheduleData.year, scheduleData.month, scheduleData.day, scheduleData.hour, scheduleData.min);
  uint32_t deltaTimeNow = DateMission - DateNow;
  if (deltaTimeNow <= deltaTime) {
    deltaTime = deltaTimeNow;
    combinaStruct(&scheduleDataNext, &scheduleData, sizeof(scheduleData));
  }
}

uint32_t nextMissionTime = 0;
void timerNextMission() {
  nextMissionTime = millis();
  if ((gps.Date >= DateNextMission) && (DateNextMission != 0)) {
    updateCurrentMission(scheduleDataNext.nMission, scheduleDataNext.idMission);
    updateMissions();
    scheduleRead();
    mission_active = 1;
    setFRAMmissionActive();
  }
  nextMissionTime = millis() - nextMissionTime;
}

void updateCurrentMission(uint16_t nMission, char *idMission) {
  indexWPNow = 0;
  setFRAMindexWPNow();
  missionCycle = 0;
  setFRAMmissionCycle();  // metto in FRAM
  goBack = 0;
  setFRAMgoBack();  // Metto in FRAM

  char nMission_ch[5];
  sprintf(nMission_ch, "%u", nMission);
  strcpy(flashFilename, idMission);
  strcat(flashFilename, "/");
  strcat(flashFilename, nMission_ch);
  strcat(flashFilename, ".bin");
  Serialprintln(flashFilename);

  readMissionParamData(&missionParamDataCurrent, sizeof(missionParamDataCurrent), flashFilename);
  setFRAMmissionParam();  // Metto l'header della missione in FRAM

  missionCycleNum = missionParamDataCurrent.cycles;
  setFRAMmissionCycleNum();  // Metto in FRAM

  RaggioStand = save_stand_radius(&missionParamDataCurrent);
  save_new_wp(&missionParamDataCurrent, indexWPNow);
  rifLonMissionOld = save_wp_lon(&wayPointData);
  rifLatMissionOld = save_wp_lat(&wayPointData);
  indexWPNow++;
  setFRAMindexWPNow();
  save_new_wp(&missionParamDataCurrent, indexWPNow);
 // Serial.println("cane");
  computeNewPoint(rifLatMission, rifLonMission, rifLatMissionOld, rifLonMissionOld, interStep, index_step, arriveModeNow);
}

uint32_t save_stand_radius(missionParam *ptr) {
  float tempRadius = (ptr->standRadius) * 1000000.0;
  // uint32_t stand_radius = (uint32_t)((ptr->standRadius)*1000000.0);
  uint32_t stand_radius = tempRadius;
  return stand_radius;
}

void updateMissions() {
  combinaStruct(&scheduleData, &scheduleDataNext, sizeof(scheduleData));
  if (scheduleData.isRepeated > 0) {
    scheduleData.isRepeated -= 1;
  } else if (scheduleData.isRepeated == 0) {
    scheduleData.isEnabled = 0;
  }

  addDateTimeInterval(
    scheduleData.year, scheduleData.month, scheduleData.day, scheduleData.hour, scheduleData.min,
    scheduleData.repMonths, scheduleData.repDays, scheduleData.repHours, scheduleData.repMins);

  scheduleData.min = dateTimeNew.min;
  scheduleData.hour = dateTimeNew.hour;
  scheduleData.day = dateTimeNew.day;
  scheduleData.month = dateTimeNew.month;
  scheduleData.year = dateTimeNew.year;

  scheduleWrite(&scheduleData);
}

void addDateTimeInterval(
  uint16_t year,
  uint8_t month,
  uint8_t day,
  uint8_t hour,
  uint8_t min,

  uint8_t repmonth,
  uint8_t repday,
  uint8_t rephour,
  uint8_t repmin) {
  memset(&dateTimeNew, 0, sizeof(dateTimeNew));

  dateTimeNew.min += min + repmin;
  checkMinute();

  dateTimeNew.hour += hour + rephour;
  checkHour();

  dateTimeNew.day += day + repday;
  checkDay(month, year);

  dateTimeNew.month += month + repmonth;
  checkMonth();

  dateTimeNew.year += year;
}

void checkMonth() {
  if (dateTimeNew.month > 12) {
    uint8_t nYears = floor((dateTimeNew.month) / 12);
    dateTimeNew.month -= 12 * (nYears);
    dateTimeNew.year += (nYears);
  }
}

void checkDay(uint8_t month, uint16_t year) {
  uint8_t days_of_month;
  if ((month == 4) || (month == 6) || (month == 9) || (month == 11)) {
    days_of_month = 30;
  } else if (month == 2) {
    if ((year % 4) == 0) {
      days_of_month = 29;
    } else {
      days_of_month = 28;
    }
  } else {
    days_of_month = 31;
  }
  if (dateTimeNew.day > days_of_month) {
    dateTimeNew.day -= days_of_month;
    dateTimeNew.month += 1;
  }
}

void checkHour() {
  if (dateTimeNew.hour >= 24) {
    dateTimeNew.hour -= 24;
    dateTimeNew.day += 1;
  }
}

void checkMinute() {
  if (dateTimeNew.min >= 60) {
    dateTimeNew.min -= 60;
    dateTimeNew.hour += 1;
  }
}

uint32_t date_to_FAT32(uint16_t Year, uint8_t Month, uint8_t Day, uint8_t Hour, uint8_t Min) {
  uint32_t Date;
  // MSDOS FAT format
  if (Year < 1980) {
    Year = 1980;
  }
  Date = (Year - 1980) & 0x7f;          // 7bit
  Date = (Date << 4) + (Month & 0x0f);  // 4bit
  Date = (Date << 5) + (Day & 0x1f);    // 5bit
  Date = (Date << 5) + (Hour & 0x1f);   // 5bit
  Date = (Date << 6) + (Min & 0x3f);    // 6bit
  uint8_t Sec = 0;
  Date = (Date << 5) + ((Sec >> 1) & 0x1f);  // 5bit

  return Date;
}

void printSchedule(schedule *schedulePtr) {
  Serialprint(schedulePtr->schedule_index);
  Serialprint(F(","));
  Serialprint(schedulePtr->idSchedule);
  Serialprint(F(","));
  Serialprint(schedulePtr->idMission);
  Serialprint(F(","));
  Serialprint(schedulePtr->isEnabled);
  Serialprint(F(","));
  Serialprint(schedulePtr->year);
  Serialprint(F("/"));
  Serialprint(schedulePtr->month);
  Serialprint(F("/"));
  Serialprint(schedulePtr->day);
  Serialprint(F(","));
  Serialprint(schedulePtr->hour);
  Serialprint(F(":"));
  if (schedulePtr->min < 10) {
    Serialprint(F("0"));
  }
  Serialprint(schedulePtr->min);
  Serialprint(F(","));
  Serialprint(schedulePtr->nMission);
  Serialprint(F(","));
  Serialprint(schedulePtr->missionStartInd);
  Serialprint(F(","));
  Serialprint(schedulePtr->isRepeated);
  Serialprint(F(","));
  Serialprint(schedulePtr->repMonths);
  Serialprint(F(","));
  Serialprint(schedulePtr->repDays);
  Serialprint(F(","));
  Serialprint(schedulePtr->repHours);
  Serialprint(F(","));
  Serialprint(schedulePtr->repMins);
  Serialprint(F(","));
  Serialprint(schedulePtr->idMissionLast);
  Serialprintln();
}

void flashExplore() {
  Serialprint(F("***********************extFLASH****************************"));
  Serialprintln();
  File root = extFlash.open("/");
  Serial.println("Prima printdir");
  printDirectory(root, 0);
  Serialprint(F("***********************extFLASH****************************"));
  Serialprintln();
}

uint8_t whileAndSend(File dir, uint16_t startIndex) {
  memset(folderArray, 0, sizeof(folderArray));

  uint8_t endArray = whileFlash(dir, startIndex);

  return endArray;
}

uint8_t whileFlash(File dir, uint16_t startIndex) {
  uint8_t endArray = 0;
  while (true) {
    wdtReset();
    if (index_buff <= 199) {
      File entry = dir.openNextFile();
      if (!entry) {

        if (fileCount >= startIndex) {
          addPoint(PARENT_CMD, &folderArray);
        }
        fileCount++;
        break;
      }

      if (fileCount >= startIndex) {
        addName(&folderArray, entry.name(), strlen(entry.name()));
      }
      if (entry.isDirectory()) {
        if (fileCount >= startIndex) {
          addPoint(DIRECTORY_CMD, &folderArray);
        }
        fileCount++;
        whileFlash(entry, startIndex);
      } else {
        if (fileCount >= startIndex) {
          addPoint(FILE_CMD, &folderArray);
        }
        fileCount++;
      }
      entry.close();
    } else {
      endArray = 1;
      break;
    }
  }
  return endArray;
}

void addPoint(char fileCmd, void *folderArray) {
  // Serial.println(index_buff);
  ((char *)folderArray)[index_buff] = (char)fileCmd;
  index_buff++;
  // Serial.println("->addPoint");
}

void addName(void *folderArray, const void *name, uint8_t size) {
  uint16_t index_buff_now = index_buff;
  for (uint8_t j = 0; j < size; j++) {
    // Serial.println(index_buff);
    ((char *)folderArray)[index_buff_now + j] = ((char *)name)[j];
    index_buff++;
    // Serial.write(folderArray[index_buff + j]);
  }
  // Serial.println("fuori da FOR");
  addPoint(END_OF_STRING, &folderArray);
}
