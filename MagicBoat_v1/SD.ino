uint8_t crypto_sd_on = 1;

uint32_t sd_write_time = 0;
uint16_t sd_write_timer = 1000;

uint32_t sd_read_time = 0;
uint16_t sd_read_timer = 1000;

char sdFolder[11];
char sdFilename[20];

PSRAM PSRAMDataIN[10];
EXTMEM uint8_t sdArray[544 * psram_ind_max];
EXTMEM uint16_t numChunks;

void sdInit() {
  if (!SD.begin(chipSelect)) {
    Serialprint(F("Card failed, or not present"));
    Serialprintln();
  }
  crypto.dcp_init();
}

void writeStruct() {
  createSdFolder();
  createSdFilename();

  uint32_t t1 = millis();

  File structFile = SD.open(sdFilename, FILE_WRITE);

  uint32_t checksumIn = 0;
  memset(sdArray, 0, sizeof(sdArray));

  for (uint8_t psram_ind = 0; psram_ind < psram_ind_max; psram_ind++) {
    uint8_t bufferIn[sizeof(PSRAMData[psram_ind])];
    memcpy(bufferIn, (byte *)&PSRAMData[psram_ind], sizeof(PSRAMData[psram_ind]));
    uint16_t numBytes = sizeof(bufferIn);
    float numChunksFloat = (numBytes) / (16.00);
    numChunks = ceil(numChunksFloat);

    if (structFile) {
      for (uint16_t i = 0; i < numChunks; i++) {
        uint8_t cipherChunkOut[16];
        uint8_t plainChunkIn[16];

        if (i < (numChunks - 1)) {
          for (uint8_t j = 0; j < 16; j++) {
            plainChunkIn[j] = bufferIn[j + i * 16];
          }
        } else if (i == (numChunks - 1)) {
          uint8_t finalByteNum = sizeof(bufferIn) - (numChunks - 1) * 16;
          for (uint8_t j = 0; j < finalByteNum; j++) {
            plainChunkIn[j] = bufferIn[j + i * 16];
          }
        }
        if (crypto_sd_on) {
          crypto.encryptAESMess(keyAes128, ive, plainChunkIn, cipherChunkOut);
          memcpy(&sdArray[16 * (psram_ind * numChunks + i)], &cipherChunkOut, sizeof(cipherChunkOut));
          for (uint16_t k = 0; k < 16; k++) {
            checksumIn += cipherChunkOut[k];
          }
        } else {
          memcpy(&sdArray[16 * (psram_ind * numChunks + i)], &plainChunkIn, sizeof(plainChunkIn));
          for (uint16_t k = 0; k < 16; k++) {
            checksumIn += plainChunkIn[k];
          }
        }
      }
    }
    PSRAMData[psram_ind].checksum = checksumIn;
    checksumIn = 0;
  }
  uint32_t t2 = millis() - t1;
  if (debug_time_sd) {
    Serialprint(F("t2:"));
    Serialprint(t2);
    Serialprintln();
  }

  structFile.write(sdArray, sizeof(sdArray));
  structFile.close();

  uint32_t t3 = millis() - t1;
  if (debug_time_sd) {
    Serialprint(F("t3:"));
    Serialprint(t3);
    Serialprintln();
  }

  // File sdFile = SD.open(sdFilename, FILE_READ);
  // uint32_t numByte = sdFile.available();
  // if (sdFile) {
  //   for (uint8_t psram_ind = 0; psram_ind < psram_ind_max; psram_ind++) {
  //     sdFile.seek(numByte - numChunks * 16 * (psram_ind_max - psram_ind));
  //     for (uint16_t j = 0; j < 16 * numChunks; j++) {
  //       uint8_t byteNow = sdFile.read();
  //       checksumOut += byteNow;
  //     }

  //     if (checksumOut == PSRAMData[psram_ind].checksum) {
  //       Serialprint(F("Checksum buono: n."));
  //       Serialprint(psram_ind);
  //       Serialprintln();
  //     }
  //     checksumOut = 0;
  //   }
  //   uint32_t t4 = millis() - t1;
  //   if (debug_time_sd) {
  //     Serialprint(F("t4:"));
  //     Serialprint(t4);
  //     Serialprintln();
  //   }
  // }
  // sdFile.close();
}

void readStruct() {
  // createSdFolder();
  // createSdFilename();

  // File structFile = SD.open(sdFilename, FILE_READ);

  // uint32_t numByte = structFile.available();
  // uint16_t struct_num = numByte/sizeof(PSRAM);
  // uint8_t plainTextOut[numByte];
  // uint8_t cipherTextIn[numByte];

  // for(uint16_t i = 0; i < numByte; i++)
  // {
  //   cipherTextIn[i] = structFile.read();
  // }

  // for(uint16_t i = 0; i < struct_num; i++)
  // {
  //   for(uint16_t j = i*sizeof(PSRAM); j < (i + 1)*sizeof(PSRAM); i++)
  //   {
  //     cipherBlock[j - sizeof(PSRAM)] = cipherTextIn[j];
  //     crypto.decryptAESMess(keyAes128,ive,cipherTextIn,plainTextOut);
  //   }
  // }

  // for(uint8_t psram_ind = 0; psram_ind < psram_ind_max; psram_ind++)
  // {

  // }
  // structFile.close();

  for (uint8_t psram_ind = 0; psram_ind < psram_ind_max; psram_ind++) {
    Serialprint(PSRAMDataIN[psram_ind].imuPSRAMData.Yaw_med);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].imuPSRAMData.Pitch_med);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].imuPSRAMData.Yaw_med);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].imuPSRAMData.Ax_med);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].imuPSRAMData.Ay_med);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].imuPSRAMData.Az_med);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].imuPSRAMData.Gx_med);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].imuPSRAMData.Gy_med);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].imuPSRAMData.Gz_med);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].pvtPSRAMData.lat);
    Serialprint(F(","));
    Serialprint(PSRAMDataIN[psram_ind].pvtPSRAMData.lon);
    Serialprintln();
  }
}

void createSdFolder() {
  String str_day = String(gps.pvt1.day);
  String str_month = String(gps.pvt1.month);
  String str_year = String(gps.pvt1.year);

  String sdFolder_str = str_day + str_month + str_year;

  sdFolder_str.toCharArray(sdFolder, sizeof(sdFolder));

  if (!SD.exists(sdFolder)) {
    SD.mkdir(sdFolder);
  }
}

void createSdFilename() {
  String str_hour = String(gps.pvt1.hour);
  String str_min = String(gps.pvt1.min);
  String str_sec = String(gps.pvt1.sec);
  char char_hour[3];
  char char_min[3];
  char char_sec[3];
  str_hour.toCharArray(char_hour, 3);
  str_min.toCharArray(char_min, 3);
  str_sec.toCharArray(char_sec, 5);
  char slash[] = "/";
  strcpy(sdFilename, slash);
  strcat(sdFilename, sdFolder);
  strcat(sdFilename, slash);
  strcat(sdFilename, char_hour);
  strcat(sdFilename, char_min);
  strcat(sdFilename, char_sec);
  strcat(sdFilename, ".bin");
}

void logBufferAnalSD(uint8_t bufferAnal[], uint8_t start_byte, uint16_t indexSD) {
  File analFile = SD.open("analRW.bin", FILE_WRITE);
  if (analFile) {
    uint8_t offset_size = 11;
    uint8_t cipherTextOut[offset_size + bufferAnal[start_byte]];
    uint8_t plainTextIn[offset_size + bufferAnal[start_byte]] = {
        highByte(indexSD),
        lowByte(indexSD),
        gps.pvt1.day,
        gps.pvt1.month,
        (uint8_t)(gps.pvt1.year - 2000),
        gps.pvt1.hour,
        gps.pvt1.min,
        gps.pvt1.sec,
        id.boat[0],
        id.boat[1],
        id.boat[2]};

    for (uint16_t i = offset_size; i < sizeof(plainTextIn); i++) {
      plainTextIn[i] = bufferAnal[i - offset_size + start_byte];
    }

    crypto.encryptAESMess(keyAes128, ive, plainTextIn, cipherTextOut);

    for (uint16_t i = 0; i < sizeof(plainTextIn); i++) {
      analFile.write(cipherTextOut[i]);
    }
    analFile.close();
  }
}

void printBufferAnalSD(char *sdFilename) {
  if ((millis() - sd_read_time) > sd_read_timer) {
    sd_read_time = millis();
    if (SD.exists(sdFilename)) {
      File analFile = SD.open(sdFilename, FILE_READ);
      uint16_t numByte = analFile.available();
      uint8_t plainTextOut[numByte];
      uint8_t cipherTextIn[numByte];
      for (uint16_t i = 0; i < numByte; i++) {
        cipherTextIn[i] = analFile.read();
      }
      crypto.decryptAESMess(keyAes128, ive, cipherTextIn, plainTextOut);
      for (uint16_t i = 0; i < sizeof(plainTextOut); i++) {
        Serialprint(plainTextOut[i]);
        Serialprint(F(","));
      }
      Serialprintln();
      analFile.close();
    }
  }
}

void sdExplore() {
  Serialprint(F("************************SD CARD****************************"));
  Serialprintln();
  File root = SD.open("/");
  Serial.println("Prima printdir");
  printDirectory(root, 0);
  Serialprint(F("************************SD CARD****************************"));
  Serialprintln();
}

uint8_t fold = 0;

void printDirectory(File dir, int numSpaces) {

  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      //  Serial.print("** no more files **");
      //  Serial.println();
      break;
    }
    printSpaces(numSpaces);

    Serial.print(entry.name());

    if (entry.isDirectory()) {
      Serial.print("/");
      Serial.println();
      printDirectory(entry, numSpaces + 2);
    } else {
      // files have sizes, directories do not
      unsigned int n = log10(entry.size());
      if (n > 10)
        n = 10;
      printSpaces(50 - numSpaces - strlen(entry.name()) - n);
      Serial.print("  ");
      Serial.print(entry.size(), DEC);
      DateTimeFields datetime;
      if (entry.getModifyTime(datetime)) {
        printSpaces(4);
        printTime(datetime);
      }
      Serial.println();
    }
    entry.close();
  }
}

void printSpaces(int num) {
  for (int i = 0; i < num; i++) {
    Serial.print(" ");
  }
}

void printTime(const DateTimeFields tm) {
  const char *months[12] = {
      "January", "February", "March", "April", "May", "June",
      "July", "August", "September", "October", "November", "December"};
  if (tm.hour < 10)
    Serial.print('0');
  Serial.print(tm.hour);
  Serial.print(':');
  if (tm.min < 10)
    Serial.print('0');
  Serial.print(tm.min);
  Serial.print("  ");
  Serial.print(tm.mon < 12 ? months[tm.mon] : "???");
  Serial.print(" ");
  Serial.print(tm.mday);
  Serial.print(", ");
  Serial.print(tm.year + 1900);
}
