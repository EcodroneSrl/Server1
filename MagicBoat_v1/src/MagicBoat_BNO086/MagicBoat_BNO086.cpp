#include "MagicBoat_BNO086.h"
#include "Arduino.h"

// Constructor

MagicBoat_BNO086::MagicBoat_BNO086() {
}
//********* SPI *************

void MagicBoat_BNO086::wakePinHigh() {
  pinMode(imuWAKPin, OUTPUT);
  digitalWrite(imuWAKPin, HIGH);
}

void MagicBoat_BNO086::wakePinLow() {
  pinMode(imuWAKPin, OUTPUT);
  digitalWrite(imuWAKPin, LOW);
}

uint8_t MagicBoat_BNO086::analBuffImu(uint8_t shtpDataRead[], uint8_t remainingBytes) {
  uint8_t analReturn = 0;

  int16_t dataLength = remainingBytes - 4; // Remove the header bytes from the data count

  switch (shtpDataRead[INDEX_CH]) {
  case CHANNEL_COMMAND_ID:

    break;

  case CHANNEL_EXECUTABLE_ID:

    break;

  case CHANNEL_CONTROL_ID:

    if (shtpDataRead[6] == SHTP_REPORT_COMMAND_RESPONSE) {
      SerialDeb.println("SHTP_REPORT_COMMAND_RESPONSE");
    }
    break;

  case CHANNEL_REPORTS_ID:

  {
    // SerialDeb.println("CHANNEL_REPORTS_ID");
    timeStamp = ((uint32_t)shtpDataRead[4] << (8 * 3)) | ((uint32_t)shtpDataRead[3] << (8 * 2)) | ((uint32_t)shtpDataRead[2] << (8 * 1)) | ((uint32_t)shtpDataRead[1] << (8 * 0));
    uint8_t status = shtpDataRead[9 + 2] & 0x03; // Get status bits
    uint16_t data1 = (uint16_t)shtpDataRead[9 + 5] << 8 | shtpDataRead[9 + 4];
    uint16_t data2 = (uint16_t)shtpDataRead[9 + 7] << 8 | shtpDataRead[9 + 6];
    uint16_t data3 = (uint16_t)shtpDataRead[9 + 9] << 8 | shtpDataRead[9 + 8];
    uint16_t data4 = 0;
    uint16_t data5 = 0; // We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

    if (remainingBytes - 5 > 9) {
      data4 = (uint16_t)shtpDataRead[9 + 11] << 8 | shtpDataRead[9 + 10];
    }

    if (dataLength - 5 > 11) {
      data5 = (uint16_t)shtpDataRead[9 + 13] << 8 | shtpDataRead[9 + 12];
    }

    switch (shtpDataRead[INDEX_SENSOR_ID]) {
    case SENSOR_REPORTID_ACCELEROMETER:
      AccData.X = data1;
      AccData.Y = data2;
      AccData.Z = data3;
      analReturn = SENSOR_REPORTID_ACCELEROMETER;

      break;

    case SENSOR_REPORTID_LINEAR_ACCELERATION:
      LinAccData.X = data1;
      LinAccData.Y = data2;
      LinAccData.Z = data3;
      analReturn = SENSOR_REPORTID_LINEAR_ACCELERATION;

      break;

    case SENSOR_REPORTID_GYROSCOPE:

      GyrData.Accuracy = status;
      GyrData.X = data1;
      GyrData.Y = data2;
      GyrData.Z = data3;
      analReturn = SENSOR_REPORTID_GYROSCOPE;

      break;

    case SENSOR_REPORTID_MAGNETIC_FIELD:

      MagData.Accuracy = status;
      MagData.X = data1;
      MagData.Y = data2;
      MagData.Z = data3;
      analReturn = SENSOR_REPORTID_MAGNETIC_FIELD;

      break;

    case SENSOR_REPORTID_ROTATION_VECTOR:

      QuatData.Accuracy = status;
      QuatData.rawQuatI = data1;
      QuatData.rawQuatJ = data2;
      QuatData.rawQuatK = data3;
      QuatData.rawQuatReal = data4;
      QuatData.rawQuatRadianAccuracy = data5;
      update_Euler();
      analReturn = SENSOR_REPORTID_ROTATION_VECTOR;
      break;
    }

    break;
  }

  case CHANNEL_WAKE_REPORTS_ID:

    break;

  case CHANNEL_GYRO_ID:

    break;
  }
  return analReturn;
}

void MagicBoat_BNO086::update_Euler()

{
  EulerAccData.Yaw = getYaw();
  EulerAccData.Pitch = getPitch();
  EulerAccData.Roll = getRoll();
}

void MagicBoat_BNO086::update_Acc() {
  EulerAccData.Ax = qToFloat(LinAccData.X, linear_accelerometer_Q1);
  EulerAccData.Ay = qToFloat(LinAccData.Y, linear_accelerometer_Q1);
  EulerAccData.Az = qToFloat(LinAccData.Z, linear_accelerometer_Q1);
}

void MagicBoat_BNO086::update_Mag() {
  Mx = qToFloat(MagData.X, magnetometer_Q1);
  My = qToFloat(MagData.Y, magnetometer_Q1);
  Mz = qToFloat(MagData.Z, magnetometer_Q1);
}

void MagicBoat_BNO086::update_Gyr() {
  Gx = qToFloat(GyrData.Z, gyro_Q1);
  Gy = qToFloat(GyrData.Z, gyro_Q1);
  Gz = qToFloat(GyrData.Z, gyro_Q1);
}

void MagicBoat_BNO086::update_Qua() {
  q0 = getQuatReal();
  q1 = getQuatI();
  q2 = getQuatJ();
  q3 = getQuatK();
}

void MagicBoat_BNO086::update_calib_stat() {
  mag_cal = MagData.Accuracy;
  acc_cal = LinAccData.Accuracy;
  gyr_cal = GyrData.Accuracy;
  sys_cal = EulerData.Accuracy;
}

// Return the rotation vector quaternion I
float MagicBoat_BNO086::getQuatI() {
  float quat = qToFloat(QuatData.rawQuatI, rotationVector_Q1);
  return (quat);
}

// Return the rotation vector quaternion J
float MagicBoat_BNO086::getQuatJ() {
  float quat = qToFloat(QuatData.rawQuatJ, rotationVector_Q1);
  return (quat);
}

// Return the rotation vector quaternion K
float MagicBoat_BNO086::getQuatK() {
  float quat = qToFloat(QuatData.rawQuatK, rotationVector_Q1);
  return (quat);
}

// Return the rotation vector quaternion Real
float MagicBoat_BNO086::getQuatReal() {
  float quat = qToFloat(QuatData.rawQuatReal, rotationVector_Q1);
  return (quat);
}

float MagicBoat_BNO086::getRoll() {
  float dqw = getQuatReal();
  float dqx = getQuatI();
  float dqy = getQuatJ();
  float dqz = getQuatK();

  float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
  dqw = dqw / norm;
  dqx = dqx / norm;
  dqy = dqy / norm;
  dqz = dqz / norm;

  float ysqr = dqy * dqy;

  // roll (x-axis rotation)
  float t0 = +2.0 * (dqw * dqx + dqy * dqz);
  float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
  float roll_temp = atan2(t0, t1);
  float roll = roll_temp * RAD_TO_DEG;

  return (roll);
}

// Return the pitch (rotation around the y-axis) in Radians
float MagicBoat_BNO086::getPitch() {
  float dqw = getQuatReal();
  float dqx = getQuatI();
  float dqy = getQuatJ();
  float dqz = getQuatK();

  float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
  dqw = dqw / norm;
  dqx = dqx / norm;
  dqy = dqy / norm;
  dqz = dqz / norm;

  // float ysqr = dqy * dqy;

  // pitch (y-axis rotation)
  float t2 = +2.0 * (dqw * dqy - dqz * dqx);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  float pitch_temp = asin(t2);
  float pitch = pitch_temp * RAD_TO_DEG;

  return (pitch);
}

// Return the yaw / heading (rotation around the z-axis) in Radians
float MagicBoat_BNO086::getYaw() {
  float dqw = getQuatReal();
  float dqx = getQuatI();
  float dqy = getQuatJ();
  float dqz = getQuatK();

  float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
  dqw = dqw / norm;
  dqx = dqx / norm;
  dqy = dqy / norm;
  dqz = dqz / norm;

  float ysqr = dqy * dqy;

  // yaw (z-axis rotation)
  float t3 = +2.0 * (dqw * dqz + dqx * dqy);
  float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
  float yaw_temp = atan2(t3, t4);
  float yaw = yaw_temp * RAD_TO_DEG;

  return (yaw);
}

void MagicBoat_BNO086::testYPR() {

  /* 	float q0 = getQuatReal();
          float q1 = getQuatI();
          float q2 = getQuatJ();
          float q3 = getQuatK();

          float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

          q0 = q0/norm;
          q1 = q1/norm;
          q2 = q2/norm;
          q3 = q3/norm;

          float R11 = q0 * q0 + q1 * q1 -0.5f; // = 0.5f - q2 * q2 - q3 * q3;
          float R21 = q1 * q2 - q0 * q3;
          float R31 = q1 * q3 + q0 * q2;
          float R12 = q1 * q2 + q0 * q3;
          float R22 = q0 * q0 + q2 * q2 -0.5; // = 0.5f - q1 * q1 - q3 * q3;
          float R32 = q2 * q3 - q0 * q1;
          float R13 = q1 * q3 - q0 * q2;                                      // = gx
          float R23 = q2 * q3 + q0 * q1;                                      // = gy
          float R33 = q0 * q0 + q3 * q3 -0.5f; // = 0.5f -q1 * q1 -q2 * q2;   // = gz

          float RollTest = atan2(R23, R33);
          float PitchTest = -asin(2.0 * R13);
          float YawTest = atan2(R12, R11);

          SerialDeb.print("w");
          SerialDeb.print(q0,6);
          SerialDeb.print("w");
          SerialDeb.print("a");
          SerialDeb.print(q1,6);
          SerialDeb.print("a");
          SerialDeb.print("b");
          SerialDeb.print(q2,6);
          SerialDeb.print("b");
          SerialDeb.print("c");
          SerialDeb.print(q3,6);
          SerialDeb.print("c");
          SerialDeb.println(); */
}

uint8_t MagicBoat_BNO086::readSPI(void *ptr1, uint16_t lenght, uint8_t chipSelectPin, SPISettings settSPI, uint8_t mode) {
  if ((mode == OPEN_CLOSE) || (mode == ONLY_OPEN) || (mode == NO_SAVE_OPEN_CLOSE)) {
    SPI.beginTransaction(settSPI);
    digitalWrite(chipSelectPin, LOW);
    Oldk_Init = 0;
  } else {
    Oldk_Init = Oldk + 1;
  }
  if (mode == NEW_INDEX_ONLY_CLOSE) {
    Oldk_Init = 0;
  }

  uint8_t thisValue = 0;
  if (!lenght) {
    lenght = 1;
  }
  for (uint8_t k = 0; k < lenght; k++) {
    thisValue = SPI.transfer(0x00);
    if ((mode != NO_SAVE_ONLY_CLOSE) || (mode != NO_SAVE_OPEN_CLOSE)) {
      ((unsigned char *)ptr1)[k + Oldk_Init] = thisValue;
    }
    Oldk = k;
  }
  if (DEBUG_SPI) {
    if (syncCom != SW_READ_HEADER) {
      SerialDeb.print("R:");
      for (uint8_t k = 0; k < lenght; k++) {
        // SerialDeb.print(k);
        // SerialDeb.print(":");
        SerialDeb.print(((unsigned char *)ptr1)[k + Oldk_Init], HEX);
        SerialDeb.print(",");
      }
      SerialDeb.println();
    } else {
      SerialDeb.print("RH:");
      for (uint8_t k = 0; k < lenght; k++) {
        // SerialDeb.print(k);
        // SerialDeb.print(":");
        SerialDeb.print(((unsigned char *)ptr1)[k + Oldk_Init], HEX);
        SerialDeb.print(",");
      }
      SerialDeb.println();
    }
  }
  if ((mode == OPEN_CLOSE) || (mode == ONLY_CLOSE) || (mode == NEW_INDEX_ONLY_CLOSE) || (mode == NO_SAVE_OPEN_CLOSE) || (mode == NO_SAVE_ONLY_CLOSE)) {
    digitalWrite(chipSelectPin, HIGH);
    SPI.endTransaction();
  }
  return thisValue;
}

uint8_t MagicBoat_BNO086::writeSPI(void *ptr1, uint8_t lenght, uint8_t chipSelectPin, SPISettings settSPI) {
  SPI.beginTransaction(settSPI);
  digitalWrite(chipSelectPin, LOW);
  uint8_t thisValue = 0;
  for (uint8_t k = 0; k < lenght; k++) {
    thisValue = ((unsigned char *)ptr1)[k];
    SPI.transfer(thisValue);
  }
  if (DEBUG_SPI) {
    SerialDeb.print("W:");
    for (uint8_t k = 0; k < lenght; k++) {
      // SerialDeb.print(k);
      // SerialDeb.print(":");
      thisValue = ((unsigned char *)ptr1)[k];
      SerialDeb.print(thisValue, HEX);
      SerialDeb.print(",");
    }
    SerialDeb.println();
  }
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
  return thisValue;
}
//********* END SPI *************

//********** IMU ****************

void MagicBoat_BNO086::begin() {
  initImu(SPI_SPEED, MSBFIRST, SPI_MODE3);
}

void MagicBoat_BNO086::initImu(uint32_t clockIn, uint8_t bitOrderIn, uint8_t dataModeIn) {
  settingsIMU = SPISettings(clockIn, bitOrderIn, dataModeIn);
  wakePinHigh();
  SPI.begin();
  // initPinSPI();
  SPI.setMOSI(pinMOSI);
  SPI.setMISO(pinMISO);
  SPI.setSCK(pinSCK);
  
  pinMode(imuCSPin, OUTPUT);
  digitalWrite(imuCSPin, HIGH);
  pinMode(imuINTPin, INPUT);
  pinMode(imuRSTPin, OUTPUT);
  digitalWrite(imuRSTPin, HIGH);
  hardReset();
  wakePinHigh();
}

void MagicBoat_BNO086::setPinsLow() {
  SPI.endTransaction();
  SPI.end();

  digitalWrite(pinMOSI, LOW);
  digitalWrite(pinSCK, LOW);
  digitalWrite(imuCSPin, LOW);
  digitalWrite(imuRSTPin, LOW);
  digitalWrite(imuWAKPin, LOW);
}

void MagicBoat_BNO086::enableDebMes() {
  if (DEBUG_SPI) {
    DEBUG_SPI = 0;
  } else {
    DEBUG_SPI = 1;
  }
}

void MagicBoat_BNO086::hardReset() {
  resetHard_Bno086(imuRSTPin);
  for (uint8_t k = 0; k < numChannel; k++) {
    sequenceNumber[k] = 0;
  }
}

void MagicBoat_BNO086::resetHard_Bno086(uint8_t pinReset) {
  digitalWrite(pinReset, LOW);
  delay(10);
  digitalWrite(pinReset, HIGH);
}

uint8_t MagicBoat_BNO086::setReports() {
  if (update_par.update_enable[UPD_ACC]) {
    // SerialDeb.print("enableLinearAccelerometer");
    enableLinearAccelerometer(update_par.update_timer[UPD_ACC]);
  }

  if (update_par.update_enable[UPD_GYR]) {
    // SerialDeb.println("enableGyroscope");
    enableGyroscope(update_par.update_timer[UPD_GYR]);
  }

  if (update_par.update_enable[UPD_MAG]) {
    // SerialDeb.println("enableMagnetometer");
    enableMagnetometer(update_par.update_timer[UPD_MAG]);
  }

  if (update_par.update_enable[UPD_EULER]) {
    // SerialDeb.println("enableRotationVector");
    enableRotationVector(update_par.update_timer[UPD_EULER]);
  }

  return 1;
}

uint8_t MagicBoat_BNO086::update() {
  uint8_t newRead = 0;
  if (interruptImu()) {
    timeToSend = millis();
    if ((micros() - time_head_i2c) > timer_head_i2c) {
      time_head_i2c = micros();
      if (receivePackets()) {
        analBuffImu(shtpDataRead, remainingBytes);
        newRead = 1;
      }
      /*
      //readSPIimu(shtpDataRead,HEADER_LENGTH,OPEN_CLOSE);
      readSPIimu(shtpDataRead,HEADER_LENGTH,ONLY_OPEN);
      remainingBytes = getRemainingBytes(shtpDataRead[1],shtpDataRead[0]);
      byteRemaning = maxByteRead(remainingBytes);
      SerialDeb.println(byteRemaning);
      readSPIimu(shtpDataRead,byteRemaning,ONLY_CLOSE);
      readSPIimu(shtpDataRead,byteRemaning,OPEN_CLOSE);
      */
    }
  }
  return newRead;
}

uint8_t MagicBoat_BNO086::receivePackets() {
  readImu(shtpDataRead, requestBytes, OPEN_CLOSE);
  uint8_t packets_type = read_Bno086_Data();
  return packets_type;
}

uint8_t MagicBoat_BNO086::read_Bno086_Data() {
  uint8_t readEnd = 0;
  switch (syncCom) {
  case SW_READ_HEADER:
    remainingBytes = getRemainingBytes(shtpDataRead[INDEX_MSB], shtpDataRead[INDEX_LSB]);
    if (remainingBytes < (MAX_BUFF_LENGTH - HEADER_LENGTH)) {
      requestBytes = remainingBytes;
      syncCom = SW_READ_END;
    } else {
      requestBytes = MAX_BUFF_LENGTH;
      syncCom = SW_READ_MAX;
    }
    break;

  case SW_READ_MAX:
    remainingBytes -= (MAX_BUFF_LENGTH - HEADER_LENGTH);
    if (remainingBytes < (MAX_BUFF_LENGTH - HEADER_LENGTH)) {
      requestBytes = remainingBytes;
      syncCom = SW_READ_END;
    }
    break;

  case SW_READ_END:
    requestBytes = HEADER_LENGTH;
    if (flagStartup == 0) {
      flagStartup = 1;
    }
    syncCom = SW_READ_HEADER;
    readEnd = 1;
    break;
  }
  return readEnd;
}

uint8_t MagicBoat_BNO086::readImu(void *ptr1, uint16_t lenght, uint8_t mode) {
  return readSPI(ptr1, lenght, imuCSPin, settingsIMU, mode);
}

uint8_t MagicBoat_BNO086::maxByteRead(uint8_t num) {
  uint8_t byteToRead = 0;
  if (num < (MAX_BUFF_LENGTH - HEADER_LENGTH)) {
    byteToRead = num;
  } else {
    byteToRead = MAX_BUFF_LENGTH - HEADER_LENGTH;
  }
  return byteToRead;
}

uint8_t MagicBoat_BNO086::writeImu(void *ptr1, uint8_t lenght) {
  return writeSPI(ptr1, lenght, imuCSPin, settingsIMU);
}
uint8_t MagicBoat_BNO086::interruptImu() {
  if (digitalRead(imuINTPin) == LOW) {
    return true;
  } else {
    return false;
  }
}
uint8_t MagicBoat_BNO086::packetContinue(uint8_t MSB) {
  uint8_t temp_bit = (MSB >> 7);
  return temp_bit;
}
uint16_t MagicBoat_BNO086::getRemainingBytes(uint8_t MSB, uint8_t LSB) {
  uint16_t dataLengtha = (((uint16_t)MSB) << 8) | ((uint16_t)LSB);
  dataLengtha &= ~((uint16_t)1 << 15); // Clear the MSbit.
  return dataLengtha;
}
//*************************************************************
uint8_t MagicBoat_BNO086::sendPacketImu(uint8_t channelNumber, uint8_t dataLength) {
  uint8_t packetLength = dataLength + 4; // Add four bytes for the header

  shtpDataSend[0] = (packetLength & 0xFF);
  shtpDataSend[1] = (packetLength >> 8);
  shtpDataSend[2] = (channelNumber);
  // shtpDataSend[3] =(sequenceNumber[channelNumber]); //Send the sequence number, increments with each packet sent, different counter for each channel
  shtpDataSend[3] = (sequenceNumber[channelNumber]++);
  for (uint8_t i = 0; i < (dataLength); i++) {
    shtpDataSend[i + 4] = shtpDataWrite[i];
  }
  // sequenceNumber[channelNumber]++;

  wakePinLow();
  delay(1);
  wakePinHigh();

  writeImu(shtpDataSend, packetLength);
  uint8_t writeOk = 0;
  return writeOk;
}

// Tell the sensor to do a command
// See 6.3.8 page 41, Command request
// The caller is expected to set P0 through P8 prior to calling
void MagicBoat_BNO086::sendCommandImu(uint8_t command) {
  shtpDataWrite[0] = SHTP_REPORT_COMMAND_REQUEST; // Command Request
  shtpDataWrite[1] = commandSequenceNumber++;     // Increments automatically each function call
  shtpDataWrite[2] = command;                     // Command

  // Caller must set these
  /*shtpDataWrite[3] = 0; //P0
  shtpDataWrite[4] = 0; //P1
  shtpDataWrite[5] = 0; //P2
  shtpDataWrite[6] = 0;
  shtpDataWrite[7] = 0;
  shtpDataWrite[8] = 0;
  shtpDataWrite[9] = 0;
  shtpDataWrite[10] = 0;
  shtpDataWrite[11] = 0;*/

  // Transmit packet on channel 2, 12 bytes
  sendPacketImu(CHANNEL_CONTROL, 12);
}

void MagicBoat_BNO086::requestCalibSettings() {
  /*shtpDataWrite[3] = 0; //P0 - Reserved
  shtpDataWrite[4] = 0; //P1 - Reserved
  shtpDataWrite[5] = 0; //P2 - Reserved
  shtpDataWrite[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
  shtpDataWrite[7] = 0; //P4 - Reserved
  shtpDataWrite[8] = 0; //P5 - Reserved
  shtpDataWrite[9] = 0; //P6 - Reserved
  shtpDataWrite[10] = 0; //P7 - Reserved
  shtpDataWrite[11] = 0; //P8 - Reserved*/

  for (uint8_t x = 3; x < 12; x++) { // Clear this section of the shtpData array
    shtpDataWrite[x] = 0;
  }

  shtpDataWrite[6] = SUBCOMMAND_ME_CALIBRATE_GET; // P3 - 0x01 - Subcommand: Get ME Calibration

  // Using this shtpData packet, send a command
  sendCommandImu(COMMAND_ME_CALIBRATE);
}

void MagicBoat_BNO086::configCalibSettings(bool accCal, bool gyrCal, bool magCal, bool planAccCal, bool onTabCal) {
  shtpDataWrite[3] = accCal;                      // P0 - Acc Cal En/Dis
  shtpDataWrite[4] = gyrCal;                      // P1 - Gyr Cal En/Dis
  shtpDataWrite[5] = magCal;                      // P2 - Mag Cal En/Dis
  shtpDataWrite[6] = SUBCOMMAND_ME_CALIBRATE_SET; // P3 - 0x00 - Subcommand: Configure ME Calibration
  shtpDataWrite[7] = planAccCal;                  // P4 - Planar Acc Cal En/Dis
  shtpDataWrite[8] = onTabCal;                    // P5 - On Table Cal En/Dis
  shtpDataWrite[9] = 0;                           // P6 - Reserved
  shtpDataWrite[10] = 0;                          // P7 - Reserved
  shtpDataWrite[11] = 0;                          // P8 - Reserved

  // Using this shtpData packet, send a command
  sendCommandImu(COMMAND_ME_CALIBRATE);
}

void MagicBoat_BNO086::periodicCalibSave(bool en_dis) {
  shtpDataWrite[3] = en_dis; // P0 - En/Dis Periodic Calib Save
  shtpDataWrite[4] = 0;      // P1 - Reserved
  shtpDataWrite[5] = 0;      // P2 - Reserved
  shtpDataWrite[6] = 0;      // P3 - Reserved
  shtpDataWrite[7] = 0;      // P4 - Reserved
  shtpDataWrite[8] = 0;      // P5 - Reserved
  shtpDataWrite[9] = 0;      // P6 - Reserved
  shtpDataWrite[10] = 0;     // P7 - Reserved
  shtpDataWrite[11] = 0;     // P8 - Reserved

  // Using this shtpData packet, send a command
  sendCommandImu(COMMAND_DCD_PERIOD_SAVE); // Save DCD command
}

void MagicBoat_BNO086::setFeatureCommandImu(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig) {
  long microsBetweenReports = (long)timeBetweenReports * 1000L;

  shtpDataWrite[0] = SHTP_REPORT_SET_FEATURE_COMMAND;     // Set feature command. Reference page 55
  shtpDataWrite[1] = reportID;                            // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
  shtpDataWrite[2] = 0;                                   // Feature flags
  shtpDataWrite[3] = 0;                                   // Change sensitivity (LSB)
  shtpDataWrite[4] = 0;                                   // Change sensitivity (MSB)
  shtpDataWrite[5] = (microsBetweenReports >> 0) & 0xFF;  // Report interval (LSB) in microseconds. 0x7A120 = 500ms
  shtpDataWrite[6] = (microsBetweenReports >> 8) & 0xFF;  // Report interval
  shtpDataWrite[7] = (microsBetweenReports >> 16) & 0xFF; // Report interval
  shtpDataWrite[8] = (microsBetweenReports >> 24) & 0xFF; // Report interval (MSB)
  shtpDataWrite[9] = 0;                                   // Batch Interval (LSB)
  shtpDataWrite[10] = 0;                                  // Batch Interval
  shtpDataWrite[11] = 0;                                  // Batch Interval
  shtpDataWrite[12] = 0;                                  // Batch Interval (MSB)
  shtpDataWrite[13] = (specificConfig >> 0) & 0xFF;       // Sensor-specific config (LSB)
  shtpDataWrite[14] = (specificConfig >> 8) & 0xFF;       // Sensor-specific config
  shtpDataWrite[15] = (specificConfig >> 16) & 0xFF;      // Sensor-specific config
  shtpDataWrite[16] = (specificConfig >> 24) & 0xFF;      // Sensor-specific config (MSB)

  // Transmit packet on channel 2, 17 bytes
  sendPacketImu(CHANNEL_CONTROL, 17);
}

void MagicBoat_BNO086::softReset() {
  uint8_t packetLength = 5;

  // Send the 4 byte packet header
  shtpDataWrite[INDEX_LSB] = (packetLength & 0xFF);                   // Packet length LSB
  shtpDataWrite[INDEX_MSB] = (0);                                     // Packet length MSB
  shtpDataWrite[INDEX_CH] = (CHANNEL_EXECUTABLE);                     // Channel number
  shtpDataWrite[INDEX_NMSG] = (sequenceNumber[CHANNEL_EXECUTABLE]++); // Send the sequence number, increments with each packet sent, different counter for each channel
  shtpDataWrite[INDEX_MESS] = (1);

  // sequenceNumber[CHANNEL_EXECUTABLE]++;
  writeImu(shtpDataWrite, packetLength);
}

void MagicBoat_BNO086::productIdReq() {
  shtpDataWrite[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // Set feature command. Reference page 55
  shtpDataWrite[1] = 0;
  sendPacketImu(CHANNEL_CONTROL, 2);
}

void MagicBoat_BNO086::enableAccelerometer(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableGyroscope(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableMagnetometer(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableLinearAccelerometer(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableRotationVector(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableGravity(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_GRAVITY, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableGameRotationVector(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableGeoRotationVector(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableGyroRotationVector(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableRawAccelerometer(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableRawGyroscope(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableRawMagnetometer(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableStabilizedRV(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_AR_VR_STABILIZED_RV, timeBetweenReports, 0);
}

void MagicBoat_BNO086::enableStabilizedGRV(float timeBetweenReports) {
  setFeatureCommandImu(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_RV, timeBetweenReports, 0);
}

float MagicBoat_BNO086::qToFloat(int16_t fixedPointValue, uint8_t qPoint) {
  float qFloat = fixedPointValue;
  qFloat *= pow(2, qPoint * -1);
  return (qFloat);
}

void MagicBoat_BNO086::combinaVar(void *ptr1, uint16_t lenVar, uint8_t bufferAnal[]) {
  uint8_t initVar = HEADER_LENGTH;
  for (uint16_t i = 0; i < lenVar; i++) {
    ((unsigned char *)ptr1)[i] = bufferAnal[initVar + i];
  }
}

void MagicBoat_BNO086::testImu() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '1') {
      wakePinLow();
      delay(1);
      wakePinHigh();
      SerialDeb.println("productIdReq");
      productIdReq();
    }
    // if (((millis()-timeToSendConf) > timerToSendConf) && (flagStartup == 2) && (!interruptOn()))
    if (cmd == '2') {
      // enableLinearAccelerometer(20);
      // enableAccelerometer(20);
      // enableGyroscope(20);
      // enableMagnetometer(20);
      enableRotationVector(20);
    }
    if (cmd == '3') {
      wakePinLow();
      delay(1);
      wakePinHigh();
      SerialDeb.println("softReset");
      softReset();
    }

    if (cmd == '4') {
      SerialDeb.println("hardReset");
      hardReset();
    }
  }
}
