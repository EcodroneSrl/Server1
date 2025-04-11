#ifndef MagicBoat_BNO086_h
#define MagicBoat_BNO086_h

#include "SPI.h"

#ifdef __GNUC__
#include <Math.h>
#else
#include <math.h>
#endif

#define SerialDeb Serial

#define UPDATE_ON 1
#define DESCR_ON 1
#define HEAD_ON 1

#define UPDATE_OFF 0
#define DESCR_OFF 0
#define HEAD_OFF 0

#define numChannel 6

#define SW_READ_HEADER 0
#define SW_READ_MAX 1
#define SW_READ_END 2

#define OPEN_CLOSE 0
#define ONLY_CLOSE 1
#define ONLY_OPEN 2
#define NEW_INDEX_ONLY_CLOSE 3
#define NO_SAVE_OPEN_CLOSE 4
#define NO_SAVE_ONLY_CLOSE 5

#define HEADER_LENGTH 4

#define INDEX_LSB 0
#define INDEX_MSB 1
#define INDEX_CH 2
#define INDEX_NMSG 3
#define INDEX_MESS 4
#define INDEX_SENSOR_ID 9

#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_RV 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_RV 0x29

#define CHANNEL_COMMAND_ID 0
#define CHANNEL_EXECUTABLE_ID 1
#define CHANNEL_CONTROL_ID 2
#define CHANNEL_REPORTS_ID 3
#define CHANNEL_WAKE_REPORTS_ID 4
#define CHANNEL_GYRO_ID 5

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7

#define SUBCOMMAND_ME_CALIBRATE_SET 0
#define SUBCOMMAND_ME_CALIBRATE_GET 1

#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

/*------- Case per update -------*/

#define UPD_CALIB_STAT 0
#define UPD_CALIB_OFFSET 1
#define UPD_ACC_CONF 2
#define UPD_GYR_CONF 3
#define UPD_MAG_CONF 4
#define UPD_EULER 5
#define UPD_ACC 6
#define UPD_GYR 7
#define UPD_MAG 8
#define UPD_QUA 9

/*------- Case per Debug -------*/

#define DEB_CALIB_STAT 0
#define DEB_CALIB_OFFSET 1
#define DEB_ACC_CONF 2
#define DEB_GYR_CONF 3
#define DEB_MAG_CONF 4
#define DEB_EULER 5
#define DEB_ACC 6
#define DEB_GYR 7
#define DEB_MAG 8
#define DEB_QUA 9
#define DEB_EUL_QUA 10
#define DEB_X 11
#define DEB_Y 12
#define DEB_Z 13

class MagicBoat_BNO086 {
public:

  // Internal variables

  bool DEBUG_SPI = 0;

  uint8_t deviceAddress_BNO086 = 0x4A;

  uint8_t MAX_BUFF_LENGTH = 32;
  uint8_t requestBytes = HEADER_LENGTH;

  uint16_t remainingBytes = 0;
  uint16_t byteRemaning = 0;

  uint8_t shtpDataRead[32];
  uint8_t shtpDataWrite[32];
  uint8_t shtpDataSend[32];
  uint8_t sequenceNumber[numChannel] = { 0, 0, 0, 0, 0, 0 };  //There are 6 com channels. Each channel has its own seqnum
  uint8_t commandSequenceNumber = 0;                          //Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
  uint8_t syncCom = 0;
  uint8_t flagStartup = 0;

  const byte CHANNEL_COMMAND = 0;
  const byte CHANNEL_EXECUTABLE = 1;
  const byte CHANNEL_CONTROL = 2;
  const byte CHANNEL_REPORTS = 3;
  const byte CHANNEL_WAKE_REPORTS = 4;
  const byte CHANNEL_GYRO = 5;

  uint8_t Oldk = 0;
  uint8_t Oldk_Init = 0;

  SPISettings settingsIMU;
  uint8_t imuINTPin = 32;
  uint8_t imuRSTPin = 37;
  uint8_t imuWAKPin = 33;
  uint8_t imuCSPin = 10;
  uint8_t pinMOSI = 11;
  uint8_t pinMISO = 12;
  uint8_t pinSCK = 13;

  uint32_t SPI_SPEED = 1000000;

  uint32_t timer_head_i2c = 70;
  uint32_t time_head_i2c = 0;

  uint32_t timerToSend = 500;
  uint32_t timeToSend = 0;

  struct __attribute__((packed)) cal {
    int16_t acc_off_x = 40;
    int16_t acc_off_y = -19;
    int16_t acc_off_z = -14;
    int16_t mag_off_x = -38;
    int16_t mag_off_y = -139;
    int16_t mag_off_z = 79;
    int16_t gyr_off_x = -1;
    int16_t gyr_off_y = -3;
    int16_t gyr_off_z = -1;
    int16_t acc_rad = 1000;
    int16_t mag_rad = 518;
  } cal_values;

  struct __attribute__((packed)) update {
    uint8_t update_enable[10] = { 0, 0, 0, 0, 0, 1, 1, 1, 0, 0 };
    uint16_t update_timer[10] = { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
  } update_par;

  uint32_t update_time[10] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

  struct __attribute__((packed)) debug {
    uint8_t debug_enable[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t debug_en_descr[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t debug_en_update[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint16_t debug_timer[14] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };
    uint8_t debug_en_head[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  } debug_par;

  uint32_t debug_time[14] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

  //These Q values are defined in the datasheet but can also be obtained by querying the meta data records
  //See the read metadata example for more info
  int16_t rotationVector_Q1 = 14;
  int16_t rotationVectorAccuracy_Q1 = 12;  //Heading accuracy estimate in radians. The Q point is 12.
  int16_t accelerometer_Q1 = 8;
  int16_t linear_accelerometer_Q1 = 8;
  int16_t gyro_Q1 = 9;
  int16_t magnetometer_Q1 = 4;
  int16_t angular_velocity_Q1 = 10;

  uint32_t timeStamp;

  struct __attribute__((packed)) EulerAcc {
    float Yaw = 0;
    float Pitch = 0;
    float Roll = 0;
    float Ax = 0;
    float Ay = 0;
    float Az = 0;
  } EulerAccData;

  struct __attribute__((packed)) Acc {
    uint8_t Accuracy;
    int16_t X;
    int16_t Y;
    int16_t Z;
  } AccData;

  struct __attribute__((packed)) LinAcc {
    uint8_t Accuracy;
    int16_t X;
    int16_t Y;
    int16_t Z;
  } LinAccData;

  struct __attribute__((packed)) Mag {
    uint8_t Accuracy;
    int16_t X;
    int16_t Y;
    int16_t Z;
  } MagData;

  struct __attribute__((packed)) Gyr {
    uint8_t Accuracy;
    int16_t X;
    int16_t Y;
    int16_t Z;
  } GyrData;

  struct __attribute__((packed)) Euler {
    uint8_t Accuracy;
    int16_t Yaw;
    int16_t Pitch;
    int16_t Roll;
  } EulerData;

  struct __attribute__((packed)) Quat {
    uint8_t Accuracy;
    int16_t rawQuatI;
    int16_t rawQuatJ;
    int16_t rawQuatK;
    int16_t rawQuatReal;
    int16_t rawQuatRadianAccuracy;
  } QuatData;

  struct __attribute__((packed)) CalibSettings {
    uint8_t acc;
    uint8_t gyr;
    uint8_t mag;
    uint8_t planar_acc;
    uint8_t on_table;
  } CalibSettingsData;

  float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz;
  float q0, q1, q2, q3;
  uint8_t mag_cal, acc_cal, gyr_cal, sys_cal;

  // Methods

  //***** SPI *****
  MagicBoat_BNO086();
  void wakePinHigh();
  void wakePinLow();
  uint8_t analBuffImu(uint8_t shtpDataRead[], uint8_t remainingBytes);
  uint8_t readSPI(void *ptr1, uint16_t lenght, uint8_t chipSelectPin, SPISettings settSPI, uint8_t mode);
  uint8_t writeSPI(void *ptr1, uint8_t lenght, uint8_t chipSelectPin, SPISettings settSPI);

  //***** IMU & SHTP *****
  void testImu();
  void begin();
  void enableDebMes();
  void initImu(uint32_t clockIn, uint8_t bitOrderIn, uint8_t dataModeIn);
  void setPinsLow();
  void softReset();
  void hardReset();
  void resetHard_Bno086(uint8_t pinReset);
  uint8_t update();
  uint8_t receivePackets();
  uint8_t read_Bno086_Data();
  uint8_t readImu(void *ptr1, uint16_t lenght, uint8_t mode);
  uint8_t maxByteRead(uint8_t num);
  uint8_t writeImu(void *ptr1, uint8_t lenght);
  uint8_t interruptImu();
  uint8_t packetContinue(uint8_t MSB);
  uint16_t getRemainingBytes(uint8_t MSB, uint8_t LSB);
  void setFeatureCommandImu(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
  void sendCommandImu(uint8_t command);
  uint8_t sendPacketImu(uint8_t channelNumber, uint8_t dataLength);

  void requestCalibSettings();
  void configCalibSettings(bool accCal, bool gyrCal, bool magCal, bool planAccCal, bool onTabCal);
  void periodicCalibSave(bool en_dis);

  void productIdReq();

  uint8_t setReports();

  void enableAccelerometer(float timeBetweenReports);
  void enableGyroscope(float timeBetweenReports);
  void enableMagnetometer(float timeBetweenReports);
  void enableLinearAccelerometer(float timeBetweenReports);
  void enableRotationVector(float timeBetweenReports);
  void enableGravity(float timeBetweenReports);
  void enableGameRotationVector(float timeBetweenReports);
  void enableGeoRotationVector(float timeBetweenReports);
  void enableGyroRotationVector(float timeBetweenReports);
  void enableRawAccelerometer(float timeBetweenReports);
  void enableRawGyroscope(float timeBetweenReports);
  void enableRawMagnetometer(float timeBetweenReports);
  void enableStabilizedRV(float timeBetweenReports);
  void enableStabilizedGRV(float timeBetweenReports);

  float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
  void combinaVar(void *ptr1, uint16_t lenVar, uint8_t bufferAnal[]);

  float getQuatI();
  float getQuatJ();
  float getQuatK();
  float getQuatReal();
  float getRoll();
  float getPitch();
  float getYaw();

  void update_Euler();
  void update_Acc();
  void update_Mag();
  void update_Gyr();
  void update_Qua();
  void update_calib_stat();

  void testYPR();


  // Properties
};

#endif
