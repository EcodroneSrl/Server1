#define NUM_MEDIA_IMU 20
#define NUM_MEDIA_MOTOR 20

#define IMU_YAW_INDEX 0
#define IMU_PITCH_INDEX 1
#define IMU_ROLL_INDEX 2
#define IMU_AX_INDEX 3
#define IMU_AY_INDEX 4
#define IMU_AZ_INDEX 5
#define IMU_GX_INDEX 6
#define IMU_GY_INDEX 7
#define IMU_GZ_INDEX 8

#define MOTOR_DD_TEMP_INDEX 0
#define MOTOR_CD_TEMP_INDEX 1
#define MOTOR_SS_TEMP_INDEX 2
#define MOTOR_CS_TEMP_INDEX 3
#define MOTOR_DD_CURR_INDEX 4
#define MOTOR_CD_CURR_INDEX 5
#define MOTOR_SS_CURR_INDEX 6
#define MOTOR_CS_CURR_INDEX 7
#define MOTOR_DD_RPM_INDEX 8
#define MOTOR_CD_RPM_INDEX 9
#define MOTOR_SS_RPM_INDEX 10
#define MOTOR_CS_RPM_INDEX 11

//******************PSRAM***********************

//NAV-PVT variables
typedef struct __attribute__((packed)) {
  uint32_t iTOW;  //ms
  uint16_t year;  //UTC
  uint8_t month;  //1_12
  uint8_t day;    //1_31_UTC
  uint8_t hour;   //0_23_UTC
  uint8_t min;    //0_59_UTC
  uint8_t sec;    //0_59UTC

  // Validity Flags
  uint8_t validDate;
  uint8_t validTime;
  uint8_t fullyResolved;
  uint8_t validMag;

  uint32_t tAcc;        //time accuracy
  int32_t nano;         //ns frazione di secondo;
  uint8_t fixType = 0;  //uguale a FIX

  // Fix Status Flags
  uint8_t gnssFixOK;
  uint8_t diffSoln;
  uint8_t psmState;
  uint8_t headVehValid;
  uint8_t carrSoln;

  // Additional Flags 1
  uint8_t confirmedAvai;
  uint8_t confirmedDate;
  uint8_t confirmedTime;

  uint8_t numSV;  //Uguale a Sats: numero di sat usati in NAV
  int32_t lon;
  int32_t lat;
  int32_t height;    //altitudine dall'ellissoide [mm]
  int32_t hMSL;      //altezza dal livello del mare[mm]
  uint32_t hAcc;     //accuratezza orizzontale stimata[mm]
  uint32_t vAcc;     //accuratezza verticale
  int32_t velN;      //[mm/s] velocità verso nord
  int32_t velE;      //verso est
  int32_t velD;      //verso down
  int32_t gSpeed;    // Speed 2d velocita
  int32_t headMot;   //uguale HEADING 2Dprecisione 1e-5;
  uint32_t sAcc;     //accuratezza speed
  uint32_t headAcc;  //accuratezza heading
  uint16_t pDOP;     //PositionDOP

  // Additional Flags 2
  uint8_t invalidL1h;

  int32_t headVeh;  //1e-5 deg tipo bussola orientazione del veicolo
  int16_t magDec;   //1e-2 magnetico declination
  uint16_t magAcc;  //1e-2 accuratezza declinazione magnetica

  uint32_t millisNow;
  uint32_t microsNow;
} pvtPSRAM;

//NAV-RELPOSNED variables
typedef struct __attribute__((packed)) {
  uint8_t version;
  uint8_t refStationId;
  uint32_t iTOW;  //ms

  int32_t relPosN;        //North component of relative position vector ~ cm
  int32_t relPosE;        //East component of relative position vector ~ cm
  int32_t relPosD;        //Down component of relative position vector ~ cm
  int32_t relPosLength;   //Length of relative position vector ~ cm
  int32_t relPosHeading;  //Heading of relative position vector ~ deg 1e-5

  int8_t relPosHPN;       //High Precision North component of relative position vector ~ 0.1mm
  int8_t relPosHPE;       //High Precision East component of relative position vector ~ 0.1mm
  int8_t relPosHPD;       //High Precision Down component of relative position vector ~ 0.1mm
  int8_t relPosHPLength;  //High Precision Length of relative position vector ~ 0.1mm
  uint32_t accN;          //Accuracy of relative position North component ~ 0.1mm
  uint32_t accE;          //Accuracy of relative position East component ~ 0.1mm
  uint32_t accD;          //Accuracy of relative position Down component ~ 0.1mm
  uint32_t accLength;     //Accuracy of Length of the relative position vector ~ 0.1mm
  uint32_t accHeading;    //Accuracy of Heading of the relative position vector ~ deg 1e-5

  //Flags
  uint8_t gnssFixOK;           //Valid Fix
  uint8_t diffSoln;            //1 if differential corrections applied
  uint8_t relPosValid;         //1 if relative position components and accuracies valid
  uint8_t carrSoln;            //0=no carrier; 1=carrier with floating ambiguities; 2= carrier with fixed ambiguities
  uint8_t isMoving;            //1 if receier in moving base mode
  uint8_t refPosMiss;          //1 if extrapolated reference position used to compute moving base in this epoch
  uint8_t refObsMiss;          //1 if extrapolated reference observations used to compute moving base in this epoch
  uint8_t relPosHeadingValid;  //1 if relative position components and accuracies valid
  uint8_t relPosNormalized;    //1 if components of relative position vector are normalized

  uint32_t millisNow;
  uint32_t microsNow;
} relposnedPSRAM;

typedef struct __attribute__((packed)) {
  float Yaw_max;
  float Yaw_min;
  float Yaw_med;

  float Pitch_max;
  float Pitch_min;
  float Pitch_med;

  float Roll_max;
  float Roll_min;
  float Roll_med;

  float Ax_max;
  float Ax_min;
  float Ax_med;

  float Ay_max;
  float Ay_min;
  float Ay_med;

  float Az_max;
  float Az_min;
  float Az_med;

  float Gx_max;
  float Gx_min;
  float Gx_med;

  float Gy_max;
  float Gy_min;
  float Gy_med;

  float Gz_max;
  float Gz_min;
  float Gz_med;
} imuPSRAM;

typedef struct __attribute__((packed)) {
  float temp_motor1_max;
  float temp_motor1_min;
  float temp_motor1_med;

  float avg_motor_current_max;
  float avg_motor_current_min;
  float avg_motor_current_med;

  float rpm_max;
  float rpm_min;
  float rpm_med;

} motorPSRAM;

typedef struct __attribute__((packed)) {
  float vcell[24];
  float PackCurrent;
  float dsg_rate;
} bmsPSRAM;

typedef struct __attribute__((packed)) {
  uint8_t store_index = 0;
  uint32_t time_past = 0;
  pvtPSRAM pvtPSRAMData;
  relposnedPSRAM relposnedPSRAMData;
  imuPSRAM imuPSRAMData;
  motorPSRAM motorPSRAMDataDD;
  motorPSRAM motorPSRAMDataCD;
  motorPSRAM motorPSRAMDataSS;
  motorPSRAM motorPSRAMDataCS;
  bmsPSRAM bmsPSRAMData;
  uint32_t checksum = 0;
} PSRAM;

//*********************** PSRAM ************************