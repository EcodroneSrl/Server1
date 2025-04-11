#define NAV_TEL 0
#define NAV_AUT 1

#define TEL_MODE_1 0 // erpm_base & +DD,+CD,-SS,-CS
#define TEL_MODE_2 1 // erpm_base & ThetaD
#define TEL_MODE_3 2 // velocità mantenuta da GPS
#define TEL_MODE_4 3 // velocità mantenuta da GPS & ThetaD

#define AUT_MODE_1 0 // erpm base + ThetaD da waypoint
#define AUT_MODE_2 1 // velocità mantenuta da GPS & ThetaD da waypoint
#define AUT_MODE_3 2 // erpm comp.energia + ThetaD da waypoint
#define AUT_MODE_4 3 // punto-raggio
#define AUT_MODE_5 4 // interno ad area con waypoint ricevuti da jetson

#define RIF_INTERMEDI 0
#define RIF_MISSION 1

#define GPS_UBX 0
#define GPS_AIS 1
#define GPS_RUT 2

#define THETAB_IMU 0
#define THETAB_DIFF 1
#define THETAB_VEL 2
#define THETAB_IMU_DIFF 3
#define THETAB_IMU_VEL 4

#define CONTROL_MODE_PID 0
#define CONTROL_MODE_ADAPTIVE 1
#define CONTROL_MODE_ROBUST 2

struct __attribute__((packed)) motorControl {
  uint16_t max_erpm = 8000;
  uint16_t max_min_diff = 2000;
  float kDiff = 2;
};

struct __attribute__((packed)) miscParam {
  uint16_t base = 5000;
  uint16_t boost = 1000;
  uint16_t wheel = 500;
  motorControl motorControlData;
} miscParamData;

struct __attribute__((packed)) controlCmd {
  float base_cmd;
  float boost_cmd;
  float diff_cmd;
  float wheel_cmd;
  uint8_t mode;
} controlCmdData;

struct __attribute__((packed)) pidT {
  float kp = 5;
  float ki = 0;
  float kd = 0;
  float AW = 100;
  int IL = 0;
} pidTheta;

struct __attribute__((packed)) pidV {
  float kp = 5;
  float ki = 0;
  float kd = 0;
  float AW = 100;
  int IL = 0;
} pidVel;

uint32_t time_tetaD;
uint16_t timer_tetaD = 500;

// PID ThetaD
float TetaD = 0;
float TetaB = 0;

float corrRoute = 0;
float corrTetaDLim = 90.0;
float kpTetaD = 5.0;

// PID VelD
float VelD = 0;
float Vel_GPS = 0;
float VelDMax = 10;

float corrVel = 0;

// Paarmetri tabulazione NavMode
float Kerpm = 1;
float Kvel = 30;
float Kcomp = 1;

// Compensazione di energia
float PowerBoat = 0;

uint8_t gpsSource = GPS_UBX;
uint8_t thetaBSource = THETAB_IMU;

uint8_t controlModeRoute = CONTROL_MODE_PID;

uint8_t controlModeVel = CONTROL_MODE_PID;

uint8_t boatNavMode = NAV_TEL;
uint8_t boatNavSubMode = TEL_MODE_1;

// Variabili per i setting di navigazione autonoma
uint8_t telInputDev = INPUT_JOYSTICK_CMD3;

//****************** Jetson **********************
// Correzione TetaD da anticollisione
uint32_t time_tetaD_AC = 0;
uint16_t timer_tetaD_AC = 3000;
float corrTetaD_AC = 0;

int16_t signalStrength = 1;
uint32_t time_signal_jetson = 0;
uint16_t timer_signal_jetson = 3000;

// Variabili per i pulsanti del joystick
float corrTetaDBut = 0;
uint8_t enNewPOV = 1;

// Debug

uint32_t time_print_js;

struct __attribute__((packed)) js_debug {
  uint8_t debug_en = 0;
  uint8_t descr_en = 1;
  uint8_t head_en = 1;
  uint16_t timer_print_js = 100;
  uint8_t index;
} js_debug_par;