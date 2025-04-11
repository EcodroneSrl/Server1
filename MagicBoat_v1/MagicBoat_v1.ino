
#include <AP_Declination.h>
#include <Bounce.h>
#include <EEPROM.h>
#include <PCF8575.h>
#include <SD.h>
#include <SPI.h>
#include <Snooze.h>
#include <Watchdog_t4.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <EBYTE.h>

// Choose the IMU
// #define BNO_055
#define BNO_086

#ifdef __GNUC__

#ifdef BNO_055
#include "src/MagicBoat_BNO055/MagicBoat_BNO055.h"
#elif defined(BNO_086)
#include "src/MagicBoat_BNO086/MagicBoat_BNO086.h"
#endif

#include "src/Crypto_T4/Crypto_T4.h"
#include "src/MB85_FRAM/MB85_FRAM.h"
#include "src/MagicBoat_BMS/MagicBoat_BMS.h"
#include "src/MagicBoat_CmdRW/MagicBoat_CmdRW.h"
#include "src/MagicBoat_GPS_F9X/MagicBoat_GPS_F9X.h"
#include "src/MagicBoat_Media/MagicBoat_Media.h"
#include "src/MagicBoat_PID/MagicBoat_PID.h"
#include "src/MagicBoat_VESC_CAN/MagicBoat_VESC_CAN.h"
#include "src/TimerOne/TimerOne.h"

#else

#ifdef BNO_055
#include "src\MagicBoat_BNO055\MagicBoat_BNO055.h"
#elif defined(BNO_086)
#include "src\MagicBoat_BNO086\MagicBoat_BNO086.h"
#endif

#include "src\Crypto_T4\Crypto_T4.h"
#include "src\MB85_FRAM\MB85_FRAM.h"
#include "src\MagicBoat_BMS\MagicBoat_BMS.h"
#include "src\MagicBoat_CmdRW\MagicBoat_CmdRW.h"
#include "src\MagicBoat_GPS_F9X\MagicBoat_GPS_F9X.h"
#include "src\MagicBoat_Media\MagicBoat_Media.h"
#include "src\MagicBoat_PID\MagicBoat_PID.h"
#include "src\MagicBoat_VESC_CAN\MagicBoat_VESC_CAN.h"
#include "src\TimerOne\TimerOne.h"

#endif

#include "cmdRW.h"
#include "control.h"
#include "dummy.h"
#include "extmem.h"
#include "wp.h"
#include <LittleFS.h>
#include <Wire.h>

uint8_t myID = ID_MODULO_BASE;
uint8_t debug_port = ID_PORTA_0;
uint8_t socketActive = 0;

struct __attribute__((packed)) loop_en {
  uint8_t debugAnalBuffer = 0;
  uint8_t debugPrintFlag = 0;
} loop_en_data;

uint16_t echoBuffer[16];

struct __attribute__((packed)) echo_cfg {
  uint8_t echo_probe_num;
  uint8_t sensor_en[16] = { 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
} echo_cfg_data;

struct __attribute__((packed)) echo_debug {
  uint8_t debug_en = 0;
  uint8_t descr_en = 1;
  uint8_t head_en = 1;
  uint16_t timer_print_echo = 200;
} echo_debug_par;

uint32_t time_print_echo;

//****************** COMUNICAZIONE ***********************

MagicBoat_CmdRW SerialRWUSB;
MagicBoat_CmdRW SerialRW6;
MagicBoat_CmdRW SerialRW7;
MagicBoat_CmdRW SerialRW8;
MagicBoat_CmdRW SerialRWSocket;
MagicBoat_CmdRW SerialRWSocketJet;

EBYTE LoRa(&Serial7, 41, 40, 39);

uint8_t pin485En = 26;

uint16_t tcp_port = 5050;
uint16_t tcp_port_jet = 5051;
EthernetServer ServerSocket(tcp_port);
EthernetServer ServerSocketJet(tcp_port_jet);

uint32_t time_receivingMission = 0;
uint16_t timer_receivingMission = 2000;
uint8_t receivingMission = 0;

#define dot Serial.print(".");
//*********************** SD *****************************
const int chipSelect = BUILTIN_SDCARD;
//********************* FLASH ****************************
LittleFS_QPINAND extFlash;
File flashFile;
File sleepFile;
char flashFilename[50];
//*********************** FRAM ***************************
MB85_FRAM_Class FRAM;
//********************** MEDIA ***************************
MagicBoat_Media mediaIMU[9];
MagicBoat_Media mediaMotor[12];
Crypto_T4 crypto;
uint8_t keyAes128[16] __attribute__((aligned)) = { 'o', 't', 't', 'o', 'o', 't', 't', 'o', 'o', 't', 't', 'o', 'o', 't', 't', 'o' };
uint8_t ive[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
uint8_t debug_time_sd = 0;
//******************** WATCHDOG **************************
WDT_T4<WDT1> wdt1;
//********************** SLEEP ***************************
SnoozeUSBSerial usb;
SnoozeDigital digital;
const uint8_t BUTTON = 6;
Bounce button = Bounce(BUTTON, 5);
SnoozeBlock config_teensy(usb, digital);
uint8_t sleep_on = 0;
//******************** PCF8575 ***************************
#define PWR_OFF 0
#define PWR_ON 1

#define PWR_LED 0
#define PWR_US_LIDAR 1
#define PWR_RADIO 2
#define PWR_JETSON 3
#define PWR_AIS 4
#define PWR_RUT 5
#define PWR_WIFI 6
#define PWR_COOL 7

uint8_t powerAddr = 0x20;
PCF8575 powerSel(powerAddr);
// uint8_t power_en[16] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
uint8_t power_en[16] = { PWR_ON, PWR_ON, PWR_ON, PWR_ON, PWR_ON, PWR_ON, PWR_ON, PWR_ON, PWR_OFF, PWR_OFF, PWR_OFF, PWR_OFF, PWR_OFF, PWR_OFF, PWR_OFF, PWR_OFF };
//********************** IMU *****************************
#ifdef BNO_055
MagicBoat_BNO055 imu;
#elif defined(BNO_086)
MagicBoat_BNO086 imu;
#endif
//********************** GPS *****************************
MagicBoat_GPS_F9X gps;
int32_t boatLat;
int32_t boatLon;

//******************** MOTORI ****************************
MagicBoat_VESC_CAN motors[4];

struct __attribute__((packed)) motorSet {
  int16_t erpm_driving[4] = { 0, 0, 0, 0 };
  uint8_t drive_off[4] = { 0, 0, 0, 0 };
} motorSetData;

struct __attribute__((packed)) JsCmd {
  uint8_t buttons[17];
  uint16_t POV = 0xFFFF;
  float axisX;
  float axisY;
  float wheel;
  float throttle;
} JsCmdData[4];  // TOFIX: perchè sono 4? sembra che venga usata solo la prima: perchè si controllano anche altri droni

struct __attribute__((packed)) lora_settings {
  uint8_t addressH = 0;     // 0-255
  uint8_t addressL = 0;     // 0-255
  uint8_t channel = 0;      // 0-31   Freq=410+channel*1MHz (433=23)
  uint8_t airDataRate = 0;  // 0=ADR_19200, 1=ADR_9600, 2=ADR_4800 or 3=ADR_2400
  uint8_t txPower = 0;      // 0=PWR_21dBm, 1=PWR_24dBm, 2=PWR_27dBm or 3=PWR_30dBm (30dBm = 1W)
} lora_settings_data;

int16_t rpm_avg_auto = 0;

uint8_t driveMode = MOTOR_ERPM_CMD3;
//********************* PID **************************
MagicBoat_PID pidRoute(&TetaB, &corrRoute, &TetaD, pidTheta.kp, pidTheta.ki, pidTheta.kd, DIRECT, P_ON_E, &pidTheta.AW, &pidTheta.IL);
MagicBoat_PID pidVelErpm(&Vel_GPS, &corrVel, &VelD, pidVel.kp, pidVel.ki, pidVel.kd, DIRECT, P_ON_E, &pidVel.AW, &pidVel.IL);
//********************* BMS **************************
#define CELL_NUM 8
MagicBoat_BMS bms(CELL_NUM);
//******************** RADIO *************************
float radioChValue[N_CH_READ];
//******************** MPPT **************************
struct __attribute__((packed)) hfrInt {
  uint16_t Iin;
  uint16_t Vin;
  uint16_t Vout;
  uint16_t Iout;
  uint16_t dutyPWM;
} hfrIntData[16];

struct __attribute__((packed)) mppt_debug {
  uint8_t debug_en = 0;
  uint8_t descr_en = 1;
  uint8_t head_en = 1;
  uint16_t timer_print_mppt = 200;
} mppt_debug_par;

uint32_t time_print_mppt;

//******************** GAV **************************
float pressureValues[16];
//******************** MUX/DEMUX **************************
#define MUX_ADDR0 5
#define MUX_ADDR1 4
#define MUX_ADDR2 3
#define MUX_ADDR3 2

uint8_t id_mux[16] = { ID_LED, ID_ECHO, ID_RADIOCOMANDO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t mux_read_en[16] = { 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t mux_ch_baud[16] = { 115200, 115200, 115200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint8_t muxAddr[4] = { MUX_ADDR0, MUX_ADDR1, MUX_ADDR2, MUX_ADDR3 };
uint8_t mux_index = 0;
uint8_t lock_index = sizeof(id_mux);

uint8_t testVar = 100;

//*************** RS485 **************************

#define CMD_RS485 0
#define BOW_RS485 1
#define STERN_RS485 2
#define AMB_RS485 3

uint32_t time_request_485 = 0;
uint32_t timer_request_485 = 100;

uint8_t index_485 = 1;
uint8_t index_485_old = 0;

uint8_t cmd_485_ok = 0;

//*************** EQUIPMENT **************************

#define MODULES_NUM 5

struct __attribute__((packed)) equip {
  uint8_t index_module = 255;
  uint8_t id_module;    // identifica di che modulo si tratta
  uint8_t sail;         // a vela o no
  uint8_t fairing;      // cupol+ino o no
  uint8_t motors;       // motori o no
  uint8_t batteries;    // batterie o no
  float batt_capacity;  // capacità batterie
  uint8_t bms;          // bms sì o no
} equipData[MODULES_NUM];

//***************  LED  **************************

#define NUM_LEDS 59

uint8_t debug_led_en = 0;

//***************  COOLING  **************************

uint8_t pinPwmCool = 36;
uint8_t pinPwmFlow = 9;

uint16_t timer_cool = 1000;
uint32_t time_cool = 0;

uint8_t pwmCool = 0;

//*************** SETUP **************************
// NON CAMBIARE ORDINE DI INIZIALIZZAZIONE
// Verificare se riesce ad avviarsi con dispositivi in avaria (LoRa..)
// dot aggiunge un punto sulla seriale (per debug se si blocca all'avvio)
void setup() {
  uint32_t startupTime = millis();
  //wdtInit(); //meglio sotto perche sopra si puo piantare
  dot flashInit();
  dot sleepInit();
  wdtInit();


  memoryInit();
  cmdRWInit();
  Serial.printf("Setup avviato.");



  dot bmsInit();
  dot powerInit();
  dot equipInit();
  dot sdInit();
  // dot mediaInit();
  dot gpsInit();
  dot motorInit();
  dot coolInit();
  dot imuInit();
  dot LoRaInit();
  Serial.printf("\nSetup completato in: ");
  Serial.printf("%2.2fs\n\n", (millis() - startupTime) / 1000.f);

}

//*************** LOOP **************************
uint32_t loopTime = 0;
void loop() {

  loopTime = millis();
  wdtReset();
  goSleepMode();  // check flag

  if (!restoreNavOk) {
    time_signal_jetson = millis();
  }

  computeIMU();  // Immagazzina i dati in arrivo dal GPS
  // storePSRAM();       // Fa la media IMU e motori poi salva i dati di telemetria sulla PSRAM
  //*printBufferAnalSD("analRW.bin");
  computeGPS();  // Immagazzina i dati in arrivo dal GPS
  //*testMotor();       // Fa girare i quattro motori con ERPM di test
  computeAllMotor();  // Chiede i dati di telemetria ai quattro motori
  boatControl();      // Processa la navigazione (GPS+RADIO / Auto+Manual)
  computeBms();       // Chiede i dati di telemetria al BMS -- Serial5
  computeMux();       // Processa il prossimo canale del [de]multiplexer della seriale.
  computeCool();      // Aggiorna parametri raffreddamento
  computeButtons();
  timerNextMission();  // Controlla se è l'ora della prossima missione
  cmdRW();             // TOFIX: verificare upload e commutazione nel timeframe corretto
  compute485();        // Processa le comunicazioni su RS485
  //computePower();      // Manda le accensioni e spengimenti
  loopTime = millis() - loopTime;
  debugPrint();  // Funzione che stampa su seriale USB tutti i dati di debug attivi
}
