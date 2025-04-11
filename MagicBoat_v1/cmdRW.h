//
#include <cstdint>
#pragma pack(push, 1)
struct paramId {
  uint8_t boat[3] = { 0x10, 0x11, 0x12 };
} id;
#pragma pack(pop)

#define REQUEST_CMD1 0
#define RESPONSE_CMD1 1
#define DEBUGMES_CMD1 2

#define ID_MODULO_BASE 0
#define ID_ALTO_LIVELLO 1
#define ID_INTERFACCIA 2
#define ID_WEBAPP 3

#define ID_MUX_MPPT 34
#define ID_RADIOCOMANDO 35

#define ID_MPPT 36

#define ID_IMU 37
#define ID_ECHO 38
#define ID_MOTORI 39
#define ID_BMS 40
#define ID_GPS 41
#define ID_POWER 42
#define ID_LED 43
#define ID_MICRO_JETSON 44
#define ID_LORA 45

#define ID_MODULO_AMB 50
#define ID_ROBOT_ARM_1 51
#define ID_ROBOT_ARM_2 52
#define ID_INCUBATORE 53
#define ID_OPENER 54
#define ID_QUANTITRAY 55

#define ID_PRUA 60
#define ID_POPPA 70

#define ID_PORTA_0 0
#define ID_PORTA_1 1
#define ID_PORTA_5 5
#define ID_PORTA_6 6
#define ID_PORTA_7 7
#define ID_PORTA_8 8

#define ID_PORTA_SOCK 9
#define ID_PORTA_SOCK_JET 10

#define PORTA_0 Serial
#if defined(ARDUINO_TEENSY41)
#define PORTA_1 Serial1
#define PORTA_5 Serial5
#define PORTA_6 Serial6
#define PORTA_7 Serial7
#define PORTA_8 Serial8
#endif


#define TEST_GENERIC_CMD2 254  //Comando per mandare un generico test dall'interfaccia
#define TEST_GENERIC_CMD3 254  //Comando per mandare un generico test dall'interfaccia

//*******************************MODULO BASE
#define EN_SLEEP_CMD2 0
#define SET_DEBUG_PORT_CMD2 1
#define SET_SD_CMD2 2
#define SET_FLASH_CMD2 3
#define SAVE_MISSION_CMD2 4
#define GET_MISSION_CMD2 5
#define SEND_SCHEDULE_CMD2 6
#define READ_SCHEDULE_CMD2 7
#define SEND_DUMMY_CMD2 8
#define START_NEXT_SCHED_CMD2 9
#define UPDATE_MISS_LIST_CMD2 10
#define START_THIS_MISS_CMD2 11
#define SET_MP_RADIUS_CMD2 12
#define EN_DEB_PRINT_CMD2 13
#define LOCK_MUX_CH_CMD2 14
#define REMOTE_CONTROL_CMD2 15
#define SET_NAV_GPS_CMD2 16
#define SET_NAV_HEAD_CMD2 17
#define JS_DRIVING_SET_CMD2 18
#define JS_DRIVING_DATA_CMD2 19
#define RADIO_DRIVING_SET_CMD2 20
#define RADIO_DRIVING_DATA_CMD2 21
#define SET_TEL_NAV_CMD2 22
#define SET_AUT_NAV_CMD2 23
#define SET_MODE_TETAD_CMD2 24
#define SET_MODE_VELD_CMD2 25
#define SET_MISC_PARAM_CMD2 26
#define SET_JS_DEB_CMD2 27
#define GET_CONTROL_INFO_CMD2 28
#define GET_ANTICOLL_TETAD_CMD2 29
#define GET_JETSON_SIGNAL_CMD2 30
#define SEND_DATA_JETSON_CMD2 31
#define GET_JETSON_WP_CMD2 32
#define SET_DRONE_EQUIP_CMD2 33
#define GET_IP_PORT_CMD2 34
#define SET_IP_PORT_CMD2 35
#define COOLING_SET_CMD2 36
#define REBOOT_TEENSY_CMD2 37
#define MAIN_DATA_CMD2 38

#define SD_SET_DEB_TIME_CMD3 0
#define SD_READ_CMD3 1
#define SD_DELETE_CMD3 2

#define FLASH_READ_CMD3 0
#define FLASH_DELETE_CMD3 1

#define SAVE_MISSION_PARAM_CMD3 0
#define SAVE_MISSION_WP_CMD3 1

#define GET_MISSION_PARAM_CMD3 0
#define GET_MISSION_WP_CMD3 1

#define START_UPDATE_LIST_CMD3 0
#define UPDATE_DIR_LIST_CMD3 1
#define UPDATE_FILE_LIST_CMD3 2
#define END_FILE_LIST_CMD3 3

#define INPUT_JOYSTICK_CMD3 0
#define INPUT_RADIO_CMD3 1

#define JS_BOAT_CMD3 0
#define JS_AIR_DRONE_CMD3 1
#define JS_SUBM_CMD3 2
#define JS_ARM_CMD3 3

//*******************************POWER
#define POWER_EN_CMD2 0         //All devices struct epected
#define POWER_EN_JETSON_CMD2 1  //Jetson power only expected

#define POWER_EN_GET_CMD3 0
#define POWER_EN_SET_CMD3 1
//*********************************IMU
#define IMU_SET_CMD2 0
#define IMU_GET_CMD2 1
#define IMU_CFG_CMD2 2
#define IMU_DEB_CFG_CMD2 3
#define IMU_086_SRESET_CMD2 4
#define IMU_086_HRESET_CMD2 5

#define IMU_RPY_CMD3 0
#define IMU_RPY_ACC_CMD3 1
#define IMU_RPY_ACC_GYR_CMD3 2
#define IMU_RPY_ACC_GYR_MAG_CMD3 3
#define IMU_GET_086_CAL_CMD3 4

#define IMU_086_SET_REPORTS_CMD3 4
#define IMU_086_REQ_CAL_STA_CMD3 5
#define IMU_086_DEB_MES_EN_CMD3 6

#define IMU_UPDATE_CFG_GET_CMD3 0
#define IMU_UPDATE_CFG_SET_CMD3 1
#define IMU_UPDATE_CAL_GET_CMD3 2
#define IMU_UPDATE_CAL_SET_CMD3 3

#define IMU_DEB_CFG_GET_CMD3 0
#define IMU_DEB_CFG_SET_CMD3 1

//******************************MOTORI
#define MOTOR_DRIVE_CMD2 0
#define MOTOR_TELEM_CMD2 1

#define MOTOR_OFF_CMD3 0
#define MOTOR_ERPM_CMD3 1
#define MOTOR_CURRENT_CMD3 2
#define MOTOR_ERPM_ALL_CMD3 3

#define MOTOR_TELEM_CDCS_CMD3 0
#define MOTOR_TELEM_DDSS_CMD3 1

#define MOTOR_CD 1
#define MOTOR_CS 2
#define MOTOR_DD 3
#define MOTOR_SS 4

#define INDEX_MOT_DD 0
#define INDEX_MOT_SS 1
#define INDEX_MOT_CD 2
#define INDEX_MOT_CS 3

#define AXIS_X 0
#define AXIS_Y 1
#define THROTTLE 2
#define WHEEL 3

//*********************************BMS

#define BMS_PARAM_CMD2 0
#define BMS_GET_DATA_CMD2 1
#define BMS_DEB_CFG_CMD2 2

#define BMS_GET_VCELL_CMD3 0
#define BMS_GET_BASIC_CMD3 1
#define BMS_GET_EEPROM_CMD3 2

#define BMS_GET_PARAM_CMD3 0
#define BMS_SET_PARAM_CMD3 1

#define BMS_DEB_CFG_GET_CMD3 0
#define BMS_DEB_CFG_SET_CMD3 1

//*********************************GPS

#define GPS_GET_CMD2 0
#define GPS_SET_CMD2 1
#define GPS_DEB_CFG_CMD2 2

#define GPS_NAV_PVT_CMD3 0
#define GPS_NAV_RELPOSNED_CMD3 1

#define GPS_DEB_CFG_GET_CMD3 0
#define GPS_DEB_CFG_SET_CMD3 1

//*********************************ECHO
#define ECHO_NANO_GET_CMD2 0

#define ECHO_GET_CMD2 0
#define ECHO_CFG_CMD2 1
#define ECHO_DEB_CFG_CMD2 2

#define ECHO_CFG_GET_CMD3 0
#define ECHO_CFG_SET_CMD3 1

#define ECHO_DEB_CFG_GET_CMD3 0
#define ECHO_DEB_CFG_SET_CMD3 1

//*********************************LED
#define RING_LIGHT_CMD2 0
#define STRIP_LIGHT_CMD2 1
#define FRONT_LIGHT_CMD2 2
#define IR_LIGHT_CMD2 3
#define LIGHT_SENSOR_CMD2 4

//RING LIGHT
#define RING_LIGHT_DEBUG_CMD3 0
#define RING_LIGHT_MANUAL_CMD3 1
//STRIP_LIGHT
#define STRIP_LIGHT_OFF_CMD3 0
#define STRIP_LIGHT_LOW_CMD3 1
#define STRIP_LIGHT_HIGH_CMD3 2
#define STRIP_LIGHT_AUTO_LOW_CMD3 3
//FRONT_LIGHT
#define FRONT_LIGHT_OFF_CMD3 0
#define FRONT_LIGHT_ON_CMD3 1
#define FRONT_LIGHT_FLASH_CMD3 2
//IR_LIGHT
#define IR_LIGHT_OFF_CMD3 0
#define IR_LIGHT_ON_CMD3 1
#define IR_LIGHT_AUTO_CMD3 2

//*********************************LORA
#define LORA_GET_CONFIG_CMD2 0
#define LORA_SET_CONFIG_CMD2 1

//*********************************MPPT
#define SINC_CHAR_MPPT_0 36
#define SINC_CHAR_MPPT_1 36
#define SINC_CHAR_MPPT_2 69

uint8_t id_mppt = SINC_CHAR_MPPT_0;
#define MPPT_CMD1 0

#define MPPT_NANO_GET_CMD2 0
#define MPPT_GET_CMD2 1
#define MPPT_SET_CMD2 2
#define MPPT_DEB_CMD2 3

#define MPPT_DEB_GET_CMD3 0
#define MPPT_DEB_SET_CMD3 1

#define MPPT_0_GET_CMD3 0
#define MPPT_1_GET_CMD3 1
#define MPPT_2_GET_CMD3 2
#define MPPT_3_GET_CMD3 3

#define VI_CMD2 5

#define DIS_VI_NF_CMD3 0
#define EN_VI_NF_CMD3 1


#define REGI_VAL_CMD1 1
#define SET_BATT_CMD2 12
#define SET_BATT_NCELL_CMD3 4

//*********************************ricevente
#define N_CH_READ 6

#define CH_DX_X 0
#define CH_DX_Y 1

#define CH_SX_Y 2
#define CH_SX_X 3

#define CH_SX_ROT 4
#define CH_DX_ROT 5
//*********************************MODULO AMB
#define REG_NEW_WP_SD_CMD2 0
#define READ_SD_WP_FILE_CMD2 1
//#define SET_SD_CMD2               2  // In comune con modulo base e interfaccia
#define SET_AMB_POWER_CMD2 3
#define REG_ACTION_CMD2 4
#define REG_HEADER_CMD2 5
#define READ_SAMPLING_CMD2 6
#define MISSION_AMB_EXEC_CMD2 7
//#define UPDATE_MISS_LIST_CMD2     10 // In comune con modulo base e interfaccia
//#define START_SERVO_MISSION_CMD2  12 // In comune fra modulo ambientale, interfaccia, bracci

//#define SET_DRONE_EQUIP_CMD2      33 // In comune con modulo base e interfaccia
//#define REBOOT_TEENSY_CMD2        37 // In comune con modulo base e interfaccia
//#define MAIN_DATA_CMD2            38 // In comune con modulo base e interfaccia
//*********************************ROBOT ARM
#define GET_DATA_ARM_CMD2 0
#define SERVO_DRIVE_PER_CMD2 1
#define SERVO_DRIVE_ANG_CMD2 2
#define NEW_POINTS_CMD2 3
#define REG_STRUCT_MIN_CMD2 4
#define REG_STRUCT_MAX_CMD2 5
#define GO_TO_RIF_VITE_CMD2 6
#define GO_TO_ZERO_VITE_CMD2 7
#define SET_CURR_CLAW_CMD2 8
#define SERVO_GET_PER_CMD2 9
#define SERVO_GET_ANG_CMD2 10
#define SET_PID_PARAM_VITE_CMD2 11
//********************************* INCUBATORE
#define GET_DATA_INC_CMD2 0
#define SET_TEMP_RIF_CMD2 1
#define DOOR_APRI_CHIUDI_CMD2 2
#define TOGGLE_BACKLIGHT_CMD2 3
#define SLEEP_INCUBATOR_CMD2 4

#define START_INCUB_MISSION_CMD2 15

#define SET_TEMP_RIF_UP_CMD3 0
#define SET_TEMP_RIF_DW_CMD3 1

#define DOOR_APRI_UP_CMD3 0
#define DOOR_APRI_DW_CMD3 1
#define DOOR_CHIUDI_UP_CMD3 2
#define DOOR_CHIUDI_DW_CMD3 3
#define DOOR_STOP_CMD3 4
//********************************* APRIBARATTOLO
#define GET_DATA_JAR_CMD2 0
#define OPEN_JAR_CMD2 1
#define CLOSE_JAR_CMD2 2
#define JAR_START_CONTROL_CMD2 3
#define JAR_STOP_CONTROL_CMD2 4

#define OPEN_JAR_MOT_CMD3 0
#define CLOSE_JAR_MOT_CMD3 1
#define OPEN_CAP_MOT_CMD3 2
#define CLOSE_CAP_MOT_CMD3 3
#define ROTATE_JAR_CW_CMD3 4
#define ROTATE_JAR_CCW_CMD3 5
//********************************* QUANTITRAY
#define MOVE_JARS_CMD2 0
#define MOVE_PISTON_CMD2 1
#define MOVE_MOTOR_CMD2 2

#define RETRACT_PISTON_CMD3 0
#define PUSH_PISTON_CMD3 1
#define STOP_PISTON_CMD3 2

#define MOVE_MOTOR_CW_CMD3 0
#define MOVE_MOTOR_CCW_CMD3 1
#define STOP_MOTOR_CMD3 2
//********************************* PRUA
#define GET_PRESSURE_CMD2 0
#define SET_PUMP_PARAM_CMD2 1
#define SET_COMPRESSOR_CMD2 2
#define SET_VALVES_CMD2 3
#define OPEN_MOD_DOOR_CMD2 4
#define CLOSE_MOD_DOOR_CMD2 5
#define SET_DEB_PRESS_CMD2 6
#define SET_GAV_PARAMS_CMD2 7
#define BOAT_OVERTURN_CMD2 8
#define EN_MODULE_BMS_CMD2 9
//****************************Indici per salvare lo stato di comunicazione
#define INDEX_PINGPONG_MISSION 0
//****************************Per l'albero dei file della flash
#define END_OF_STRING 0
#define DIRECTORY_CMD 1
#define FILE_CMD 2
#define PARENT_CMD 3
//****************************struttura messaggio Inviato
#define INDEX_SINCHAR_0 0
#define INDEX_SINCHAR_1 1
#define INDEX_SINCHAR_2 2

#define INDEX_BUF_LENG 3
#define INDEX_BUF_SORG 4
#define INDEX_BUF_DEST 5
#define INDEX_BUF_ID_D 6

#define INDEX_BUF_CMD_1 7
#define INDEX_BUF_CMD_2 8
#define INDEX_BUF_CMD_3 9
#define INDEX_BUF_CONTB 10

#pragma pack(push, 1)

struct mess {
  uint8_t lengCmd = 0;
  uint8_t sorgCmd = 0;
  uint8_t destCmd = 0;
  uint8_t id_dCmd = 0;

  uint8_t Cmd1 = 0;
  uint8_t Cmd2 = 0;
  uint8_t Cmd3 = 0;
} mes, mes485;

#pragma pack(pop)
