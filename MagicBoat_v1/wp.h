//*************Define per range variabili Header e WP*******************

#define NAV_M_VEL_ERPM 0
#define NAV_M_VEL_GPS 1
#define NAV_M_ENERGY 2

#define NAV_LIM_1 10
#define NAV_LIM_2 20
#define NAV_LIM_3 30

#define ARRIVE_M_NO_MIDPOINTS 0
#define ARRIVE_M_SIMPLE 1
#define ARRIVE_M_HAVERSINE 2

#define ARRIVE_LIM_1 50
#define ARRIVE_LIM_2 100
#define ARRIVE_LIM_3 150

#define NAV_M_ERPM 0
#define NAV_M_VEL_GPS 1
#define NAV_M_ENER_COMP 2

#define NAV_M_LIM_1 10
#define NAV_M_LIM_2 20
#define NAV_M_LIM_3 30

//****************************Per decidere cosa fare alla fine della missione

#define NO_NEXT_MISSION 0
#define TO_NEXT_MISSION 1
#define TO_FINAL_WP_AND_NEXT 2
#define STAND_AT_FINAL_WP 3
#define STAND_AT_FINAL_WP_CYCLE 4

uint32_t timer_coord = 100;
uint32_t time_coord = 0;

//****************************Per decidere se il controllo sul segnale è presente, poi diventeranno funzioni che restituiscono un valore

#define SIGNAL_STRENGTH_CHECK true
#define SATELLITE_OFF true
#define SENSOR_SIGNAL_OFF true

//****************************Suffissi

#define SUFF_0 0
#define CHECK_SIGNAL_SUFF 1
#define SUFF_2 2
#define SUFF_3 3
#define SUFF_4 4
#define SUFF_5 5
#define SUFF_6 6
#define SUFF_7 7
#define SUFF_8 8
#define SUFF_9 9

//*******************WAYPOINT***************************
uint16_t nWP_now = 0;
uint16_t total_mission_nWP = 0;

struct __attribute__((packed)) wayPoint {
  uint16_t Nmissione;  // Nome del file contenente il waypoint
  uint16_t IndexWP;    // Indice dell'attuale waypoint
  float Latitude;      // Latitudine attuale WP
  float Longitude;     // Latitudine attuale WP
  uint8_t NavMode;     // [1 - 10]: definita come velocità erpm base all'indice [NavMode] dell'array di 10 velocità -> modifica erpm_base dei motori
  //************* // [11 - 19]: definita come velocità gps all'indice [NavMode] dell'array di 10 velocità -> modifica rif. di velocità gps
  //************* // [21 - 30]: definita come compensazione di energia all'indice [NavMode] dell'array di 10 livelli -> modifica erpm_max/min dei motori
  uint8_t PointType;     // 0: fermo; 1: non fermo; 2: fa parte di un'area; 3: stazionamento in un raggio
  uint8_t MonitoringOp;  //[1 - 125]: ti fermi ed effettui monitoraggio; 126 - 255: non ti fermi ed effettui monitoraggio; 0: non ti fermi e vai avanti
  uint8_t ArriveMode;    // [0 - 49]: senza vincoli intermedi
  //*****************// [50 - 99]: algoritmo punti intermedi con retta cartesiana
  //*****************// [100 - 149]: algoritmo punti intermedi con emisenoverso
  float WaypointRadius;  //Raggio entro il quale passare al WP successivo
  //16.600.000 waypoint totali sulla flash

} wayPointData, wayPointDataCksum, wayPointDataCurrentMission;

struct __attribute__((packed)) missionParam {
  char idMission[32];          //Mission ID
  uint16_t nMission;           //Mission number
  uint16_t total_mission_nWP;  //Number of waypoints of current mission
  uint16_t wpStart;            //Start waypoint of repetition cycle
  uint16_t cycles;             //How many execution cycles of the current mission
  uint16_t wpEnd;              //Final waypoint of repetition cycle
  uint8_t NMmode;              //What to do next (0,1,2,3,4) - 0: nessuna missione; 1: da fine cicla vai alla successiva;
                               //  2: dopo tutti i waypoint vai alla successiva; 3: staziona all'ultimo WP della missione fino a nuovo ordine
                               //  4: all'ultimo WP del ciclo stazioni in un certo raggio fino a nuovo ordine
  uint16_t NMnum;              //Number of the next mission
  uint16_t NMStartInd;         //Start waypoint of next mission
  char idMissionNext[32];      //String with the path of the mission
  float standRadius;           //WP standing mode radius
} missionParamData, missionParamDataCksum, missionParamDataCurrent;

struct __attribute__((packed)) schedule {
  uint16_t schedule_index;
  char idSchedule[32];
  char idMission[32];  //Mission ID
  uint8_t isEnabled = 1;
  uint16_t year;  //UTC
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint16_t nMission;  //Mission number
  uint16_t missionStartInd;
  int16_t isRepeated = 1;
  uint8_t repMonths;
  uint8_t repDays;
  uint8_t repHours;
  uint8_t repMins;
  char idMissionLast[32];  //Last Mission Id
} scheduleData, scheduleDataCksum, scheduleDataNext;

struct __attribute__((packed)) dateTime {
  uint16_t year;  //UTC
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
} dateTimeNew;

File flashRoot;

uint32_t mission_cksum = 0;
uint32_t schedule_cksum = 0;
uint8_t debug_schedule = 0;
uint32_t deltaTime;
uint32_t DateNextMission = 0;
uint16_t indexWPNow = 0;
uint8_t mission_active = 0;
uint8_t standActive = 0;

int32_t rifLatMission;
int32_t rifLonMission;

int32_t rifLatTrue = 100;
int32_t rifLonTrue;

int32_t rifLatMissionOld;
int32_t rifLonMissionOld;

int32_t rifLatJetson;
int32_t rifLonJetson;

float TetaB_DiffGPS = 0;
float TetaB_VelGPS = 0;
float TetaB_IMU = 0;
float declination = 0;
float errTetaB_Vel = 0;
float errTetaB_Diff = 0;

double radius_start = 5;
uint32_t index_radius = 1;
double interStep = 5;  //precisione con cui dividiamo in WP intermedi
uint32_t index_step = 1;
uint8_t arriveModeNow = 0;

double earthR = 6371000;

uint32_t timer_change_wp = 0;

uint32_t RaggioGPS = 20;
uint32_t RaggioGPSTrue = 6;  //threshold prossimo wp
uint32_t RaggioStand = 20;

uint16_t index_buff;
uint16_t fileCount = 0;
uint16_t startIndex = 0;

uint8_t goBack = 0;
uint8_t missionCycleNum = 0;
uint8_t missionCycle = 0;

uint8_t restoreNavOk = 0;

char folderArray[255];