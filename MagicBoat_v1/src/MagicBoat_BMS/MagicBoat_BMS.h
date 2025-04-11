#ifndef MagicBoat_BMS_h
#define MagicBoat_BMS_h

#include "Arduino.h"

#define SerialBMS Serial5

#define DEBUG_VCELL true
#define DEBUG_BASIC_INFO true
#define DEBUG_MINIMAL_INFO false
#define HEADERS_ON false

#define START_REG_RESP 0x03
#define END_REG_RESP 0xE2

#define REG_BASIC_INFO 0x03
#define REG_VCELL 0x04

//Variabili attivazione carica/scarica attive alte 0-->ON
#define CHG_ON 0
#define CHG_OFF 1

#define DSG_ON 0
#define DSG_OFF 1

//----------------------------//
/* ---- EEPROM REGISTERS ---- */
//----------------------------//

#define DESIGN_CAP_REG 0x10          //Pack Capacity, as designed
#define CYCLE_CAP_REG 0x11           //Pack Capacity, per cycle
#define TEN_CAP_VOLTAGE_REG 0x12     //100% Capacity estimate voltage
#define EIGHT_CAP_VOLTAGE_REG 0x32   // 80% Capacity estimate voltage
#define SIX_CAP_VOLTAGE_REG 0x33     // 60% Capacity estimate voltage
#define FOUR_CAP_VOLTAGE_REG 0x34    // 40% Capacity estimate voltage
#define TWO_CAP_VOLTAGE_REG 0x35     // 20% Capacity estimate voltage
#define ZERO_CAP_VOLTAGE_REG 0x13    //  0% Capacity estimate voltage
#define DISCHARGE_RATE_REG 0x14      //	Cell estimated self discharge rate
#define CYCLE_CNT_REG 0x17           //	Cycle count
#define CHG_OT_REG 0x18              //	Charge Over Temperature threshold
#define CHG_OT_REL_REG 0x19          //	Charge Over Temperature release threshold
#define CHG_UT_REG 0x1A              //	Charge Under Temperature threshold
#define CHG_UT_REL_REG 0x1B          //	Charge Under Temperature release threshold
#define DSG_OT_REG 0x1C              //	Discharge Over Temperature threshold
#define DSG_OT_REL_REG 0x1D          //	Discharge Over Temperature release threshold
#define DSG_UT_REG 0x1E              //	Discharge Under Temperature threshold
#define DSG_UT_REL_REG 0x1F          //	Discharge Under Temperature release threshold
#define POVP_REG 0x20                //	Pack Over Voltage Protection threshold
#define POVP_REL_REG 0x21            //	Pack Over Voltage Protection release threshold
#define PUVP_REG 0x22                //	Pack Under Voltage Protection threshold
#define PUVP_REL_REG 0x23            //	Pack Under Voltage Protection release threshold
#define COVP_REG 0x24                //	Cell Over Voltage Protection threshold
#define COVP_REL_REG 0x25            //	Cell Over Voltage Protection release threshold
#define CUVP_REG 0x26                //	Cell Under Voltage Protection threshold
#define CUVP_REL_REG 0x27            //	Cell Under Voltage Protection release threshold
#define COCP_REG 0x28                //	Charge Over Current threshold
#define DOCP_REG 0x29                //	Discharge Over Current threshold
#define BAL_START_REG 0x2A           //	Cell Balance voltage
#define BAL_WINDOW_REG 0x2B          //	Balance Window
#define SHUNT_RES_REG 0x2C           //	Ampere measurement shunt resistor value
#define FUNC_CONFIG_REG 0x2D         //	Various functional config bits
#define NTC_CONFIG_REG 0x2E          //	Enable/disable NTCs
#define CELL_CNT_REG 0x2F            //	Cells count read
#define COVP_HIGH_REG 0x36           //	Secondary Cell Over Voltage Protection
#define CUVP_HIGH_REG 0x37           //	Secondary Cell Under Voltage Protection
#define SHORT_DSGOC_HIGH_REG 0x38    //	Short circuit & Secondary Over Current Protection
#define CXVP_HIGH_DELAY_SC_REG 0x39  //	Secondary Cell Under/Over Voltage release time, and Short Circuit release time
#define MOSFET_CONTROL_REG 0xE1      //	MOSFET Control


#define CHGT_DELAY_REG 0x3A  //	Charge Under/Over Temperature Release Delay
#define DSGT_DELAY_REG 0x3B  //	Discharge Under/Over Temperature Release Delay
#define PACK_DELAY_REG 0x3C  //	Pack Under/Over Temperature Release Delay
#define CELL_DELAY_REG 0x3D  //	Cell Under/Over Temperature Release Delay
#define CHGC_DELAY_REG 0x3E  //	Charge Over Current Release Delay
#define DSGC_DELAY_REG 0x3F  //	Discharge Over Current Release Delay

//--------------------------------//
/* ---- END EEPROM REGISTERS ---- */
//--------------------------------//


#define SOP 0xDD
#define EOP 0X77
#define ERR 0x80

#define DW0 0xDD
#define DW1 0x5A
#define DW3 0x02
#define DW8 0x77

#define DEB_BMS_VCELL 0
#define DEB_BMS_BASIC 1
#define DEB_BMS_EEPROM 2

class MagicBoat_BMS {
private:


public:
  MagicBoat_BMS(uint8_t num);

  uint8_t num_cell_set;
  uint8_t inArr[100];
  uint8_t index = 0;
  uint8_t Length = 0;
  uint8_t regAddrResp = 0;
  uint32_t checksum = 0;
  uint16_t cks0 = 0;
  uint16_t cks1 = 0;
  uint16_t cks_temp0 = 0;
  uint16_t cks_temp1 = 0;
  uint8_t stepBMS = 0;
  uint8_t read_ok = 0;
  uint8_t start_register_write = 0;
  uint8_t start_read_eprom = 0;
  uint8_t enable_read_all_eeprom = 0;
  uint16_t time_start_register_write = 1000;
  uint32_t timer_start_register_write = 0;
  float vcell[24];
  uint8_t index_read_eeprom_max = 45;
  uint8_t index_read_eeprom = 0;

  struct __attribute__((packed)) debug {
    uint8_t debug_enable[3] = { 0, 0, 0 };
    uint8_t debug_en_descr[3] = { 0, 0, 0 };
    uint16_t debug_timer[3] = { 100, 100, 100 };
    uint8_t debug_en_head[3] = { 0, 0, 0 };
  } debug_par;

  uint32_t debug_time[3] = { 1, 1, 1 };

  struct __attribute__((packed)) start {
    uint8_t ncell = 8;
    float povp = 32;
    float povp_rel = 31.8;
    float puvp = 22.5;
    float puvp_rel = 22.7;
    uint8_t chg_on_off = CHG_ON;  //attivo basso 0 -> ON
    uint8_t dsg_on_off = DSG_ON;  //attivo basso 0 -> ON
    uint8_t read_eeprom_on_off = 0;
  } start_par;

  /*---- Basic info variables ----*/

  struct __attribute__((packed)) BasicInfo {
    float PackVoltage;
    float PackCurrent;
    float RemainCapacity;
    float FullCapacity;
    uint16_t CyclesNum;
    uint8_t balance_status[32];

    //Errors flags
    uint8_t cell_ov;
    uint8_t cell_uv;
    uint8_t pack_ov;
    uint8_t pack_uv;
    uint8_t charge_ot;
    uint8_t charge_ut;
    uint8_t discharge_ot;
    uint8_t discharge_ut;
    uint8_t charge_oc;
    uint8_t discharge_oc;
    uint8_t short_circuit;
    uint8_t frontend_error;
    uint8_t locked_FET;
    //End Errors flags

    uint8_t RSOC;
    uint8_t FET_charge;
    uint8_t FET_discharge;
    float NTC1;
    float NTC2;
  } BasicData;

  /*---- EEPROM read variables ----*/

  struct __attribute__((packed)) EepromInfo {
    float design_cap;     //0x10
    float cycle_cap;      //0x11
    float cap100;         //0x12
    float cap80;          //0x32
    float cap60;          //0x33
    float cap40;          //0x34
    float cap20;          //0x35
    float cap0;           //0x13
    float dsg_rate;       //0x14
    uint16_t mfg_rate;    //0x15
    uint16_t serial_num;  //0x16
    uint16_t cyc_cnt;     //0x17
    float chg_ot;         //0x18
    float chg_ot_rel;     //0x19
    float chg_ut;         //0x1A
    float chg_ut_rel;     //0x1B
    float dsg_ot;         //0x1C
    float dsg_ot_rel;     //0x1D
    float dsg_ut;         //0x1E
    float dsg_ut_rel;     //0x1F
    float povp;           //0x20
    float povp_rel;       //0x21
    float puvp;           //0x22
    float puvp_rel;       //0x23
    float covp;           //0x24
    float covp_rel;       //0x25
    float cuvp;           //0x26
    float cuvp_rel;       //0x27
    float cocp;           //0x28
    float docp;           //0x29
    float bal_start;      //0x2A
    float bal_window;     //0x2B
    float shunt_res;      //0x2C
    uint8_t switch_en;    //0x2D bit0
    uint8_t scrl_en;      //0x2D bit1
    uint8_t bal_en;       //0x2D bit2
    uint8_t ch_bal_en;    //0x2D bit3
    uint8_t led_en;       //0x2D bit4
    uint8_t led_num_en;   //0x2D bit5
    uint8_t ntc[8];       //0x2E
    uint8_t cell_cnt;     //0x2F
    uint8_t fet_ctrl;     //0x30
    uint8_t led_timer;    //0x31
    float covp_high;      //0x36
    float cuvp_high;      //0x37

    uint8_t sc_dsgoc_x2;   //0x38 byte 0 -> bit 7
    uint8_t sc_delay;      //0x38 byte 0 -> bit [4:3]
    uint8_t sc;            //0x38 byte 0 -> bit [2:0]
    uint8_t dsgoc2_delay;  //0x38 byte 1 -> bit [7:4]
    uint8_t dsgoc2;        //0x38 byte 1 -> bit [3:0]

    uint8_t cuvp_high_delay;  //0x39 byte 0 -> bit [7:6]
    uint8_t covp_high_delay;  //0x39 byte 0 -> bit [5:4]
    float sc_rel_time;        //0x39 byte 1

    float chg_ut_delay;  //0x3A byte 0
    float chg_ot_delay;  //0x3A byte 1

    float dsg_ut_delay;  //0x3B byte 0
    float dsg_ot_delay;  //0x3B byte 1

    float puvp_delay;  //0x3C byte 0
    float povp_delay;  //0x3C byte 1

    float cuvp_delay;  //0x3D byte 0
    float covp_delay;  //0x3D byte 1

    float cocp_delay;      //0x3E byte 0
    float cocp_rel_delay;  //0x3E byte 1

    float docp_delay;      //0x3F byte 0
    float docp_rel_delay;  //0x3F byte 1

    //Continua...

  } EepromData;


  void begin(uint32_t baudrate);

  uint8_t get_bms_feedback();
  void read_eprom(uint8_t reg_Addr);
  uint8_t analBuffer();

  void setCellNum(uint8_t numcell_set);
  void setCutOff(float povp, float povp_r, float puvp, float puvp_r);
  void setMOSFET(uint8_t charge, uint8_t discharge);

  void request_vcell_info();
  void request_basic_info();

  void write_register(uint8_t regAddr, uint16_t value);
  void write_request_start();
  void write_request_end();
  void e_write_request_end();
  uint8_t checksum_calc(uint8_t *data_write);
  uint8_t en_dis_read_eprom(uint8_t en_dis);
  uint8_t readAllParam();

  uint16_t parse16(uint8_t highbyte, uint8_t lowbyte);
  void flush();
};

#endif