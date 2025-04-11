#define BMS_CHANGED false
//Variabili timer per il tick di richiesta dati
uint32_t timer_get_vcell = 2000;
uint32_t time_get_vcell = 0;
uint32_t timer_get_basic_info = 500;
uint32_t time_get_basic_info = 0;

uint8_t enable_new_get_bms = 0;
uint8_t index_read_eprom = 0;
uint8_t toggle_read_vcell = 1;  // Tick vcell
uint8_t tick_read_vcell = 0;

uint32_t time_print_vcell = 0;
uint32_t time_print_basic_info = 0;
uint32_t time_print_eeprom = 0;

void bmsInit() {
  SerialBMS.begin(9600);  //BMS a 9600 di default
  //bms.setCellNum(bms.num_cell_set); // Setto il numero di celle
  //bms.setCutOff(POVP,POVP_REL,PUVP,PUVP_REL); // Setto i cutoff
  bms.setMOSFET(CHG_ON, DSG_ON);  // Abilito/disabilito carica/scarica
  delay(1000);
  setDsgBms(ID_PRUA, DSG_ON);
  //bms.en_dis_read_eprom(1);  // Abilito/disabilito lettura EEPROM
}

//Ogni N volte che leggo le Basic Info, leggo una volta le tensioni di cella
uint32_t bmsTime = 0;
void computeBms() {
  bmsTime = millis();
  if(BMS_CHANGED)
  {
    bms.start_register_write = 0;
  }
  if ((!bms.start_register_write) && ((enable_new_get_bms) || (millis() - time_get_vcell) > timer_get_vcell)) {
    time_get_vcell = millis();
    bms.readAllParam(); //se abilitata la lettura della eprom manda la richiesta di lettura di un parametro alla volta fino a leggerli tutti

    if (!tick_read_vcell) {
      bms.request_vcell_info();
    } else {
      bms.request_basic_info();
    }

    tick_read_vcell++;
    if (tick_read_vcell > toggle_read_vcell) {
      tick_read_vcell = 0;
    }
  }

  enable_new_get_bms = bms.get_bms_feedback();//questa legge la roba sulla seriale del bms
  if (enable_new_get_bms == REG_BASIC_INFO) {
    PowerBoat = (bms.BasicData.PackCurrent) * (bms.BasicData.PackVoltage);
  }
  bmsTime = millis() - bmsTime;
}

/* ------------ FUNCTIONS TO PRINT BMS DEBUG --------------- */



//*************** Stampa le tensioni di cella *********************

void printBmsVcell(uint8_t description, uint8_t head) {
  for (int j = 0; j < (bms.num_cell_set); j++) {
    if (description) {
      Serialprint(F("Cell"));
      Serialprint(j + 1);
      Serialprint(F(":"));
    }
    Serialprint(bms.vcell[j], 3);
    Serialprint(F(","));
  }
  if (head) {
    Serialprintln();
  }
}


//*************** Stampa le info di base in versione completa o minima *********************

void printBmsBasic(uint8_t description, uint8_t head) {
  if (description) {
    Serialprint(F("Pack Voltage:"));
  }
  Serialprint(bms.BasicData.PackVoltage);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Current:"));
  }
  Serialprint(bms.BasicData.PackCurrent);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Full Capacity:"));
  }
  Serialprint(bms.BasicData.FullCapacity);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("FET_CH:"));
  }
  Serialprint(bms.BasicData.FET_charge);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("FET_DIS:"));
  }
  Serialprint(bms.BasicData.FET_discharge);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Charge/Discharge Cycles:"));
  }
  Serialprint(bms.BasicData.CyclesNum);
  Serialprint(F(","));

  if (bms.num_cell_set <= 16) {
    for (int i = 0; i < bms.num_cell_set; i++) {
      if (description) {
        Serialprint(F("Bal"));
        Serialprint(i + 1);
        Serialprint(F(":"));
      }
      Serialprint(bms.BasicData.balance_status[i]);
      Serialprint(F(","));
    }
  } else if (bms.num_cell_set > 16) {
    for (int i = 16; i < bms.num_cell_set; i++) {
      if (description) {
        Serialprint(F("Bal"));
        Serialprint(i + 1);
        Serialprint(F(":"));
      }
      Serialprint(bms.BasicData.balance_status[i]);
      Serialprint(F(","));
    }
    for (int i = 0; i < 15; i++) {
      if (description) {
        Serialprint(F("Bal"));
        Serialprint(i + 1);
        Serialprint(F(":"));
      }
      Serialprint(bms.BasicData.balance_status[i]);
      Serialprint(F(","));
    }
  }

  if (description) {
    Serialprint(F("Error count:"));
    Serialprint(F("Cell OV:"));
  }
  Serialprint(bms.BasicData.cell_ov);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Cell UV:"));
  }
  Serialprint(bms.BasicData.cell_uv);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Pack OV:"));
  }
  Serialprint(bms.BasicData.pack_ov);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Pack UV:"));
  }
  Serialprint(bms.BasicData.pack_uv);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Chg OT:"));
  }
  Serialprint(bms.BasicData.charge_ot);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Chg OT:"));
  }
  Serialprint(bms.BasicData.charge_ut);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Dsg OT:"));
  }
  Serialprint(bms.BasicData.discharge_ot);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Dsg UT:"));
  }
  Serialprint(bms.BasicData.discharge_ut);
  Serialprint(F(" "));
  if (description) {
    Serialprint(F("Chg OC:"));
  }
  Serialprint(bms.BasicData.charge_oc);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Dsg OC:"));
  }
  Serialprint(bms.BasicData.discharge_oc);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Short Circuit:"));
  }
  Serialprint(bms.BasicData.short_circuit);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Frontend Err:"));
  }
  Serialprint(bms.BasicData.frontend_error);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Locked FET:"));
  }
  Serialprint(bms.BasicData.locked_FET);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("RSOC:"));
  }
  Serialprint(bms.BasicData.RSOC);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("NTC1:"));
  }
  Serialprint(bms.BasicData.NTC1);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("NTC2:"));
  }
  Serialprint(bms.BasicData.NTC2);
  if (head) {
    Serialprintln();
  }
}

//*************** Stampa i dati letti dalla EEPROM *********************

void printBmsEeprom(uint8_t description, uint8_t head) {
  if (description) {
    Serialprint(F("Pack Capacity:"));
  }
  Serialprint(bms.EepromData.design_cap);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Cycle Capacity:"));
  }
  Serialprint(bms.EepromData.cycle_cap);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("100% Capacity Voltage:"));
  }
  Serialprint(bms.EepromData.cap100, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("80% Capacity Voltage:"));
  }
  Serialprint(bms.EepromData.cap80, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("60% Capacity Voltage:"));
  }
  Serialprint(bms.EepromData.cap60, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("40% Capacity Voltage:"));
  }
  Serialprint(bms.EepromData.cap40, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("20% Capacity Voltage:"));
  }
  Serialprint(bms.EepromData.cap20, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("0% Capacity Voltage:"));
  }
  Serialprint(bms.EepromData.cap0, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Cells self discharge rate:"));
  }
  Serialprint(bms.EepromData.dsg_rate);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Cycle count:"));
  }
  Serialprint(bms.EepromData.cyc_cnt);
  Serialprint(F(" "));

  if (description) {
    Serialprint(F("Charge Over Temperaure threshold:"));
  }
  Serialprint(bms.EepromData.chg_ot);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Charge Over Temperature Release threshold:"));
  }
  Serialprint(bms.EepromData.chg_ot_rel);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Charge Under Temperature threshold:"));
  }
  Serialprint(bms.EepromData.chg_ut);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Charge Under Temperature Release threshold:"));
  }
  Serialprint(bms.EepromData.chg_ut);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Discharge Over Temperaure threshold:"));
  }
  Serialprint(bms.EepromData.dsg_ot);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Discharge Over Temperature Release threshold:"));
  }
  Serialprint(bms.EepromData.dsg_ot_rel);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Discharge Under Temperature threshold:"));
  }
  Serialprint(bms.EepromData.dsg_ut);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Discharge Under Temperature Release threshold:"));
  }
  Serialprint(bms.EepromData.dsg_ut);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Charge Under Temperature release delay:"));
  }
  Serialprint(bms.EepromData.chg_ut_delay);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Charge Over Temperature release delay:"));
  }
  Serialprint(bms.EepromData.chg_ot_delay);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Discharge Under Temperature release delay:"));
  }
  Serialprint(bms.EepromData.dsg_ut_delay);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Discharge Over Temperature release delay:"));
  }
  Serialprint(bms.EepromData.dsg_ot_delay);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Pack Over Voltage Protection threshold:"));
  }
  Serialprint(bms.EepromData.povp, 3);
  Serialprint(F("V "));

  if (description) {
    Serialprint(F("Pack Over Voltage Protection release threshold:"));
  }
  Serialprint(bms.EepromData.povp_rel, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Pack Under Voltage Protection threshold:"));
  }
  Serialprint(bms.EepromData.puvp, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Pack Under Voltage Protection threshold:"));
  }
  Serialprint(bms.EepromData.puvp_rel, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Cell Over Voltage Protection threshold:"));
  }
  Serialprint(bms.EepromData.covp, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Cell Over Voltage Protection release threshold:"));
  }
  Serialprint(bms.EepromData.covp_rel, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Cell Under Voltage Protection threshold:"));
  }
  Serialprint(bms.EepromData.cuvp, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Cell Under Voltage Protection threshold:"));
  }
  Serialprint(bms.EepromData.cuvp_rel, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Pack Under Voltage release delay:"));
  }
  Serialprint(bms.EepromData.puvp_delay);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Pack Over Voltage release delay:"));
  }
  Serialprint(bms.EepromData.povp_delay);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Cell Under Voltage release delay:"));
  }
  Serialprint(bms.EepromData.cuvp_delay);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("Cell Over Voltage release delay:"));
  }
  Serialprint(bms.EepromData.covp_delay);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Charge Over Current Protection threshold:"));
  }
  Serialprint(bms.EepromData.cocp, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Discharge Over Current Protection threshold:"));
  }
  Serialprint(bms.EepromData.docp, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Charge Over Current delay:"));
  }
  Serialprint(bms.EepromData.cocp_delay);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Charge Over Current release delay:"));
  }
  Serialprint(bms.EepromData.cocp_rel_delay);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Discharge Over Current delay:"));
  }
  Serialprint(bms.EepromData.docp_delay);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Discharge Over Current release delay:"));
  }
  Serialprint(bms.EepromData.docp_rel_delay);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Balance start voltage:"));
  }
  Serialprint(bms.EepromData.bal_start, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Balance Window:"));
  }
  Serialprint(bms.EepromData.bal_window, 3);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Shunt Res:"));
  }
  Serialprint(bms.EepromData.shunt_res);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("SWITCH_EN:"));
  }
  Serialprint(bms.EepromData.switch_en);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("SCRL_EN:"));
  }
  Serialprint(bms.EepromData.scrl_en);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("BAL_EN:"));
  }
  Serialprint(bms.EepromData.bal_en);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("CH_BAL_EN:"));
  }
  Serialprint(bms.EepromData.ch_bal_en);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("LED_EN:"));
  }
  Serialprint(bms.EepromData.led_en);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("LED_N_EN:"));
  }
  Serialprint(bms.EepromData.led_num_en);


  for (int i = 0; i < 8; i++) {
    if (description) {
      Serialprint(F("NTC"));
      Serialprint(i + 1);
      Serialprint(F(":"));
    }
    Serialprint(bms.EepromData.ntc[i]);
    Serialprint(F(","));
  }

  if (description) {
    Serialprint(F("Cell count:"));
  }
  Serialprint(bms.EepromData.cell_cnt);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Secondary Cell Over Voltage Protection threshold:"));
  }
  Serialprint(bms.EepromData.covp_high);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Secondary Cell Under Voltage Protection threshold:"));
  }
  Serialprint(bms.EepromData.cuvp_high);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("val_x2_en:"));
  }
  Serialprint(bms.EepromData.sc_dsgoc_x2);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("sc delay:"));
  }
  switch (bms.EepromData.sc_delay) {
    case 0:
      Serialprint(F("70us "));
      break;
    case 1:
      Serialprint(F("100us "));
      break;
    case 2:
      Serialprint(F("200us "));
      break;
    case 3:
      Serialprint(F("400us "));
      break;
  }
  Serialprint(F(","));
  if (description) {
    Serialprint(F("sc:"));
  }
  switch (bms.EepromData.sc) {
    case 0:
      Serialprint(F("22mV "));
      break;
    case 1:
      Serialprint(F("33mV "));
      break;
    case 2:
      Serialprint(F("44mV "));
      break;
    case 3:
      Serialprint(F("56mV "));
      break;
    case 4:
      Serialprint(F("67mV "));
      break;
    case 5:
      Serialprint(F("78mV "));
      break;
    case 6:
      Serialprint(F("89mV "));
      break;
    case 7:
      Serialprint(F("100mV "));
      break;
  }
  Serialprint(F(","));
  if (description) {
    Serialprint(F("dsgoc2_delay:"));
  }
  switch (bms.EepromData.dsgoc2_delay) {
    case 0:
      Serialprint(F("8ms "));
      break;
    case 1:
      Serialprint(F("20ms "));
      break;
    case 2:
      Serialprint(F("40ms "));
      break;
    case 3:
      Serialprint(F("80ms "));
      break;
    case 4:
      Serialprint(F("160ms "));
      break;
    case 5:
      Serialprint(F("320ms "));
      break;
    case 6:
      Serialprint(F("640ms "));
      break;
    case 7:
      Serialprint(F("1280ms "));
      break;
  }
  Serialprint(F(","));
  if (description) {
    Serialprint(F("dsgoc2:"));
  }
  switch (bms.EepromData.dsgoc2) {
    case 0:
      Serialprint(F("8mV "));
      break;
    case 1:
      Serialprint(F("11mV "));
      break;
    case 2:
      Serialprint(F("14mV "));
      break;
    case 3:
      Serialprint(F("17mV "));
      break;
    case 4:
      Serialprint(F("19mV "));
      break;
    case 5:
      Serialprint(F("22mV "));
      break;
    case 6:
      Serialprint(F("25mV "));
      break;
    case 7:
      Serialprint(F("28mV "));
      break;
    case 8:
      Serialprint(F("31mV "));
      break;
    case 9:
      Serialprint(F("33mV "));
      break;
    case 10:
      Serialprint(F("36mV "));
      break;
    case 11:
      Serialprint(F("39mV "));
      break;
    case 12:
      Serialprint(F("42mV "));
      break;
    case 13:
      Serialprint(F("44mV "));
      break;
    case 14:
      Serialprint(F("47mV "));
      break;
    case 15:
      Serialprint(F("50mV "));
      break;
  }
  Serialprint(F(","));

  if (description) {
    Serialprint(F("sc_rel_time:"));
  }
  Serialprint(bms.EepromData.sc_rel_time);
  Serialprint(F(","));
  if (description) {
    Serialprint(F("cuvp_high_delay:"));
  }
  switch (bms.EepromData.cuvp_high_delay) {
    case 0:
      Serialprint(F("1s "));
      break;
    case 1:
      Serialprint(F("4s "));
      break;
    case 2:
      Serialprint(F("8s "));
      break;
    case 3:
      Serialprint(F("16s "));
      break;
  }
  Serialprint(F(","));
  if (description) {
    Serialprint(F("covp_high_delay:"));
  }
  switch (bms.EepromData.covp_high_delay) {
    case 0:
      Serialprint(F("1s "));
      break;
    case 1:
      Serialprint(F("2s "));
      break;
    case 2:
      Serialprint(F("4s "));
      break;
    case 3:
      Serialprint(F("8s "));
      break;
  }
  if (head) {
    Serialprintln();
  }
}
