unsigned long time_echo_request = 0;
unsigned long timer_echo_request = 65;
#define debmultiserial false

uint32_t time_multiserial = 0;

// TOFIX: adapter superfluo (chiamata unica a 200ms)
void computeMux() {
  muxRead(200);
}
uint8_t maxMux_index = 0;
void resetTime_multiserial() {
  time_multiserial = millis();
}
uint8_t getIndexToID(uint8_t ID) {
  uint8_t index = 0;  // TOFIX: non era inizializzato, potenziale return NAN. conviene inserire un -1 per segnalare la casistica in cui il confronto fallisca sempre?
  for (uint8_t j = 0; j < 16; j++) {
    if (ID == id_mux[j]) {
      index = j;
      break;
    }
  }
  return index;
}

// TOFIX: parametro in ingresso superfluo. è hardcoded a 200ms

void muxRead(uint16_t timer_multiserial) {
  if ((millis() - time_multiserial > timer_multiserial) || (SerialRW6.multiserial_ok)) {
    time_multiserial = millis();
    if (debmultiserial) {
      Serial.print(mux_index);
      // TOFIX: ISO C++ forbids converting a string constant to 'char*'
      if (SerialRW6.multiserial_ok) {
        Serial.println(" TOPPERIA");
      } else {
        Serial.println(" NO_TOPPERIA");
      }
    }
    // TOFIX: meglio un define essendo una caratteristica hw oppure spostare ciclo nell'init
    for (uint8_t j = 0; j < 16; j++) {
      if (mux_read_en[j] != 0) {
        maxMux_index = j;
      }
    }

    if (mux_index > maxMux_index) {
      mux_index = 0;
    }

    Serial6.flush();
    Serial6.begin(mux_ch_baud[mux_index]);

    // TOFIX: sizeof(id_mux) è hardcoded a 16. vedi definizione. inoltre significa che lock_index va in overflow di 1 essendo (0-15)
    // TOFIX: probabilmente è superfluo il caso principale. Dovrebbe bastare setMuxChannel(mux_index);
    // Notare inoltre che lock_index è immutabile?!

    if (lock_index != sizeof(id_mux)) {
      Serialprint(F("Canale bloccato!!!!"));
      Serialprintln();
      setMuxChannel(lock_index);
    } else {
      setMuxChannel(mux_index);
    }
    sendMessMux(id_mux[mux_index]);  //COLLO DI BOTTIGLIA
    mux_index++;
  }
}
uint32_t ledFreq = 0;
uint32_t usFreq = 0;
uint32_t radioFreq = 0;

uint32_t sendMuxTime;
void sendMessMux(uint8_t id_Disp) {
  sendMuxTime = millis();
  switch (id_Disp) {
    case ID_RADIOCOMANDO:
      if (power_en[PWR_RADIO]) {
        sendCostructBuff(ID_PORTA_6, myID, ID_RADIOCOMANDO, ID_RADIOCOMANDO, REQUEST_CMD1, 0, 0);
        //Serial.print("RADIO: ");
        //Serial.println(millis()-radioFreq);
        //radioFreq = millis();
      }
      break;
    case ID_ECHO:
      if (power_en[PWR_US_LIDAR]) {
        sendCostructBuff(ID_PORTA_6, myID, ID_ECHO, ID_ECHO, REQUEST_CMD1, ECHO_NANO_GET_CMD2, 0);
        //Serial.print("US: ");
        //Serial.println(millis()-usFreq);
        //usFreq = millis();
      }
      break;
    case ID_LED:
      if (power_en[PWR_LED]) {
        sendCostructBuff(ID_PORTA_6, myID, ID_LED, 255, 255, 255, 255);  //TOFIX: temp dummy data to move the ring light position and update the white override state
        //Serial6.println("LED");
        //Serial.println(millis()-ledFreq);
        //ledFreq = millis();
        //delay(10);
      }
      break;
  }
  sendMuxTime = millis() - sendMuxTime;
}

uint8_t compute_next_ind(uint8_t mux_index, uint8_t lock_index) {
  if (lock_index != 16) {
    mux_index = lock_index;
  } else {
    mux_index++;
  }
  return mux_index;
}

uint8_t g_channel_truth_table[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

void setMuxChannel(uint8_t muxChannel) {
  for (uint8_t i = 0; i < sizeof(muxAddr); i++) {
    digitalWrite(muxAddr[i], (g_channel_truth_table[muxChannel] >> i) & (0b00000001));
  }
}
//**************************** ECHO ***************************

void computeEcho() {
  if (millis() - time_echo_request > timer_echo_request) {
    sendCostructBuff(ID_PORTA_6, myID, ID_ECHO, ID_ECHO, REQUEST_CMD1, ECHO_NANO_GET_CMD2, 0);
  }
}

void printEcho(uint8_t description, uint8_t head) {
  for (uint8_t i = 0; i < 8; i++) {
    if (description) {
      Serialprint(F("S"));
      Serialprint(i + 1);
      Serialprint(F(":"));
    }
    Serialprint(echoBuffer[i]);
    Serialprint(F(","));
  }
  if (head) {
    Serialprintln();
  }
}

//**************************** MPPT ***************************

void sendMpptParam(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, float value) {
  for (uint8_t i = 0; i < sizeof(id_mux); i++) {
    if (id_mux[i] == ID_MPPT) {
      setMuxChannel(i);
    }
  }
  costructBuff(&value, sizeof(value));
  sendCostructBuff(ID_PORTA_6, myID, ID_MPPT, ID_MPPT, cmd1, cmd2, cmd3);
}

void sendMuxMpptRequest(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3) {
  sendCostructBuff(ID_PORTA_6, myID, ID_MPPT, ID_MPPT, cmd1, cmd2, cmd3);
}
