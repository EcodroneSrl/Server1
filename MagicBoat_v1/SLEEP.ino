void sleepInit() {
  // Configure pin 6 for bounce library
  pinMode(BUTTON, INPUT_PULLUP);
  // Serialprintln(F("start..."));
  // delay(20);
  //pin, mode, type
  digital.pinMode(BUTTON, INPUT_PULLUP, FALLING);

  readSleepFlash();

  goSleepMode();
}

void goSleepMode() {
  if (sleep_on == 1) {
    // debug led
    pinMode(LED_BUILTIN, OUTPUT);
    //usb.println("entro in sleep mode");
    button.update();

    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(5);
    //**************************
    allPinLow();  //Funzione che spenge tutti i dispositivi e la comunicazione

    Snooze.sleep(config_teensy);

    //SerialRWSocket.beginSocket(ServerSocket,ID_PORTA_SOCK,analBuff,id.boat[0],id.boat[1],id.boat[2]);

    //digitalWrite(LED_BUILTIN, HIGH);

    checkButtonSleep();

    doReboot();
  }
}

void allPinLow() {
  imu.setPinsLow();
  setDsgBms(ID_PRUA, DSG_OFF);
  delay(100);
  bms.setMOSFET(CHG_ON, DSG_OFF);  // Abilito/disabilito carica/scarica
  delay(10);
  //pinMode(pinEnCan,OUTPUT);
  //digitalWrite(pinEnCan,HIGH);//va in sleeep alto
}

void setDsgBms(uint8_t id_mod, uint8_t bms_on_off) {
  uint8_t cmd1 = REQUEST_CMD1;
  uint8_t cmd2 = EN_MODULE_BMS_CMD2;
  uint8_t cmd3 = bms_on_off;

  sendCostructBuffB(ID_PORTA_8, myID, id_mod, id_mod, cmd1, cmd2, cmd3, false);
}

void doReboot() {
  SCB_AIRCR = 0x05FA0004;
}

void checkButtonSleep() {
  elapsedMillis timeout = 0;
  while (timeout < 6) button.update();
  bool awake = timerButtonHold();
  if (!awake) {
    sleep_on = 1;
    writeSleepFlash(&sleep_on);
    for (uint8_t i = 0; i < 20; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
  } else {
    sleep_on = 0;
    writeSleepFlash(&sleep_on);
  }
}

bool timerButtonHold() {
  // this is the 3 sec button press check
  while (button.duration() < 1000) {

    // get the current pin state, must have this!
    button.update();

    // Check button state, if the button
    // is not pressed for 3 seconds go
    // back to sleep.
    if (button.read() != 0) return false;
  }
  // button was held for 3 seconds so now we are awake
  return true;
}

void readSleepFlash() {
  sleepFile = extFlash.open("sleep.bin", FILE_READ);
  sleep_on = sleepFile.read();
  sleepFile.close();
}

void writeSleepFlash(uint8_t *sleep) {
  extFlash.remove("sleep.bin");
  sleepFile = extFlash.open("sleep.bin", FILE_WRITE);
  sleepFile.write(sleep, sizeof(uint8_t));
  sleepFile.close();
}
