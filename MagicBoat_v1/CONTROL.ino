unsigned long dbgTest = 0;

uint32_t boatTime = 0;

uint32_t timerJs[4] = {5000,10000,10000,10000};
uint32_t timeJs[4] = {0,0,0,0};

void resetTimerJs(uint8_t indexStruct)
{
    timeJs[indexStruct] = millis();
}
void jsTimerNoSignal()
{
 for (uint8_t i = 0; i < 4; i++) 
 {
  if(millis()-timeJs[i]>timerJs[i])
  {
    JsCmdData[i].throttle = 0;
    JsCmdData[i].axisY= 0;
    JsCmdData[i].axisX= 0;
    JsCmdData[i].wheel= 0;
    timeJs[i]= millis();
  }
 }
}


void boatControl() {

  boatTime = millis();
  // Decido da quale dei dispositivi prendere la fix gps
  boatLat = getMyLat(gpsSource);
  boatLon = getMyLon(gpsSource);

  if (mission_active) {
    checkTargetDist();  // Scala i waypoint della missione attuale
  }

  if (millis() - time_tetaD_AC < timer_tetaD_AC) 
  {
    corrTetaD_AC = 0;
  }
  jsTimerNoSignal();//
  switch (telInputDev) {
    
    case INPUT_JOYSTICK_CMD3:
      controlCmdData.base_cmd = -JsCmdData[JS_BOAT_CMD3].throttle;
      controlCmdData.boost_cmd = -JsCmdData[JS_BOAT_CMD3].axisY;
      controlCmdData.diff_cmd = JsCmdData[JS_BOAT_CMD3].axisX;
      controlCmdData.wheel_cmd = JsCmdData[JS_BOAT_CMD3].wheel;
    
      break;

    case INPUT_RADIO_CMD3:
      controlCmdData.base_cmd = radioChValue[CH_SX_ROT];
      controlCmdData.boost_cmd = radioChValue[CH_DX_Y];
      controlCmdData.diff_cmd = radioChValue[CH_SX_X];
      controlCmdData.wheel_cmd = radioChValue[CH_DX_X];
      break;
  }
  //debugJoyControl();
  // DEBUG ZONE #########
  /*
  if (millis() - dbgTest > 250) {
    Serial.printf("THR: %2.2f - aY: %2.2f - aX: %2.2f - WHE: %2.2f\n", controlCmdData.base_cmd, controlCmdData.boost_cmd, controlCmdData.diff_cmd, controlCmdData.wheel_cmd);
    dbgTest = millis();
  }
  */
  // ####################

  

  switch (boatNavMode) {
    case NAV_TEL:
      switch (boatNavSubMode) {
        case TEL_MODE_1:
          // erpm_base & +DD,+CD,-SS,-CS
          break;

        case TEL_MODE_2:
          // erpm_base & ThetaD
          TetaD = getTetaDMemCorr(TetaD) + getTetaDTempCorr() + corrTetaDBut + corrTetaD_AC;
          TetaD = get180(TetaD);
          setFRAMTetaD();
          TetaB = getThetaB(thetaBSource, TetaD);
          applyControlRoute(controlModeRoute);
          controlCmdData.diff_cmd = corrRoute;
          break;

        case TEL_MODE_3:
          // velocità mantenuta da gps
          VelD = getVelDTel(controlCmdData.base_cmd);
          applyControlVel(controlModeVel);
          controlCmdData.base_cmd = corrVel;
          break;

        case TEL_MODE_4:
          // velocità mantenuta da gps & ThetaD
          TetaD = getTetaDMemCorr(TetaD) + getTetaDTempCorr() + corrTetaDBut + corrTetaD_AC;
          TetaD = get180(TetaD);
          setFRAMTetaD();
          TetaB = getThetaB(thetaBSource, TetaD);
          applyControlRoute(controlModeRoute);
          controlCmdData.diff_cmd = corrRoute;

          VelD = getVelDTel(controlCmdData.base_cmd);
          applyControlVel(controlModeVel);
          controlCmdData.base_cmd = corrVel;
          break;
      }
      break;

    case NAV_AUT:
      switch (boatNavSubMode) {
        case AUT_MODE_1:
          // erpm base + thetaD da waypoint
          TetaD = getThetaDMission(arriveModeNow) + corrTetaDBut + corrTetaD_AC;
          TetaD = get180(TetaD);
          setFRAMTetaD();
          TetaB = getThetaB(thetaBSource, TetaD);
          applyControlRoute(controlModeRoute);
          controlCmdData.diff_cmd = corrRoute;

          controlCmdData.base_cmd = getRifNavMode();
          break;
        case AUT_MODE_2:
          // velocità mantenuta da GPS & ThetaD da waypoint
          TetaD = getThetaDMission(arriveModeNow) + corrTetaDBut + corrTetaD_AC;
          TetaD = get180(TetaD);
          setFRAMTetaD();
          TetaB = getThetaB(thetaBSource, TetaD);
          applyControlRoute(controlModeRoute);
          controlCmdData.diff_cmd = corrRoute;

          VelD = getRifNavMode();
          applyControlVel(controlModeVel);
          controlCmdData.base_cmd = corrVel;
          break;

        case AUT_MODE_3:
          // erpm da compensazione energia + ThetaD da waypoint
          TetaD = getThetaDMission(arriveModeNow) + corrTetaDBut + corrTetaD_AC;
          TetaD = get180(TetaD);
          setFRAMTetaD();
          TetaB = getThetaB(thetaBSource, TetaD);
          applyControlRoute(controlModeRoute);
          controlCmdData.diff_cmd = corrRoute;
          break;

        case AUT_MODE_4:
          // punto-raggio
          break;

        case AUT_MODE_5:
          // interno ad area con waypoint ricevuti da jetson
          rifLonTrue = rifLonJetson;
          rifLatTrue = rifLatJetson;

          TetaD = getThetaDMission(arriveModeNow) + corrTetaDBut + corrTetaD_AC;
          TetaD = get180(TetaD);
          setFRAMTetaD();
          TetaB = getThetaB(thetaBSource, TetaD);
          applyControlRoute(controlModeRoute);
          controlCmdData.diff_cmd = corrRoute;
          break;
      }
      break;
  }

  // TetaD = getThetaDMission(arriveModeNow);

  // float diffTetaD = applyControlTheta(tetaD,tetaB,controlMode);
  // float rifVelErpmBase = applyControlVelBase(controlMode);

  miscApplyParam(controlCmdData.base_cmd, controlCmdData.boost_cmd, controlCmdData.diff_cmd, controlCmdData.wheel_cmd, boatNavMode, boatNavSubMode);

  motorDrive();  // Applico i limiti e setto gli ERPM o corrente dalla struttura
  boatTime = millis() - boatTime;
}

float getTetaDTempCorr() {
  float corrTetaD = controlCmdData.wheel_cmd * corrTetaDLim;
  return corrTetaD;
}

float getTetaDMemCorr(float memCorrTetaD) {
  if (millis() - time_tetaD < timer_tetaD) {
    time_tetaD = millis();
    memCorrTetaD += kpTetaD * controlCmdData.diff_cmd;
  }
  memCorrTetaD = get180(memCorrTetaD);

  return memCorrTetaD;
}

void applyControlRoute(uint8_t mode) {
  switch (mode) {
    case CONTROL_MODE_PID:
      pidRoute.ComputeMB();
      break;

    case CONTROL_MODE_ADAPTIVE:
      // diffTeta = computeAdaptive();
      break;

    case CONTROL_MODE_ROBUST:
      // diffTeta = computeRobust();
      break;
  }
}

void applyControlVel(uint8_t mode) {
  switch (mode) {
    case CONTROL_MODE_PID:
      pidVelErpm.ComputeMB();
      break;

    case CONTROL_MODE_ADAPTIVE:
      // diffTeta = computeAdaptive();
      break;

    case CONTROL_MODE_ROBUST:
      // diffTeta = computeRobust();
      break;
  }
}

float getThetaDMission(uint8_t mode) {
  double DifLon;
  double DifLat;

  if ((mode == ARRIVE_M_SIMPLE) || (mode == ARRIVE_M_HAVERSINE)) {
    DifLon = boatLon - rifLonTrue;
    DifLat = boatLat - rifLatTrue;
  }
  if (mode == ARRIVE_M_NO_MIDPOINTS) {
    DifLon = boatLon - rifLonMission;
    DifLat = boatLat - rifLatMission;
  }

  float tetaDrad = (atan2((DifLon), DifLat));
  float tetaD = tetaDrad * RAD_TO_DEG;  // Passaggio da radianti a gradi
  tetaD = get180(tetaD);

  return tetaD;
}

float getThetaB(uint8_t mode, float tetaD)  // mode: 1) yaw IMU; 2) heading RTK 3) heading NAV-PVT
{
  float tetaB = 0;
  float tetaE = tetaD - tetaB;

  switch (mode) {
    case THETAB_IMU:
      tetaB = TetaB_IMU;
      break;

    case THETAB_DIFF:
      tetaB = TetaB_DiffGPS;
      break;

    case THETAB_VEL:
      tetaB = TetaB_VelGPS;
      break;

    case THETAB_IMU_DIFF:
      tetaB = TetaB_IMU + errTetaB_Diff;
      tetaB = get180(tetaB);
      break;

    case THETAB_IMU_VEL:
      tetaB = TetaB_IMU + errTetaB_Vel;
      tetaB = get180(tetaB);
      break;
  }

  float Corr = 0;

  if ((tetaB < 0) && (tetaD > 0) && (tetaE >= 180)) {
    Corr = 360;  // do in ingresso un valore maggiore di 180° oppure minore di -180° in modo che calcoli il pid
  }
  if ((tetaB > 0) && (tetaD < 0) && (tetaE <= -180)) {
    Corr = -360;
  }
  tetaB += Corr;

  return tetaB;
}

float getVelDTel(float velRif) {
  float vel_D;

  vel_D = ((1 - velRif) * (VelDMax / 2));

  return vel_D;
}

float getRifNavMode() {
  float rifNavMode = 0;

  if (wayPointData.NavMode <= NAV_LIM_1) {
    rifNavMode = getNavModeErpm();
  }
  if ((wayPointData.NavMode > NAV_LIM_1) && (wayPointData.NavMode <= NAV_LIM_2)) {
    rifNavMode = getNavModeVel();
  }
  if ((wayPointData.NavMode > NAV_LIM_2) && (wayPointData.NavMode <= NAV_LIM_3)) {
    rifNavMode = getNavModeComp();
  }

  return rifNavMode;
}

float getNavModeErpm() {
  float erpm_rif;
  uint8_t range = NAV_LIM_1;

  erpm_rif = (Kerpm * wayPointData.NavMode) / range;
  return erpm_rif;
}

float getNavModeVel() {
  float vel_rif;
  uint8_t range = NAV_LIM_2 - NAV_LIM_1;

  vel_rif = (Kvel * wayPointData.NavMode) / range;
  return vel_rif;
}

float getNavModeComp() {
  float comp_rif;
  uint8_t range = NAV_LIM_3 - NAV_LIM_2;
  comp_rif = (Kerpm * wayPointData.NavMode) / range;
  return comp_rif;
}

float get180(float angle) {
  // Funzione che converte tra +-180

  if (angle < (-180)) {
    angle = angle + 360;
  }
  if (angle >= (180)) {
    angle = angle - 360;
  }
  return angle;
}

int32_t getMyLat(uint8_t disp) {
  int32_t myLat = 0;
  switch (disp) {
    case GPS_UBX:
      myLat = gps.pvt1.lat;
      break;

    case GPS_AIS:
      // Lo faremo
      break;

    case GPS_RUT:
      // Lo Faremo
      break;
  }
  return myLat;
}

int32_t getMyLon(uint8_t disp) {
  int32_t myLon = 0;
  switch (disp) {
    case GPS_UBX:
      myLon = gps.pvt1.lon;
      break;

    case GPS_AIS:
      // Lo faremo
      break;

    case GPS_RUT:
      // Lo Faremo
      break;
  }
  return myLon;
}

uint32_t buttonTime = 0;
void computeButtons() {
  buttonTime = millis();
  for (uint8_t i = 0; i < sizeof(JsCmdData[JS_BOAT_CMD3].buttons); i++) {
    if (JsCmdData[JS_BOAT_CMD3].buttons[i]) {
      switch (i) {
        case 0:
          clacson();
          break;

        case 1:

          break;

        case 2:

          break;

        case 3:

          break;

        case 4:
          corrTetaDBut = buttonTetaD(-10.0);
          break;

        case 5:
          corrTetaDBut = buttonTetaD(-5.0);
          break;

        case 6:
          corrTetaDBut = buttonTetaD(-1.0);
          break;

        case 7:

          break;

        case 8:

          break;

        case 9:

          break;

        case 10:
          corrTetaDBut = buttonTetaD(10.0);
          break;

        case 11:
          corrTetaDBut = buttonTetaD(5.0);
          break;

        case 12:
          corrTetaDBut = buttonTetaD(1.0);
          break;

        case 13:

          break;

        case 14:

          break;

        case 15:

          break;

        case 16:

          break;
      }
    }
  }

  if (JsCmdData[JS_BOAT_CMD3].POV == 0xFFFF) {
    enNewPOV = 1;
  }

  if (enNewPOV) {
    switch (JsCmdData[JS_BOAT_CMD3].POV) {
      case 0:
        Serial.println("Luce accesa");
        enNewPOV = 0;
        break;

      case 90:
        Serial.println("Luce dx");
        enNewPOV = 0;
        break;

      case 180:
        Serial.println("Luce spenta");
        enNewPOV = 0;
        break;

      case 270:
        Serial.println("Luce sx");
        enNewPOV = 0;
        break;
    }
  }
  buttonTime = millis() - buttonTime;
}

float buttonTetaD(float value) {
  float corr = corrTetaDBut + value;
  return corr;
}

void clacson() {
  // TBD
  Serial.println("PA-PA-PA-PI-PO! PA-PA-PA-PI-PO!");
  delay(10);
}

void resetJSButtons() {
  for (uint8_t index_js = 0; index_js < sizeof(JsCmdData) / sizeof(JsCmdData[0]); index_js++) {
    memset(&JsCmdData[index_js].buttons[4], 0, sizeof(JsCmdData[index_js].buttons) - 4);
  }
}

void printJS(uint8_t description, uint8_t head, uint8_t index_js) {
  if (description) {
    Serialprint(F("X:"));
  }
  Serialprint(JsCmdData[index_js].axisX);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("Y:"));
  }
  Serialprint(JsCmdData[index_js].axisY);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("T:"));
  }
  Serialprint(JsCmdData[index_js].throttle);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("W:"));
  }
  Serialprint(JsCmdData[index_js].wheel);
  Serialprint(F(","));

  if (description) {
    Serialprint(F("P:"));
  }
  Serialprint(JsCmdData[index_js].POV);
  Serialprint(F(","));

  if (head) {
    Serialprintln();
  }
}