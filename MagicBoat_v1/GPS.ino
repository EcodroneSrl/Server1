void gpsInit() {
  gps.begin(115200);
  gps.ucenter_OFF();        //Abilita/disabilita la comunicazione con ucenter
  gps.printClearGPS_OFF();  //Abilita/disabilita la stampa dei caratteri GPS grezzi
}

//Acquisice il messaggio GPS e fa i parsing delle variabili dei msg NAV-PVT e NAV-RELPOSNED
uint32_t gpsTime = 0;
void computeGPS() {
  gpsTime = millis();
  uint8_t gps_return = gps.Read();

  if (gps_return == PVT_OK) {
    Vel_GPS = ((float(gps.pvt1.gSpeed)) * 3600.0) / 1000000.0;
    TetaB_VelGPS = (float(gps.pvt1.headMot)) / 100000.0;
    TetaB_VelGPS = get180(TetaB_VelGPS);
    errTetaB_Vel = get180(TetaB_VelGPS - TetaB_IMU);

    sendJetson(PVT_OK);
  }

  if (gps_return == RELPOSNED_OK) {
    TetaB_DiffGPS = float(gps.relposned1.relPosHeading) / 100000.0;
    TetaB_DiffGPS = getGPSDiffTetaB(TetaB_DiffGPS, true);
    errTetaB_Diff = get180(TetaB_DiffGPS - TetaB_IMU);

    sendJetson(RELPOSNED_OK);
  }
  gpsTime = millis() - gpsTime;
}

float getGPSDiffTetaB(float TetaB_GPS, uint8_t sign) {
  float tetaTemp;
  tetaTemp = get180(TetaB_GPS);
  if (sign) {
    tetaTemp += 90;
  } else {
    tetaTemp -= 90;
  }
  tetaTemp = get180(tetaTemp);

  return tetaTemp;
}

/* ------------ FUNCTIONS TO PRINT GPS DEBUG --------------- */

void printParsePVT(uint8_t head) {
  Serialprint(F("NAV-PVT: "));
  Serialprint(gps.pvt1.iTOW);
  Serialprint(F(","));
  Serialprint(gps.pvt1.year);
  Serialprint(F(","));
  Serialprint(gps.pvt1.month);
  Serialprint(F(","));
  Serialprint(gps.pvt1.day);
  Serialprint(F(","));
  Serialprint(gps.pvt1.hour);
  Serialprint(F(","));
  Serialprint(gps.pvt1.min);
  Serialprint(F(","));
  Serialprint(gps.pvt1.sec);
  Serialprint(F(","));
  Serialprint(gps.pvt1.tAcc);
  Serialprint(F(","));
  Serialprint(gps.pvt1.nano);
  Serialprint(F(","));
  Serialprint(gps.pvt1.fixType);
  Serialprint(F(","));
  Serialprint(gps.pvt1.numSV);
  Serialprint(F(","));
  Serialprint(gps.pvt1.lon);
  Serialprint(F(","));
  Serialprint(gps.pvt1.lat);
  Serialprint(F(","));
  Serialprint(gps.pvt1.height);
  Serialprint(F(","));
  Serialprint(gps.pvt1.hMSL);
  Serialprint(F(","));
  Serialprint(gps.pvt1.hAcc);
  Serialprint(F(","));
  Serialprint(gps.pvt1.vAcc);
  Serialprint(F(","));
  Serialprint(gps.pvt1.velN);
  Serialprint(F(","));
  Serialprint(gps.pvt1.velE);
  Serialprint(F(","));
  Serialprint(gps.pvt1.velD);
  Serialprint(F(","));
  Serialprint(gps.pvt1.gSpeed);
  Serialprint(F(","));
  Serialprint(gps.pvt1.headMot);
  Serialprint(F(","));
  Serialprint(gps.pvt1.sAcc);
  Serialprint(F(","));
  Serialprint(gps.pvt1.headAcc);
  Serialprint(F(","));
  Serialprint(gps.pvt1.pDOP);
  Serialprint(F(","));
  Serialprint(gps.pvt1.invalidL1h);
  Serialprint(F(","));
  Serialprint(gps.pvt1.headVeh);
  Serialprint(F(","));
  Serialprint(gps.pvt1.magDec);
  Serialprint(F(","));
  Serialprint(gps.pvt1.magAcc);
  Serialprint(F(","));
  if (head) {
    Serialprintln();
  }
}

void printParseRELPOSNED(uint8_t head) {
  Serialprint(F("NAV-RELPOSNED: "));
  Serialprint(gps.relposned1.version);
  Serialprint(F(","));
  Serialprint(gps.relposned1.refStationId);
  Serialprint(F(","));
  Serialprint(gps.relposned1.iTOW);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosN);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosE);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosD);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosLength);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosHeading);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosHPN);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosHPE);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosHPD);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosHPLength);
  Serialprint(F(","));
  Serialprint(gps.relposned1.accN);
  Serialprint(F(","));
  Serialprint(gps.relposned1.accE);
  Serialprint(F(","));
  Serialprint(gps.relposned1.accD);
  Serialprint(F(","));
  Serialprint(gps.relposned1.accLength);
  Serialprint(F(","));
  Serialprint(gps.relposned1.accHeading);
  Serialprint(F(","));
  Serialprint(gps.relposned1.gnssFixOK);
  Serialprint(F(","));
  Serialprint(gps.relposned1.diffSoln);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosValid);
  Serialprint(F(","));
  Serialprint(gps.relposned1.carrSoln);
  Serialprint(F(","));
  Serialprint(gps.relposned1.isMoving);
  Serialprint(F(","));
  Serialprint(gps.relposned1.refPosMiss);
  Serialprint(F(","));
  Serialprint(gps.relposned1.refObsMiss);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosHeadingValid);
  Serialprint(F(","));
  Serialprint(gps.relposned1.relPosNormalized);
  Serialprint(F(","));
  if (head) {
    Serialprintln();
  }
}


void printHead() {
  Serialprintln(F(","));
}
