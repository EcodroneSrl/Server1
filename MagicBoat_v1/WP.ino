double computeDistAB(int32_t RifLat, int32_t RifLon, int32_t RifLatOld, int32_t RifLonOld) {
  double latMid, m_per_deg_lat, m_per_deg_lon, deltaLat, deltaLon, dist_m;
  double RifLat_d = ((double)RifLat) / 1000000.0;
  double RifLatOld_d = ((double)RifLatOld) / 1000000.0;
  double RifLon_d = ((double)RifLon) / 1000000.0;
  double RifLonOld_d = ((double)RifLonOld) / 1000000.0;

  latMid = (RifLat_d + RifLatOld_d) / 2.0;  // or just use Lat1 for slightly less accurate estimate

  m_per_deg_lat = 111132.954 - 559.822 * cos(2.0 * latMid) + 1.175 * cos(4.0 * latMid);
  m_per_deg_lon = (3.14159265359 / 180) * 6367449 * cos(latMid);

  deltaLat = fabs(RifLat_d - RifLatOld_d);
  deltaLon = fabs(RifLon_d - RifLonOld_d);

  dist_m = sqrt(pow(deltaLat * m_per_deg_lat, 2) + pow(deltaLon * m_per_deg_lon, 2));
  return dist_m / 1000.0;
}

double computeT(double distAB, double step) {
  double t;
  //t = (step/distAB);
  t = 0.000001;
  // Serialprint(t);
  // Serialprintln();
  return t;
}

double computeNewLat(int32_t RifLat, int32_t RifLon, int32_t RifLatOld, int32_t RifLonOld, double t, uint16_t index, double dist, uint8_t mode) {
  double RifLon_d = ((double)RifLon) / 1000000.0;
  double RifLonOld_d = ((double)RifLonOld) / 1000000.0;
  double RifLat_d = ((double)RifLat) / 1000000.0;
  double RifLatOld_d = ((double)RifLatOld) / 1000000.0;
  double newLatTrue = 0;

  if (mode == ARRIVE_M_NO_MIDPOINTS) {
    newLatTrue = rifLatMission;
  }

  if (mode == ARRIVE_M_SIMPLE) {
    newLatTrue = RifLat_d * t * index + RifLatOld_d * (1 - t * index);
  }

  if (mode == ARRIVE_M_HAVERSINE) {
    newLatTrue = haversineMidpointLat(RifLat_d, RifLon_d, RifLatOld_d, RifLonOld_d, dist, t, index);
  }

  return newLatTrue;
}

double computeNewLon(int32_t RifLat, int32_t RifLon, int32_t RifLatOld, int32_t RifLonOld, double t, uint16_t index, double dist, uint8_t mode) {
  double RifLon_d = ((double)RifLon) / 1000000.0;
  double RifLonOld_d = ((double)RifLonOld) / 1000000.0;
  double RifLat_d = ((double)RifLat) / 1000000.0;
  double RifLatOld_d = ((double)RifLatOld) / 1000000.0;
  double newLonTrue = 0;

  if (mode == ARRIVE_M_NO_MIDPOINTS) {
    newLonTrue = rifLonMission;
  }

  if (mode == ARRIVE_M_SIMPLE) {
    newLonTrue = RifLon_d * t * index + RifLonOld_d * (1 - t * index);
  }

  if (mode == ARRIVE_M_HAVERSINE) {
    newLonTrue = haversineMidpointLon(RifLat_d, RifLon_d, RifLatOld_d, RifLonOld_d, dist, t, index);
  }

  return newLonTrue;
}

void computeNewPoint(int32_t RifLat, int32_t RifLon, int32_t RifLatOld, int32_t RifLonOld, double step, uint16_t index, uint8_t mode) {
  double distAB = computeDistAB(RifLat, RifLon, RifLatOld, RifLonOld);
  double t = computeT(distAB, step);
  double distTot = haversine(RifLat, RifLon, RifLatOld, RifLonOld);
  double newLonTrue = computeNewLon(RifLat, RifLon, RifLatOld, RifLonOld, t, index, distTot, mode);
  double newLatTrue = computeNewLat(RifLat, RifLon, RifLatOld, RifLonOld, t, index, distTot, mode);
  rifLonTrue = double(newLonTrue * 1000000.0);
  rifLatTrue = double(newLatTrue * 1000000.0);
  setFRAMrifTrue();
  Serial.print(F("newMidpointLat:"));
  Serial.print(rifLatTrue);
  Serial.print(F(","));
  Serial.print(F("newMidpointLon:"));
  Serial.println(rifLonTrue);
}

double getBearing(double lat2, double lon2, double lat1, double lon1) {
  double y = sin(lon2 * DEG_TO_RAD - lon1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD);
  double x = cos(lat1 * DEG_TO_RAD) * sin(lat2 * DEG_TO_RAD) - sin(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * cos(lon2 * DEG_TO_RAD - lon1 * DEG_TO_RAD);
  double teta = atan2(y, x);
  double bearing = get180(teta * RAD_TO_DEG);
  return bearing;
}

double get180(double angle) {
  //Funzione che converte tra +-180

  if (angle < (-180)) {
    angle = angle + 360;
  }
  if (angle >= (180)) {
    angle = angle - 360;
  }
  return angle;
}

double haversineMidpointLat(double lat2, double lon2, double lat1, double lon1, double dist, double t, uint32_t index) {
  double delta = dist / earthR;
  t = t * 100000.0;
  uint32_t numPoints = dist / double(t);

  double f = index / (double(numPoints));
  // Serial.print(index);
  // Serial.print(",");
  // Serial.println(f,15);

  double a = sin((1 - f) * delta) / sin(delta);
  double b = sin(f * delta) / sin(delta);
  double x = a * cos(lat1 * DEG_TO_RAD) * cos(lon1 * DEG_TO_RAD) + b * cos(lat2 * DEG_TO_RAD) * cos(lon2 * DEG_TO_RAD);
  double y = a * cos(lat1 * DEG_TO_RAD) * sin(lon1 * DEG_TO_RAD) + b * cos(lat2 * DEG_TO_RAD) * sin(lon2 * DEG_TO_RAD);
  double z = a * sin(lat1 * DEG_TO_RAD) + b * sin(lat2 * DEG_TO_RAD);
  double latNow = RAD_TO_DEG * atan2(z, sqrt((x * x) + (y * y)));
  return latNow;
}

double haversineMidpointLon(double lat2, double lon2, double lat1, double lon1, double dist, double t, uint32_t index) {
  double delta = dist / earthR;
  t = t * 100000.0;
  uint32_t numPoints = dist / double(t);

  double f = index / (double(numPoints));

  double a = sin((1 - f) * delta) / sin(delta);
  double b = sin(f * delta) / sin(delta);
  double x = a * cos(lat1 * DEG_TO_RAD) * cos(lon1 * DEG_TO_RAD) + b * cos(lat2 * DEG_TO_RAD) * cos(lon2 * DEG_TO_RAD);
  double y = a * cos(lat1 * DEG_TO_RAD) * sin(lon1 * DEG_TO_RAD) + b * cos(lat2 * DEG_TO_RAD) * sin(lon2 * DEG_TO_RAD);
  double lonNow = RAD_TO_DEG * atan2(y, x);
  return lonNow;
}

double haversine(double lat2, double lon2, double lat1, double lon1) {
  double lat1_rad = ((double)lat1 / 1000000.0) * DEG_TO_RAD;
  double lat2_rad = ((double)lat2 / 1000000.0) * DEG_TO_RAD;
  double lon1_rad = ((double)lon1 / 1000000.0) * DEG_TO_RAD;
  double lon2_rad = ((double)lon2 / 1000000.0) * DEG_TO_RAD;
  double dLat_rad = lat2_rad - lat1_rad;
  double dLon_rad = lon2_rad - lon1_rad;

  double a = sin((dLat_rad) / 2) * sin(dLat_rad / 2) + cos(lat1_rad) * cos(lat2_rad) * sin(dLon_rad / 2) * sin(dLon_rad / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  double dist = earthR * c;  // in metres
  return dist;
}

void checkTargetDist() {
  uint32_t difAssLonTrue = abs(boatLon - rifLonTrue);
  uint32_t difAssLatTrue = abs(boatLat - rifLatTrue);
  if((boatLon != 0) && (boatLat != 0))
  {

    if ((difAssLonTrue <= RaggioGPSTrue) && (difAssLatTrue <= RaggioGPSTrue))  // Controllo su un quadrato
    {
      index_step++;
      setFRAMindexStep();  //Metto in FRAM
    //Serial.println("cagnaccio");

      computeNewPoint(rifLatMission, rifLonMission, rifLatMissionOld, rifLonMissionOld, interStep, index_step, arriveModeNow);
      // Serialprint(F("difAssLonTrue: "));
      // Serialprint(difAssLonTrue);
      // Serialprint(F(";difAssLatTrue: "));
      // Serialprint(difAssLatTrue);
      // Serialprintln();
      // delay(100);
    }

    uint32_t difAssLon = abs(boatLon - rifLonMission);
    uint32_t difAssLat = abs(boatLat - rifLatMission);

    checkSignal();

    if ((difAssLon <= RaggioGPS) && (difAssLat <= RaggioGPS)) {
      timer_change_wp = millis();

      if (!goBack) {
        navNext();
      } else {
        navBack();
      }
    }

    if (standActive) {
      if ((difAssLon >= RaggioStand) || (difAssLat >= RaggioStand)) {
        standActive = 0;
        Serial.println("Fuori stazionamento");
      }
    }
  }
}

void checkSignal() {
  if ((SIGNAL_STRENGTH_CHECK) && (SATELLITE_OFF) && (SENSOR_SIGNAL_OFF)) {
    if (millis() - time_signal_jetson > timer_signal_jetson) {
      if (!goBack) {
        goBack = 1;
        setFRAMgoBack();
        if (!standActive) {
          navBack();
        }
      }
    } else {
      if (restoreNavOk) {
        if (goBack) {
          goBack = 0;
          setFRAMgoBack();
        }
      }
    }
  }
}

void navNext() {
  if (indexWPNow == missionParamDataCurrent.wpEnd)  //wpEnd = waypoint finale del ciclo interno alla missione
  {
    if (missionCycleNum > 0) {
      missionCycle = 1;
      setFRAMmissionCycle();                         //metto in FRAM
      indexWPNow = missionParamDataCurrent.wpStart;  //wpStart = waypoint iniziale del ciclo interno alla missione
      setFRAMindexWPNow();

      missionCycleNum--;
      setFRAMmissionCycleNum();  //Metto in FRAM

      rifLonMissionOld = rifLonMission;
      rifLatMissionOld = rifLatMission;
      save_new_wp(&missionParamDataCurrent, indexWPNow);
      computeNewPoint(rifLatMission, rifLonMission, rifLatMissionOld, rifLonMissionOld, interStep, index_step, arriveModeNow);
    } else {
      if (!standActive)  // Questa ci vuole perchè senza, se devo stazionare nel raggio del WP, scrivo a loop su FRAM
      {
        missionCycle = 0;
        setFRAMmissionCycle();  //metto in FRAM
      }

      switch (missionParamDataCurrent.NMmode) {
        case NO_NEXT_MISSION:

          break;

        case TO_NEXT_MISSION:  //Carico i punti della missione successiva alla fine del ciclo interno
          updateCurrentMission(missionParamDataCurrent.NMnum, missionParamDataCurrent.idMissionNext);
          break;

        case TO_FINAL_WP_AND_NEXT:  //Proseguo con i waypoint fino ad arrivare all'ultimo e poi vado alla missione successiva
          toNextWP();
          break;

        case STAND_AT_FINAL_WP:
          toNextWP();
          break;

        case STAND_AT_FINAL_WP_CYCLE:
          standActive = 1;
          break;
      }
    }
  } else {
    if (indexWPNow < missionParamDataCurrent.total_mission_nWP - 1) {
      toNextWP();
    } else {
      switch (missionParamDataCurrent.NMmode) {
        case TO_FINAL_WP_AND_NEXT:  //Proseguo con i waypoint fino ad arrivare all'ultimo
          updateCurrentMission(missionParamDataCurrent.NMnum, missionParamDataCurrent.idMissionNext);
          break;

        case STAND_AT_FINAL_WP:
          standActive = 1;
          break;
      }
    }
  }
}

void navBack() {
  if (checkNumSuffix(wayPointData.ArriveMode, CHECK_SIGNAL_SUFF)) {
    rifLatMissionOld = boatLat;
    rifLonMissionOld = boatLon;

    if (!indexWPNow) {
      rifLatMissionOld = rifLatMission;
      rifLonMissionOld = rifLonMission;
    } else {
      setFRAMrifOld();
      if (indexWPNow == missionParamDataCurrent.wpStart) {
        if (missionCycle) {
          indexWPNow = missionParamDataCurrent.wpEnd;
          setFRAMindexWPNow();

          if (missionCycleNum < missionParamDataCurrent.cycles) {
            missionCycleNum++;
            setFRAMmissionCycleNum();  //Metto in FRAM

            if (missionCycleNum == missionParamDataCurrent.cycles) {
              missionCycle = 0;
              setFRAMmissionCycle();  //metto in FRAM
            }
          }
        } else {
          indexWPNow--;
          setFRAMindexWPNow();
        }
      }

      else if (indexWPNow == missionParamDataCurrent.wpEnd) {
        if (missionCycleNum < missionParamDataCurrent.cycles) {
          if (!standActive) {
            missionCycle = 1;
            setFRAMmissionCycle();  //metto in FRAM
          }
        }
        indexWPNow--;
        setFRAMindexWPNow();
      }

      else {
        indexWPNow--;
        setFRAMindexWPNow();
      }

      save_new_wp(&missionParamDataCurrent, indexWPNow);
      computeNewPoint(rifLatMission, rifLonMission, rifLatMissionOld, rifLonMissionOld, interStep, index_step, arriveModeNow);
    }
  }
}

void toNextWP() {
  standActive = 0;
  indexWPNow++;
  setFRAMindexWPNow();

  rifLonMissionOld = rifLonMission;
  rifLatMissionOld = rifLatMission;
  save_new_wp(&missionParamDataCurrent, indexWPNow);
  computeNewPoint(rifLatMission, rifLonMission, rifLatMissionOld, rifLonMissionOld, interStep, index_step, arriveModeNow);
  // Serialprint(F("difAssLon: "));
  // Serialprint(difAssLon);
  // Serialprint(F(";difAssLat: "));
  // Serialprint(difAssLat);
  // Serialprintln();
}

void save_new_wp(missionParam *missionParamPtr, uint16_t indexWP) {
  createFlashFilename(missionParamPtr);
  index_step = 1;
  setFRAMindexStep();  //Metto in FRAM

  readMissionWaypointData(&wayPointData, sizeof(wayPointData), flashFilename, indexWP);
  rifLonMission = save_wp_lon(&wayPointData);
  rifLatMission = save_wp_lat(&wayPointData);
  RaggioGPS = save_wp_radius(&wayPointData);

  if (wayPointData.ArriveMode < ARRIVE_LIM_1) {
    arriveModeNow = ARRIVE_M_NO_MIDPOINTS;
  }
  if ((wayPointData.ArriveMode >= ARRIVE_LIM_1) && (wayPointData.ArriveMode < ARRIVE_LIM_2)) {
    arriveModeNow = ARRIVE_M_SIMPLE;
  }
  if ((wayPointData.ArriveMode >= ARRIVE_LIM_2) && (wayPointData.ArriveMode < ARRIVE_LIM_3)) {
    arriveModeNow = ARRIVE_M_HAVERSINE;
  }
}

int32_t save_wp_lat(wayPoint *ptr) {
  int32_t new_lat = (int32_t)((ptr->Latitude) * 1000000);
  return new_lat;
}

int32_t save_wp_lon(wayPoint *ptr) {
  int32_t new_lon = (int32_t)((ptr->Longitude) * 1000000);
  return new_lon;
}

uint32_t save_wp_radius(wayPoint *ptr) {
  float tempRadius = (ptr->WaypointRadius) * 1000000.0;
  //uint32_t wp_radius = (uint32_t)((ptr->WaypointRadius)*1000000.0);
  uint32_t wp_radius = tempRadius;
  return wp_radius;
}

void restoreNavState(uint32_t index_step_rest) {

  Serial.println("RestoreNavState");

  if (goBack) {
    // save_new_wp(&missionParamDataCurrent,indexWPNow + 1);
    // rifLatMissionOld = save_wp_lat(&wayPointData);
    // rifLonMissionOld = save_wp_lon(&wayPointData);

    Serial.println("IFFFFFFFFF");
  } else {

    save_new_wp(&missionParamDataCurrent, indexWPNow - 1);
    rifLatMissionOld = save_wp_lat(&wayPointData);
    rifLonMissionOld = save_wp_lon(&wayPointData);
    Serial.println("ELSEEEEEEE");
  }

  save_new_wp(&missionParamDataCurrent, indexWPNow);
  rifLatMission = save_wp_lat(&wayPointData);
  rifLonMission = save_wp_lon(&wayPointData);

  index_step = index_step_rest;

  computeNewPoint(rifLatMission, rifLonMission, rifLatMissionOld, rifLonMissionOld, interStep, index_step, arriveModeNow);
}

//Controlla se A finisce con le cifre del numero B.
uint8_t checkNumSuffix(int A, int B) {
  // Converti numeri in stringhe
  String s1 = String(A);
  String s2 = String(B);

  // Lunghezza di s1 e s2
  int n1 = s1.length();
  int n2 = s2.length();

  // Se B ha meno cifre di A, return 0
  if (n1 < n2) {
    return 0;
  }

  // Controllo le n2 cifre partendo dall'ultima
  for (int i = 0; i < n2; i++) {
    // Se una è diversa, return 0
    if (s1[n1 - i - 1] != s2[n2 - i - 1]) {
      return 0;
    }
  }

  // Return 1
  return 1;
}