#define DEBUG_MES_SYNC false
#define DEBUG_SEND_COSTR_BUFF false
#define DEBUG_PRINT_SOCKET false
#define DEBUG_ALL_MPPT false

uint16_t initMultiVar = INDEX_BUF_CONTB - INDEX_BUF_LENG;
uint8_t indexByteSend = INDEX_BUF_CONTB;
uint8_t indexByteSend485 = INDEX_BUF_CONTB;
uint8_t bufferSend[255];
uint8_t bufferSend485[255];
uint16_t indexSD = 0;

uint8_t analBuff(uint8_t bufferAnal[], uint8_t porta) {
  indexSD++;
  logBufferAnalSD(bufferAnal, INDEX_SINCHAR_0, indexSD);
  
  combinaStruct(&mes, bufferAnal, sizeof(mes));
  digitalToggle(LED_BUILTIN);
  //loop_en_data.debugAnalBuffer = 0;
  if (loop_en_data.debugAnalBuffer) {
    for (int i = 0; i < bufferAnal[0]; i++) {
      Serialprint(bufferAnal[i]);
      Serialprint(F(","));
    }
    Serialprintln();
  }

  if (DEBUG_MES_SYNC) {
    Serialprint(mes.sorgCmd);
    Serialprint(F(","));
    Serialprint(mes.destCmd);
    Serialprint(F(","));
    Serialprint(mes.id_dCmd);
    Serialprint(F(","));
    Serialprint(mes.Cmd1);
    Serialprint(F(","));
    Serialprint(mes.Cmd2);
    Serialprint(F(","));
    Serialprint(mes.Cmd3);
    Serialprintln();
  }

  if (porta == ID_PORTA_8) {
    cmd_485_ok = 1;
  }

  if ((mes.destCmd) == myID) {
    switch (mes.id_dCmd) {
      case ID_MODULO_BASE:
        if (mes.Cmd1 == REQUEST_CMD1) {
          if (mes.Cmd2 == TEST_GENERIC_CMD2) {
            if (mes.Cmd3 == TEST_GENERIC_CMD3) {
              // Codice di test da provare
              Serialprint(F("Funzione di test:"));
              Serialprintln();
              //*******************
            }
          }
          if (mes.Cmd2 == EN_DEB_PRINT_CMD2) {
            combinaVar(&loop_en_data, sizeof(loop_en_data), bufferAnal);
            setLoopEn();
          }
          if (mes.Cmd2 == SEND_DUMMY_CMD2) {
            combinaVar(&dummyData, sizeof(dummyData), bufferAnal);
            boatLat = dummyData.lat;
            boatLon = dummyData.lon;
            gps.pvt1.lat = boatLat;
            gps.pvt1.lon = boatLon;
            gps.pvt1.year = dummyData.year;
            gps.pvt1.month = dummyData.month;
            gps.pvt1.day = dummyData.day;
            gps.pvt1.hour = dummyData.hour;
            gps.pvt1.min = dummyData.min;
            gps.pvt1.sec = dummyData.sec;
            gps.Date = date_to_FAT32(dummyData.year, dummyData.month, dummyData.day, dummyData.hour, dummyData.min);

            signalStrength = dummyData.signal;
            if (signalStrength > 0) {
              time_signal_jetson = millis();
              if (!restoreNavOk) {
                restoreNavOk = 1;
              }
            }

            dummy_risp_data.rifLatTrue = rifLatTrue;
            dummy_risp_data.rifLonTrue = rifLonTrue;
            dummy_risp_data.rifLatMission = rifLatMission;
            dummy_risp_data.rifLonMission = rifLonMission;
            dummy_risp_data.gpsLat = boatLat;
            dummy_risp_data.gpsLon = boatLon;

            costructBuff(&dummy_risp_data, sizeof(dummy_risp_data));
            sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);

            // Qui devo rispondere con la struttura dati arbitraria che mescola alcune variabili (non le dummy ma quelle vere)
          }

          if (mes.Cmd2 == UPDATE_MISS_LIST_CMD2) {
            if (mes.Cmd3 == UPDATE_FILE_LIST_CMD3) {
              fileCount = 0;
              combinaVar(&startIndex, sizeof(startIndex), bufferAnal);
              File root = extFlash.open("/");
              uint8_t endArray = whileAndSend(root, startIndex);

              costructBuff(&fileCount, sizeof(fileCount));
              costructBuff(&index_buff, sizeof(index_buff));
              costructBuff(folderArray, index_buff);

              if (endArray) {
                sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, UPDATE_MISS_LIST_CMD2, UPDATE_FILE_LIST_CMD3);
              } else {
                sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, UPDATE_MISS_LIST_CMD2, END_FILE_LIST_CMD3);
                fileCount = 0;
              }

              index_buff = 0;
            }
          }

          if (mes.Cmd2 == SET_MP_RADIUS_CMD2) {
            combinaVar(&RaggioGPSTrue, sizeof(RaggioGPSTrue), bufferAnal);
          }

          if (mes.Cmd2 == START_THIS_MISS_CMD2) {
            char idMissionNow[32];
            uint16_t nMissionNow;

            combinaMultiVar(&nMissionNow, sizeof(nMissionNow), bufferAnal);
            combinaMultiVar(&idMissionNow, sizeof(idMissionNow), bufferAnal);
            endCombinaMultiVar();

            Serial.println(idMissionNow);
            Serial.println(nMissionNow);

            updateCurrentMission(nMissionNow, idMissionNow);

            mission_active = 1;
            setFRAMmissionActive();
          }

          if (mes.Cmd2 == START_NEXT_SCHED_CMD2) {
            updateCurrentMission(scheduleDataNext.nMission, scheduleDataNext.idMission);
            updateMissions();
            scheduleRead();
            mission_active = 1;
            setFRAMmissionActive();
          }

          if (mes.Cmd2 == SEND_SCHEDULE_CMD2) {
            combinaVar(&scheduleData, sizeof(scheduleData), bufferAnal);
            scheduleWrite(&scheduleData);
            scheduleRead();
          }

          if (mes.Cmd2 == READ_SCHEDULE_CMD2) {
            debug_schedule = 1;
            scheduleRead();
          }

          if (mes.Cmd2 == GET_MISSION_CMD2) {
            if (mes.Cmd3 == GET_MISSION_PARAM_CMD3) {
              combinaVar(&flashFilename, sizeof(flashFilename), bufferAnal);
              Serial.println(flashFilename);
              if (!readMissionParamData(&missionParamData, sizeof(missionParamData), flashFilename))
                sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, 40, 4);
              else {
                costructBuff(&missionParamData, sizeof(missionParamData));
                sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
              }
            }
            if (mes.Cmd3 == GET_MISSION_WP_CMD3) {
              combinaVar(&nWP_now, sizeof(nWP_now), bufferAnal);
              if (!readMissionWaypointData(&wayPointData, sizeof(wayPointData), flashFilename, nWP_now))
                sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, 40, 4);
              else {
                costructBuff(&wayPointData, sizeof(wayPointData));
                sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
              }
            }
          }

          if (mes.Cmd2 == SAVE_MISSION_CMD2) {
            if (mes.Cmd3 == SAVE_MISSION_PARAM_CMD3) {
              nWP_now = 0;
              mission_cksum = 0;
              receivingMission = 1;

              combinaVar(&missionParamData, sizeof(missionParamData), bufferAnal);

              mission_cksum += sumStructBytes(&missionParamData, sizeof(missionParamData));

              createFlashFilename(&missionParamData);
              createMissionFile(flashFilename, &missionParamData);
              saveHeader(flashFilename, &missionParamData);

              costructBuff(&nWP_now, sizeof(nWP_now));
              sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, REQUEST_CMD1, mes.Cmd2, SAVE_MISSION_WP_CMD3);
            }
          }

          if (mes.Cmd2 == EN_SLEEP_CMD2) {
            Serialprintln(F("EN_SLEEP_CMD2"));
            sleep_on = 1;
            powerSelectAllOff();//spengo tutto
            writeSleepFlash(&sleep_on);
          }

          if (mes.Cmd2 == SET_DEBUG_PORT_CMD2) {
            combinaVar(&debug_port, sizeof(debug_port), bufferAnal);
          }

          if (mes.Cmd2 == SET_SD_CMD2) {
            if (mes.Cmd3 == SD_SET_DEB_TIME_CMD3) {
              combinaVar(&debug_time_sd, sizeof(debug_time_sd), bufferAnal);
            }

            if (mes.Cmd3 == SD_READ_CMD3) {
              sdExplore();
            }

            if (mes.Cmd3 == SD_DELETE_CMD3) {
              char filenameIn[64];
              combinaVar(&filenameIn, sizeof(filenameIn), bufferAnal);
              for (uint8_t i = 0; i < sizeof(filenameIn); i++) {
                if (filenameIn[i] == '\0') {
                  break;
                }
              }

              if (SD.exists(filenameIn)) {
                if (SD.open(filenameIn).isDirectory()) {
                  SD.rmdir(filenameIn);
                } else {
                  SD.remove(filenameIn);
                }
              }

              if (SD.exists(filenameIn)) {
                Serialprint(F("Eliminato!"));
                Serialprintln();
              }

              sdExplore();
            }
          }

          if (mes.Cmd2 == SET_FLASH_CMD2) {
            if (mes.Cmd3 == FLASH_READ_CMD3) {
              flashExplore();
              sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, 0, 8, 0);
            }

            if (mes.Cmd3 == FLASH_DELETE_CMD3) {
              char filenameIn[64];
              combinaVar(&filenameIn, sizeof(filenameIn), bufferAnal);
              for (uint8_t i = 0; i < sizeof(filenameIn); i++) {
                if (filenameIn[i] == '\0') {
                  break;
                }
              }

              if (extFlash.exists(filenameIn)) {
                if (extFlash.open(filenameIn).isDirectory()) {
                  extFlash.rmdir(filenameIn);
                } else {
                  extFlash.remove(filenameIn);
                }
              }

              if (extFlash.exists(filenameIn)) {
                Serialprint(F("Eliminato!"));
                Serialprintln();
              }

              flashExplore();
            }
          }

          if (mes.Cmd2 == REMOTE_CONTROL_CMD2) {
            telInputDev = mes.Cmd3;
            setRemoteControl();  // Save in EEPROM
          }

          if (mes.Cmd2 == SET_NAV_GPS_CMD2) {
            gpsSource = mes.Cmd3;
            setNavGPS();  // Save in EEPROM
          }

          if (mes.Cmd2 == SET_NAV_HEAD_CMD2) {
            thetaBSource = mes.Cmd3;
            setNavHead();  // Save in EEPROM
          }

          if (mes.Cmd2 == JS_DRIVING_SET_CMD2) {
            // combinaVar(&miscParamData.motorControlData,sizeof(miscParamData.motorControlData),bufferAnal);
          }

          if (mes.Cmd2 == JS_DRIVING_DATA_CMD2) {
            combinaVar(&JsCmdData[mes.Cmd3], sizeof(JsCmdData[mes.Cmd3]), bufferAnal);
            resetTimerJs(mes.Cmd3);
          }

          if (mes.Cmd2 == RADIO_DRIVING_SET_CMD2) {
            // combinaVar(&miscParamData.motorControlData,sizeof(miscParamData.motorControlData),bufferAnal);
          }

          if (mes.Cmd2 == RADIO_DRIVING_DATA_CMD2) {
            costructBuff(&radioChValue, sizeof(radioChValue));
            sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
          }

          if (mes.Cmd2 == SET_TEL_NAV_CMD2) {
            corrTetaDBut = 0;

            boatNavMode = NAV_TEL;
            boatNavSubMode = mes.Cmd3;

            if (boatNavSubMode == TEL_MODE_2) {
              TetaD = TetaB;
              setFRAMTetaD();
            }
            setNavMode();  // Save in EEPROM
          }

          if (mes.Cmd2 == SET_AUT_NAV_CMD2) {
            corrTetaDBut = 0;

            boatNavMode = NAV_AUT;
            boatNavSubMode = mes.Cmd3;
            setNavMode();  // Save in EEPROM
          }

          if (mes.Cmd2 == SET_MODE_TETAD_CMD2) {
            controlModeRoute = mes.Cmd3;
            combinaVar(&pidTheta, sizeof(pidTheta), bufferAnal);
            setControlTeta();  // Save in EEPROM
          }

          if (mes.Cmd2 == SET_MODE_VELD_CMD2) {
            controlModeVel = mes.Cmd3;
            combinaVar(&pidVel, sizeof(pidVel), bufferAnal);
            setControlVel();  // Save in EEPROM
          }

          if (mes.Cmd2 == SET_MISC_PARAM_CMD2) {
            combinaVar(&miscParamData, sizeof(miscParamData), bufferAnal);

            setMiscParam();  // Save in EEPROM
          }

          if (mes.Cmd2 == SET_JS_DEB_CMD2) {
            combinaVar(&js_debug_par, sizeof(js_debug_par), bufferAnal);
            // setJSDeb(); //Save in EEPROM
          }

          if (mes.Cmd2 == GET_CONTROL_INFO_CMD2) {
            costructBuff(&telInputDev, sizeof(telInputDev));
            costructBuff(&gpsSource, sizeof(gpsSource));
            costructBuff(&thetaBSource, sizeof(thetaBSource));
            costructBuff(&boatNavMode, sizeof(boatNavMode));
            costructBuff(&boatNavSubMode, sizeof(boatNavSubMode));
            costructBuff(&controlModeRoute, sizeof(controlModeRoute));
            costructBuff(&pidTheta, sizeof(pidTheta));
            costructBuff(&controlModeVel, sizeof(controlModeVel));
            costructBuff(&pidVel, sizeof(pidVel));
            costructBuff(&miscParamData, sizeof(miscParamData));
            costructBuff(&js_debug_par, sizeof(js_debug_par));

            sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, GET_CONTROL_INFO_CMD2, 0);
          }

          if (mes.Cmd2 == GET_ANTICOLL_TETAD_CMD2) {
            combinaVar(&corrTetaD_AC, sizeof(corrTetaD_AC), bufferAnal);
            time_tetaD_AC = millis();
          }

          if (mes.Cmd2 == GET_JETSON_SIGNAL_CMD2) {
            combinaVar(&signalStrength, sizeof(signalStrength), bufferAnal);
            if (signalStrength > 0) {
              time_signal_jetson = millis();
            }
          }

          if (mes.Cmd2 == GET_JETSON_WP_CMD2) {
            combinaMultiVar(&rifLatJetson, sizeof(rifLatJetson), bufferAnal);
            combinaMultiVar(&rifLonJetson, sizeof(rifLonJetson), bufferAnal);
            endCombinaMultiVar();
          }

          if (mes.Cmd2 == SET_DRONE_EQUIP_CMD2) {
            combinaVar(&equipData[mes.Cmd3], sizeof(&equipData[mes.Cmd3]), bufferAnal);
            setEquipment();
          }

          if (mes.Cmd2 == LOCK_MUX_CH_CMD2) {
            uint8_t ch_lock_send;
            switch (mes.Cmd3) {
              case ID_MODULO_BASE:
                combinaVar(&lock_index, sizeof(lock_index), bufferAnal);
                break;

              case ID_MUX_MPPT:
                combinaVar(&ch_lock_send, sizeof(ch_lock_send), bufferAnal);
                costructBuff(&ch_lock_send, sizeof(ch_lock_send));
                for (uint8_t i = 0; i < sizeof(id_mux); i++) {
                  if (id_mux[i] == ID_MUX_MPPT) {
                    setMuxChannel(i);
                    sendCostructBuff(ID_PORTA_6, myID, ID_MUX_MPPT, ID_MUX_MPPT, REQUEST_CMD1, LOCK_MUX_CH_CMD2, 0);
                  }
                }
                break;

              case ID_ECHO:
                combinaVar(&ch_lock_send, sizeof(ch_lock_send), bufferAnal);
                costructBuff(&ch_lock_send, sizeof(ch_lock_send));
                for (uint8_t i = 0; i < sizeof(id_mux); i++) {
                  if (id_mux[i] == ID_ECHO) {
                    setMuxChannel(i);
                    sendCostructBuff(ID_PORTA_6, myID, ID_ECHO, ID_ECHO, REQUEST_CMD1, LOCK_MUX_CH_CMD2, 0);
                  }
                }
                break;
            }
          }

          if (mes.Cmd2 == GET_IP_PORT_CMD2) {
            costructBuff(&SerialRWSocket.ip, sizeof(SerialRWSocket.ip));
            costructBuff(&tcp_port, sizeof(tcp_port));
            sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
          }

          if (mes.Cmd2 == SET_IP_PORT_CMD2) {
            combinaMultiVar(&SerialRWSocket.ip, sizeof(SerialRWSocket.ip), bufferAnal);
            combinaMultiVar(&tcp_port, sizeof(tcp_port), bufferAnal);
            endCombinaMultiVar();

            setIP();
            ServerSocket = EthernetServer(tcp_port);
            SerialRWSocket.beginSocket(ServerSocket, ID_PORTA_SOCK, analBuff, id.boat[0], id.boat[1], id.boat[2]);
          }

          if (mes.Cmd2 == COOLING_SET_CMD2) {
            pwmCool = mes.Cmd3;
          }

          if (mes.Cmd2 == REBOOT_TEENSY_CMD2) {
            doReboot();
          }
        }

        if (mes.Cmd1 == RESPONSE_CMD1) {
          if (mes.Cmd2 == SAVE_MISSION_CMD2) {
            if (mes.Cmd3 == SAVE_MISSION_WP_CMD3) {
              if (nWP_now < missionParamData.total_mission_nWP) {
                time_receivingMission = millis();
                combinaVar(&wayPointData, sizeof(wayPointData), bufferAnal);
                mission_cksum += sumStructBytes(&wayPointData, sizeof(wayPointData));
                saveWP(flashFilename, nWP_now, &wayPointData);
                nWP_now++;
                costructBuff(&nWP_now, sizeof(nWP_now));
                saveCommState(INDEX_PINGPONG_MISSION, porta, myID, mes.sorgCmd, mes.id_dCmd, REQUEST_CMD1, mes.Cmd2, mes.Cmd3);
                sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, REQUEST_CMD1, mes.Cmd2, mes.Cmd3);
              } else {
                receivingMission = 0;
                cksumTestFlash(flashFilename);
              }
            }
          }

          if (mes.Cmd2 == MAIN_DATA_CMD2) {
            switch (mes.sorgCmd) {
              case ID_PRUA:
                combinaMultiVar(&hfrIntData, sizeof(hfrIntData), bufferAnal);

                if (DEBUG_ALL_MPPT) {
                  Serial.print("\nI_in: ");
                  for (uint8_t i = 0; i < 10; i++) {
                    Serial.printf("%u ", hfrIntData[i].Iin);
                  }

                  Serial.print("\nV_in: ");
                  for (uint8_t i = 0; i < 10; i++) {
                    Serial.printf("%u ", hfrIntData[i].Vin);
                  }

                  Serial.print("\nI_out: ");
                  for (uint8_t i = 0; i < 10; i++) {
                    Serial.printf("%u ", hfrIntData[i].Iout);
                  }

                  Serial.print("\nV_out: ");
                  for (uint8_t i = 0; i < 10; i++) {
                    Serial.printf("%u ", hfrIntData[i].Vout);
                  }
                  Serial.println();
                }

                combinaMultiVar(&pressureValues, sizeof(pressureValues), bufferAnal);
                endCombinaMultiVar();
                break;

              case ID_MODULO_AMB:

                break;
            }
            cmd_485_ok = 1;
          }
        }
        break;

      case ID_POWER:
        if (mes.Cmd1 == REQUEST_CMD1) {
          if (mes.Cmd2 == POWER_EN_CMD2)  // Setta tutte le alimentazioni come da struct in arrivo
          {
            switch (mes.Cmd3) {
              case POWER_EN_SET_CMD3:
                combinaVar(&power_en, sizeof(power_en), bufferAnal);
                setPowerOnOff();
                powerSelectSet();
                break;

              case POWER_EN_GET_CMD3:
                costructBuff(&power_en, sizeof(power_en));
                break;
            }

          } else if (mes.Cmd2 == POWER_EN_JETSON_CMD2)  // Setta l'alimentazione della Jestson
          {
            power_en[PWR_JETSON] = mes.Cmd3;
            setPowerOnOff();
            powerSelectSet();
          }
          sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
        }
        break;

      case ID_PRUA:
        // per processare main data da 485
        break;

      case ID_MODULO_AMB:

        break;

      case ID_ALTO_LIVELLO:

        break;

      case ID_INTERFACCIA:

        break;

      case ID_RADIOCOMANDO:
        if (mes.Cmd1 == RESPONSE_CMD1) {
          combinaVar(&radioChValue, sizeof(radioChValue), bufferAnal);
          // debugRadioChValue();
        }
        break;

      case ID_IMU:
        if (mes.Cmd1 == REQUEST_CMD1) {
#ifdef BNO_086

          if (mes.Cmd2 == IMU_086_SRESET_CMD2) {
            imu.softReset();
            delay(10);
          }

          if (mes.Cmd2 == IMU_086_HRESET_CMD2) {
            imu.hardReset();
            delay(10);
          }

#endif

          if (mes.Cmd2 == IMU_GET_CMD2) {
            if (mes.Cmd3 == IMU_RPY_ACC_CMD3) {
              costructBuff(&imu.EulerAccData, sizeof(imu.EulerAccData));
              // sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
            }

#ifdef BNO_086

            if (mes.Cmd3 == IMU_GET_086_CAL_CMD3) {
              costructBuff(&imu.EulerAccData, sizeof(imu.EulerAccData));
            }

#endif
          }
          if (mes.Cmd2 == IMU_CFG_CMD2) {
            switch (mes.Cmd3) {
              case IMU_UPDATE_CFG_GET_CMD3:
                costructBuff(&imu.update_par, sizeof(imu.update_par));
                break;

              case IMU_UPDATE_CFG_SET_CMD3:
                combinaVar(&imu.update_par, sizeof(imu.update_par), bufferAnal);
                setConfigIMU();
                break;

              case IMU_UPDATE_CAL_GET_CMD3:
#ifdef BNO_055
                imu.getCalAll();
                costructBuff(&imu.cal_values, sizeof(imu.cal_values));
#endif
                break;

              case IMU_UPDATE_CAL_SET_CMD3:
                combinaVar(&imu.cal_values, sizeof(imu.cal_values), bufferAnal);
                setCalIMU();
                break;

#ifdef BNO_086

              case IMU_086_SET_REPORTS_CMD3:
                combinaVar(&imu.update_par, sizeof(imu.update_par), bufferAnal);
                imu.hardReset();
                delay(2000);
                imu.setReports();
                break;

              case IMU_086_REQ_CAL_STA_CMD3:
                imu.requestCalibSettings();
                break;

              case IMU_086_DEB_MES_EN_CMD3:
                imu.enableDebMes();
                break;

#endif
            }
          }
          if (mes.Cmd2 == IMU_DEB_CFG_CMD2) {
            switch (mes.Cmd3) {
              case IMU_DEB_CFG_GET_CMD3:
                costructBuff(&imu.debug_par, sizeof(imu.debug_par));
                break;

              case IMU_DEB_CFG_SET_CMD3:
                combinaVar(&imu.debug_par, sizeof(imu.debug_par), bufferAnal);
                setDebConfigIMU();
                break;
            }
          }
          // Sempre chiamato
          sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
        }

        break;

      case ID_MOTORI:
        if (mes.Cmd1 == REQUEST_CMD1) {
          if (mes.Cmd2 == MOTOR_DRIVE_CMD2) {
            if (mes.Cmd3 == MOTOR_OFF_CMD3) {
              combinaVar(&motorSetData.drive_off, sizeof(motorSetData.drive_off), bufferAnal);
            }
            if (mes.Cmd3 == MOTOR_ERPM_CMD3) {
              driveMode = MOTOR_ERPM_CMD3;  // var = 1
                                            //                 Serialprint(driveMode);
                                            //                 Serialprintln(F(","));
              combinaVar(&motorSetData.erpm_driving, sizeof(motorSetData.erpm_driving), bufferAnal);
            }
            if (mes.Cmd3 == MOTOR_CURRENT_CMD3) {
              driveMode = MOTOR_CURRENT_CMD3;  // var = 2
                                               //               Serialprint(driveMode);
                                               //               Serialprintln(F(","));
                                               //               combinaVar(&current_driving,sizeof(current_driving),bufferAnal);
            }
            if (mes.Cmd3 == MOTOR_ERPM_ALL_CMD3) {
              driveMode = MOTOR_ERPM_CMD3;
              combinaVar(&motorSetData.erpm_driving, sizeof(motorSetData.erpm_driving), bufferAnal);  //
              Serial.println(motorSetData.erpm_driving[0]);
              // rpm_avg_auto = motorSetData.erpm_driving[0];
            }
          }
          if (mes.Cmd2 == MOTOR_TELEM_CMD2) {
            if (mes.Cmd3 == MOTOR_TELEM_CDCS_CMD3) {
              costructBuff(&motors[INDEX_MOT_CD].bufferTelData, sizeof(motors[INDEX_MOT_CD].bufferTelData));
              costructBuff(&motors[INDEX_MOT_CS].bufferTelData, sizeof(motors[INDEX_MOT_CS].bufferTelData));
            }
            if (mes.Cmd3 == MOTOR_TELEM_DDSS_CMD3) {
              costructBuff(&motors[INDEX_MOT_DD].bufferTelData, sizeof(motors[INDEX_MOT_DD].bufferTelData));
              costructBuff(&motors[INDEX_MOT_SS].bufferTelData, sizeof(motors[INDEX_MOT_SS].bufferTelData));
            }
          }
          sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
        }
        break;

      case ID_BMS:
        if (mes.Cmd1 == REQUEST_CMD1) {
          if (mes.Cmd2 == BMS_PARAM_CMD2) {
            if (mes.Cmd3 == BMS_SET_PARAM_CMD3) {
              combinaVar(&bms.start_par, sizeof(bms.start_par), bufferAnal);
              setConfigBMS();
              bms.setCellNum(bms.start_par.ncell);                                                                    // Setto il numero di celle
              bms.setCutOff(bms.start_par.povp, bms.start_par.povp_rel, bms.start_par.puvp, bms.start_par.puvp_rel);  // Setto i cutoff
              bms.setMOSFET(bms.start_par.chg_on_off, bms.start_par.dsg_on_off);                                      // Abilito/disabilito carica/scarica
              bms.en_dis_read_eprom(bms.start_par.read_eeprom_on_off);                                                // Abilito/disabilito lettura EEPROM
            }
            if (mes.Cmd3 == BMS_GET_PARAM_CMD3) {
              costructBuff(&bms.start_par, sizeof(bms.start_par));
            }
          }
          if (mes.Cmd2 == BMS_GET_DATA_CMD2) {
            if (mes.Cmd3 == BMS_GET_VCELL_CMD3) {
              costructBuff(&bms.vcell, sizeof(bms.vcell));
            }
            if (mes.Cmd3 == BMS_GET_BASIC_CMD3) {
              costructBuff(&bms.BasicData, sizeof(bms.BasicData));
            }
            if (mes.Cmd3 == BMS_GET_EEPROM_CMD3) {
              costructBuff(&bms.EepromData, sizeof(bms.EepromData));
            }
          }
          if (mes.Cmd2 == BMS_DEB_CFG_CMD2) {
            switch (mes.Cmd3) {
              case BMS_DEB_CFG_GET_CMD3:
                costructBuff(&bms.debug_par, sizeof(bms.debug_par));
                break;

              case BMS_DEB_CFG_SET_CMD3:
                combinaVar(&bms.debug_par, sizeof(bms.debug_par), bufferAnal);
                setDebConfigBMS();
                break;
            }
          }
          sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
        }
        break;

      case ID_GPS:
        if (mes.Cmd1 == REQUEST_CMD1) {
          if (mes.Cmd2 == GPS_GET_CMD2) {
            if (mes.Cmd3 == GPS_NAV_PVT_CMD3) {
              costructBuff(&gps.pvt1, sizeof(gps.pvt1));
            }
            if (mes.Cmd3 == GPS_NAV_RELPOSNED_CMD3) {
              costructBuff(&gps.relposned1, sizeof(gps.relposned1));
            }
          }
          if (mes.Cmd2 == GPS_DEB_CFG_CMD2) {
            switch (mes.Cmd3) {
              case GPS_DEB_CFG_GET_CMD3:
                costructBuff(&gps.debug_par, sizeof(gps.debug_par));
                break;

              case GPS_DEB_CFG_SET_CMD3:
                combinaVar(&gps.debug_par, sizeof(gps.debug_par), bufferAnal);
                // setDebConfigGPS();
                break;
            }
          }
          sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
        }
        break;

      case ID_ECHO:
        if (mes.Cmd1 == REQUEST_CMD1) {
          if (mes.Cmd2 == ECHO_GET_CMD2) {  // Questo funziona perchè manda la propria struct
            costructBuff(&echoBuffer, sizeof(echoBuffer));
            sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
          }

          else if (mes.Cmd2 == ECHO_CFG_CMD2) {  // TOFIX: questo funziona male e a tratti perchè NON risponde con i dati in locale
            if (mes.Cmd3 == ECHO_CFG_SET_CMD3) {
              combinaVar(&echo_cfg_data.sensor_en, sizeof(echo_cfg_data.sensor_en), bufferAnal);
              uint8_t temp_probe_num = 0;
              for (uint8_t i = 0; i < sizeof(echo_cfg_data.sensor_en); i++) {
                if (echo_cfg_data.sensor_en[i]) {
                  temp_probe_num++;
                }
              }
              echo_cfg_data.echo_probe_num = temp_probe_num;
              costructBuff(&echo_cfg_data, sizeof(echo_cfg_data));
              sendCostructBuff(ID_PORTA_6, myID, ID_ECHO, ID_ECHO, REQUEST_CMD1, mes.Cmd2, mes.Cmd3);
            }
            if (mes.Cmd3 == ECHO_CFG_GET_CMD3) {
              sendCostructBuff(ID_PORTA_6, myID, ID_ECHO, ID_ECHO, REQUEST_CMD1, mes.Cmd2, mes.Cmd3);
            }
          }

          else if (mes.Cmd2 == ECHO_DEB_CFG_CMD2) {
            if (mes.Cmd3 == ECHO_DEB_CFG_GET_CMD3) {
              costructBuff(&echo_debug_par, sizeof(echo_debug_par));
              sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
            }

            if (mes.Cmd3 == ECHO_DEB_CFG_SET_CMD3) {
              combinaVar(&echo_debug_par, sizeof(echo_debug_par), bufferAnal);
            }
          }
        }

        if (mes.Cmd1 == RESPONSE_CMD1) {
          if (mes.Cmd2 == ECHO_NANO_GET_CMD2) {
            combinaVar(&echoBuffer, sizeof(echoBuffer), bufferAnal);
          }
          if (mes.Cmd2 == ECHO_CFG_CMD2) {
            if (mes.Cmd3 == ECHO_CFG_GET_CMD3) {
              combinaVar(&echo_cfg_data, sizeof(echo_cfg_data), bufferAnal);
              costructBuff(&echo_cfg_data, sizeof(echo_cfg_data));
              sendCostructBuff(PORTA_0, myID, ID_INTERFACCIA, ID_ECHO, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
            }
          }
        }
        break;

      case ID_LORA:
        Serial.println("ID_LORA");
        if (mes.Cmd1 == REQUEST_CMD1) {
          if (mes.Cmd2 == LORA_GET_CONFIG_CMD2) {
            Serial.println("LORA_GET_CONFIG_CMD2");
            costructBuff(&lora_settings_data, sizeof(lora_settings_data));
            sendCostructBuff(porta, myID, mes.sorgCmd, mes.id_dCmd, RESPONSE_CMD1, mes.Cmd2, mes.Cmd3);
          } else if (mes.Cmd2 == LORA_SET_CONFIG_CMD2) {
            Serial.println("LORA_SET_CONFIG_CMD2");
            combinaVar(&lora_settings_data, sizeof(lora_settings_data), bufferAnal);
            LoRa_setConfig();
          }
        }
        break;

      case ID_MPPT:
        if (mes.Cmd1 == REQUEST_CMD1) {
          if (mes.Cmd2 == MPPT_GET_CMD2) {
            costructBuff(&hfrIntData[mes.Cmd3], sizeof(hfrIntData[0]));
            sendCostructBuff(porta, myID, ID_INTERFACCIA, mes.id_dCmd, RESPONSE_CMD1, 0, 0);
          }
        }
        break;

    }  // Fine switch case id_d
  }    // Fine myID ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  if ((mes.destCmd == ID_PRUA) || (mes.destCmd == ID_MODULO_AMB) || (mes.destCmd == ID_ROBOT_ARM_1) || (mes.destCmd == ID_ROBOT_ARM_2) || (mes.destCmd == ID_INCUBATORE)) {
    forwardMessage(ID_PORTA_8, bufferAnal, true);
  }

  if ((mes.destCmd == ID_LED) || (mes.destCmd == ID_ECHO) || (mes.destCmd == ID_RADIOCOMANDO)) {
    setMuxChannel(getIndexToID(mes.destCmd));
    Serial6.begin(mux_ch_baud[getIndexToID(mes.destCmd)]);
    forwardMessage(ID_PORTA_6, bufferAnal, false);
    resetTime_multiserial();
  }

  if (mes.destCmd == ID_INTERFACCIA) {
    forwardMessage(ID_PORTA_0, bufferAnal, false);
  }
  return 1;
}

void costructBuff(void *ptr1, uint16_t lenVar) {
  for (uint16_t i = 0; i < lenVar; i++) {
    bufferSend[indexByteSend] = ((unsigned char *)ptr1)[i];

    indexByteSend++;
  }
  initMultiVar = INDEX_BUF_CONTB - INDEX_BUF_LENG;
}

void endCombinaMultiVar() {
  initMultiVar = INDEX_BUF_CONTB - INDEX_BUF_LENG;
}

uint32_t sendCostructBuffTime;
void sendCostructBuff(uint8_t porta, uint8_t dispSorg, uint8_t dispDest, uint8_t disId, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3) {
  sendCostructBuffTime = millis();
  sendCostructBuffB(porta, dispSorg, dispDest, disId, cmd1, cmd2, cmd3, 0);
  sendCostructBuffTime = millis() - sendCostructBuffTime;
}

void sendCostructBuffB(uint8_t porta, uint8_t dispSorg, uint8_t dispDest, uint8_t disId, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t cmd485) {

  bufferSend[INDEX_SINCHAR_0] = id.boat[0];
  bufferSend[INDEX_SINCHAR_1] = id.boat[1];
  bufferSend[INDEX_SINCHAR_2] = id.boat[2];

  bufferSend[INDEX_BUF_LENG] = indexByteSend - INDEX_SINCHAR_2;
  bufferSend[INDEX_BUF_SORG] = dispSorg;
  bufferSend[INDEX_BUF_DEST] = dispDest;
  bufferSend[INDEX_BUF_ID_D] = disId;

  bufferSend[INDEX_BUF_CMD_1] = cmd1;
  bufferSend[INDEX_BUF_CMD_2] = cmd2;
  bufferSend[INDEX_BUF_CMD_3] = cmd3;
  bufferSend[indexByteSend] = cksumCompute(bufferSend);

  if (DEBUG_SEND_COSTR_BUFF) {
    for (uint8_t i = 0; i < sizeof(bufferSend); i++) {
      Serial.print(bufferSend[i]);
      Serial.print(",");
    }
    Serial.println();
  }

  // logBufferAnalSD(bufferSend, INDEX_BUF_LENG, indexSD); //TOFIX: Se la SD si rompe, questo blocca il sistema. Disattivato con Marco

  //tofix: Questu + 1 pare siano necessari, capire il funzionamento
  if (porta == ID_PORTA_0) {
    PORTA_0.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_1) {
    PORTA_1.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_5) {
    PORTA_5.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_6) {
    PORTA_6.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_7) {
    PORTA_7.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_8) {
    if (cmd485) {
      // Serial.println("sendCMd485");
      combinaStruct(&bufferSend485, &bufferSend, sizeof(bufferSend));
      indexByteSend485 = indexByteSend;
      index_485_old = index_485;
      index_485 = 0;
    } else {
      PORTA_8.write(bufferSend, indexByteSend + 1);
    }
    //
  }
  if (porta == ID_PORTA_SOCK) {
    SerialRWSocket.client.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_SOCK_JET) {
    SerialRWSocketJet.client.write(bufferSend, indexByteSend + 1);
  }
  indexByteSend = INDEX_BUF_CONTB;
}

void forwardMessage(uint8_t porta, uint8_t bufferAnal[], uint8_t cmd485) {
  memset(&bufferSend, 0, sizeof(bufferSend));

  bufferSend[INDEX_SINCHAR_0] = id.boat[0];
  bufferSend[INDEX_SINCHAR_1] = id.boat[1];
  bufferSend[INDEX_SINCHAR_2] = id.boat[2];

  uint8_t length = bufferAnal[0];
  indexByteSend = length + INDEX_BUF_LENG;

  memcpy(&bufferSend[INDEX_BUF_LENG], bufferAnal, length);

  // for(uint8_t i = 0; i < sizeof(bufferSend); i++)
  // {
  //   Serial.print(bufferSend[i]);
  //   Serial.print(",");
  // }
  // Serial.println();

  if (porta == ID_PORTA_0) {
    PORTA_0.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_1) {
    PORTA_1.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_5) {
    PORTA_5.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_6) {
    PORTA_6.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_7) {
    PORTA_7.write(bufferSend, indexByteSend + 1);
  }
  if (porta == ID_PORTA_8) {
    if (cmd485) {
      // Serial.println("sendCmd485");
      combinaStruct(&bufferSend485, &bufferSend, sizeof(bufferSend));
      indexByteSend485 = indexByteSend;
      index_485_old = index_485;
      index_485 = 0;
    } else {
      PORTA_8.write(bufferSend, indexByteSend + 1);
    }
  }

  indexByteSend = INDEX_BUF_CONTB;
}

void combinaVar(void *ptr1, uint16_t lenVar, uint8_t bufferAnal[]) {
  uint8_t initVar = INDEX_BUF_CONTB - INDEX_BUF_LENG;
  for (uint16_t i = 0; i < lenVar; i++) {
    ((unsigned char *)ptr1)[i] = bufferAnal[initVar + i];
  }
}

void combinaMultiVar(void *ptr1, uint16_t lenVar, uint8_t bufferAnal[]) {
  for (uint16_t i = 0; i < lenVar; i++) {
    ((unsigned char *)ptr1)[i] = bufferAnal[initMultiVar];
    initMultiVar++;
  }
}

void combinaStruct(void *ptr1, void *ptr2, uint16_t lenVar) {
  for (uint16_t i = 0; i < lenVar; i++) {
    ((unsigned char *)ptr1)[i] = ((unsigned char *)ptr2)[i];
  }
}

uint32_t sumStructBytes(void *ptr1, uint16_t lenVar) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < lenVar; i++) {
    sum = ((unsigned char *)ptr1)[i];
  }
  return sum;
}