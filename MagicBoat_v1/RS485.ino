uint8_t lastIndexMppt = 0;

uint32_t ser485Time = 0;
void compute485() {
  ser485Time = millis();
  if ((millis() - time_request_485 > timer_request_485) || (cmd_485_ok)) {
    time_request_485 = millis();

    if (cmd_485_ok) {
      cmd_485_ok = 0;
    }

    struct __attribute__((packed)) mainParam {
      int32_t lat = boatLat;
      int32_t lon = boatLon;

      float yaw = imu.EulerAccData.Yaw;
      float pitch = imu.EulerAccData.Pitch;
      float roll = imu.EulerAccData.Roll;
    } mainParamData;

    switch (index_485) {
      case BOW_RS485:
        // Serial.println("BOW_RS485");
        costructBuff(&mainParamData, sizeof(mainParamData));
        sendCostructBuff(ID_PORTA_8, myID, ID_PRUA, ID_PRUA, REQUEST_CMD1, MAIN_DATA_CMD2, 0);

        //Send MPPT update request
        //sendCostructBuff(ID_PORTA_8, myID, ID_PRUA, ID_MPPT, REQUEST_CMD1, MPPT_GET_CMD2, lastIndexMppt);
        //lastIndexMppt++;
        //if (lastIndexMppt == 16) lastIndexMppt = 0;

        break;

      case STERN_RS485:
        // Serial.println("STERN_RS485");
        costructBuff(&mainParamData, sizeof(mainParamData));
        sendCostructBuff(ID_PORTA_8, myID, ID_POPPA, ID_POPPA, REQUEST_CMD1, MAIN_DATA_CMD2, 0);
        break;

      case AMB_RS485:
        // Serial.println("AMB_RS485");
        costructBuff(&mainParamData, sizeof(mainParamData));
        sendCostructBuff(ID_PORTA_8, myID, ID_MODULO_AMB, ID_MODULO_AMB, REQUEST_CMD1, MAIN_DATA_CMD2, 0);
        break;

      case CMD_RS485:
        //Serial.println("CMD_RS485");
        PORTA_8.write(bufferSend485, indexByteSend485 + 1);
        break;
    }
    if (index_485 <= MODULES_NUM) {
      if (index_485 != 0) {
        index_485++;
      } else {
        index_485 = index_485_old;
      }
    } else {
      index_485 = 1;
    }
  }
  ser485Time = millis() - ser485Time;
}