//Al momento le define per structType di IMU e GPS non si
//incrociano ma forse potrebbe essere saggio rifarle da capo

void sendJetson(uint8_t structType) {
  /*
  switch(structType)
  {
    case PVT_OK:
      costructBuff(&gps.pvt1,sizeof(gps.pvt1));
    break;

    case RELPOSNED_OK:
      costructBuff(&gps.relposned1,sizeof(gps.relposned1));
    break;

    case SENSOR_REPORTID_ROTATION_VECTOR:
      costructBuff(&imu.EulerAccData,sizeof(imu.EulerAccData));
    break;
  }
  
  sendCostructBuff(ID_PORTA_SOCK,myID,ID_ALTO_LIVELLO,ID_ALTO_LIVELLO,REQUEST_CMD1,SEND_DATA_JETSON_CMD2,structType);
  sendCostructBuff(ID_PORTA_SOCK_JET,myID,ID_ALTO_LIVELLO,ID_ALTO_LIVELLO,REQUEST_CMD1,SEND_DATA_JETSON_CMD2,structType);
  //va fatta una nuov soket per la jetson
  */
}