void equipInit() {
  for (uint8_t i = 0; i < MODULES_NUM; i++) {
    switch (i) {
      costructBuff(&equipData[i], sizeof(equipData[i]));

      case BOW_RS485:
        sendCostructBuff(ID_PORTA_8, myID, ID_PRUA, ID_PRUA, REQUEST_CMD1, SET_DRONE_EQUIP_CMD2, 0);
        break;

      case STERN_RS485:
        sendCostructBuff(ID_PORTA_8, myID, ID_POPPA, ID_POPPA, REQUEST_CMD1, SET_DRONE_EQUIP_CMD2, 0);
        break;

      case AMB_RS485:
        sendCostructBuff(ID_PORTA_8, myID, ID_MODULO_AMB, ID_MODULO_AMB, REQUEST_CMD1, SET_DRONE_EQUIP_CMD2, 0);
        break;
    }
  }
}