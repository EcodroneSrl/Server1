#define SOCKET_UNCONNECTED 0
#define SOCKET_CONNECTED 23

void cmdRWInit() {
  Serial.begin(115200);
  Serial6.begin(115200);  //Questo è fittizzio, c'è un array di baudrate applicati al cambio del mux addr
  Serial7.begin(9600);    //LoRa 9600
  Serial8.begin(115200);
  Serial3.begin(115200);
  getIP();
  SerialRWSocket.beginSocket(ServerSocket, ID_PORTA_SOCK, analBuff, id.boat[0], id.boat[1], id.boat[2]);
  SerialRWSocketJet.beginSocket(ServerSocketJet, ID_PORTA_SOCK_JET, analBuff, id.boat[0], id.boat[1], id.boat[2]);
  SerialRW6.begin(Serial6, ID_PORTA_6, analBuff, id.boat[0], id.boat[1], id.boat[2]);
  SerialRW7.begin(Serial7, ID_PORTA_7, analBuff, id.boat[0], id.boat[1], id.boat[2]);
  SerialRW8.beginRs485(Serial8, ID_PORTA_8, analBuff, id.boat[0], id.boat[1], id.boat[2], pin485En);
  SerialRWUSB.beginUSB(Serial, ID_PORTA_0, analBuff, id.boat[0], id.boat[1], id.boat[2]);
}

uint32_t cmdRwTime = 0;
void cmdRW() {
  cmdRwTime = millis();
  resetJSButtons();  //perché resetta i bottoni del joystick?
  cmdRadioRW();
  cmdUsbRW();
  cmdSocketRW();
  cmdRs485RW();
  cmdMuxRW();
  cmdRwTime = millis() - cmdRwTime;
}

void cmdRadioRW() {
  SerialRW7.CmdRW();
}

void cmdSocketRW() {
  for (uint8_t s = 0; s < MAX_SOCK_NUM; s++) {
    if (Ethernet.socketStatus(s) == SOCKET_UNCONNECTED) {
      socketActive = 0;
    }
    if (Ethernet.socketStatus(s) == SOCKET_CONNECTED) {
      socketActive = 1;
      break;
    }
  }

  SerialRWSocket.CmdRWSocket();
  SerialRWSocketJet.CmdRWSocket();
}

void cmdMuxRW() {
  SerialRW6.CmdRW();
}

void cmdRs485RW() {
  SerialRW8.CmdRW();
}
//*********************************************************************************

void cmdUsbRW() {
  SerialRWUSB.CmdRWUSB();
}

//************************************************
uint8_t cksumTest(uint8_t buff[]) {
  uint8_t cksum = 0;
  for (uint8_t i = 0; i < (buff[0] - 1); i++) {
    cksum += buff[i];
    //Serialprint(buff[i]);
    //Serialprint(F(" "));
  }
  //Serialprintln(F(""));
  if (cksum == buff[(buff[0] - 1)]) {
    return 1;
  } else {
    return 0;
  }
}
uint8_t cksumCompute(uint8_t buff[]) {
  uint8_t cksum = 0;
  for (uint8_t i = INDEX_BUF_LENG; i < (buff[INDEX_BUF_LENG] + INDEX_BUF_LENG - 1); i++) {
    cksum += buff[i];
  }
  return cksum;
}
//************************************************************************************

//************************************************************************************
struct __attribute__((packed)) commState {
  uint8_t porta, dispSorg, dispDest, disId, cmd1, cmd2, cmd3;
} commStateData[5];

void checkReceivingMission() {
  if ((millis() - time_receivingMission > timer_receivingMission) && receivingMission) {
    time_receivingMission = millis();
    costructBuff(&nWP_now, sizeof(nWP_now));
    sendCostructBuffCommState(INDEX_PINGPONG_MISSION);
  }
}

void sendCostructBuffCommState(uint8_t indexCommState) {
  sendCostructBuff(
    commStateData[indexCommState].porta,
    commStateData[indexCommState].dispSorg,
    commStateData[indexCommState].dispDest,
    commStateData[indexCommState].disId,
    commStateData[indexCommState].cmd1,
    commStateData[indexCommState].cmd2,
    commStateData[indexCommState].cmd3);
}

void saveCommState(uint8_t indexResend, uint8_t porta, uint8_t dispSorg, uint8_t dispDest, uint8_t disId,
                   uint8_t cmd1, uint8_t cmd2, uint8_t cmd3) {
  commStateData[indexResend].porta = porta;
  commStateData[indexResend].dispSorg = dispSorg;
  commStateData[indexResend].dispDest = dispDest;
  commStateData[indexResend].disId = disId;
  commStateData[indexResend].cmd1 = cmd1;
  commStateData[indexResend].cmd1 = cmd2;
  commStateData[indexResend].cmd1 = cmd3;
}