#include "MagicBoat_CmdRW.h"

#define DEBUG_CMDRW false

void MagicBoat_CmdRW::begin(HardwareSerial &SerialNow, uint8_t id_porta, uint8_t (*analBuff)(uint8_t bufferAnal[], uint8_t porta), uint8_t idCmd1, uint8_t idCmd2, uint8_t idCmd3) {

  SerialCmd = &SerialNow;
  id_portaCmd = id_porta;
  analBuffCallback = analBuff;
  idCmd.boat[0] = idCmd1;
  idCmd.boat[1] = idCmd2;
  idCmd.boat[2] = idCmd3;
}

void MagicBoat_CmdRW::beginRs485(HardwareSerial &SerialNow, uint8_t id_porta, uint8_t (*analBuff)(uint8_t bufferAnal[], uint8_t porta), uint8_t idCmd1, uint8_t idCmd2, uint8_t idCmd3, uint8_t enPin) {

  SerialCmd = &SerialNow;
  id_portaCmd = id_porta;
  SerialCmd->transmitterEnable(enPin);
  analBuffCallback = analBuff;
  idCmd.boat[0] = idCmd1;
  idCmd.boat[1] = idCmd2;
  idCmd.boat[2] = idCmd3;
}

void MagicBoat_CmdRW::beginUSB(usb_serial_class &SerialNowUSB, uint8_t id_porta, uint8_t (*analBuff)(uint8_t bufferAnal[], uint8_t porta), uint8_t idCmd1, uint8_t idCmd2, uint8_t idCmd3) {

  SerialCmdUSB = &SerialNowUSB;
  id_portaCmd = id_porta;
  analBuffCallback = analBuff;
  idCmd.boat[0] = idCmd1;
  idCmd.boat[1] = idCmd2;
  idCmd.boat[2] = idCmd3;
}

void MagicBoat_CmdRW::beginSocket(EthernetServer &SerialNowServer, uint8_t id_porta, uint8_t (*analBuff)(uint8_t bufferAnal[], uint8_t porta), uint8_t idCmd1, uint8_t idCmd2, uint8_t idCmd3) {
  IPAddress ipNow = IPAddress(ip[0], ip[1], ip[2], ip[3]);
  Ethernet.init(20); // Teensy++ 2.0
  Ethernet.begin(mac, 0, 0);
  Ethernet.setLocalIP(ipNow);
  Ethernet.setGatewayIP(gateway);

  SerialCmdServer = &SerialNowServer;

  SerialCmdServer->begin();

  id_portaCmd = id_porta;
  analBuffCallback = analBuff;
  idCmd.boat[0] = idCmd1;
  idCmd.boat[1] = idCmd2;
  idCmd.boat[2] = idCmd3;
}

#define STEPCMD_1 1
#define STEPCMD_2 2
#define STEPCMD_3 3
#define STEPCMD_END 0
#define STEPCMD_NULL 5

//TOFIX: CmdRW e CmdRWUSB sono identici, si potrebbe fare una funzione unica inserendo come parametro il puntatore alla porta seriale
uint8_t MagicBoat_CmdRW::CmdRW() {
  multiserial_ok = 0;
  uint8_t buff_ok = STEPCMD_NULL;
  uint16_t numByteNow = SerialCmd->available();

  //if (numByteNow) Serial7.print(numByteNow); //TOFIX: rimuovere

  for (uint16_t i = 0; i < numByteNow; i++) {
    // condizione di stop verificata quando il messaggio è stato letto correttamente
    if (multiserial_ok)
      break;

    uint8_t dataread = SerialCmd->read();

    switch (buff_step) {
    case 0:
      if (dataread == idCmd.boat[0]) {
        buff_step++;
      } else {
        // buff_step = 0;
        buff_ok = STEPCMD_1;
      }
      break;
    case 1:
      if (dataread == idCmd.boat[1]) {
        buff_step++;
      } else {
        buff_step = 0;
        buff_ok = STEPCMD_2;
      }
      break;
    case 2:
      if (dataread == idCmd.boat[2]) {
        buff_step++;
      } else {
        buff_step = 0;
        buff_ok = STEPCMD_3;
      }
      break;
    case 3:
      lenCmd = dataread;
      indexByte = 0;
      bufferRead[indexByte] = dataread;
      buff_step++;
      break;
    case 4:
      indexByte++;
      bufferRead[indexByte] = dataread;

      if (indexByte == lenCmd - 1) {
        if (cksumTest(bufferRead)) {
          analBuffCallback(bufferRead, id_portaCmd);
          multiserial_ok = 1; // serve a fermare la lettura del messaggio se ci fossero altri dati disponibili. in quel caso sarebbe l'inizio di un nuovo messaggio.
          buff_ok = STEPCMD_END;
        } else {
          Serial.print("SER_CKS_ERR: ");

          for(int i = 0; i < lenCmd; i++){
            Serial.print(bufferRead[i]);
            Serial.print(" ");
          }
          Serial.println("");
        }
        buff_step = 0;
      }
      break;
    }
  }
  return buff_ok;
}

uint8_t MagicBoat_CmdRW::CmdRWUSB() {
  multiserial_ok = 0;
  uint8_t buff_ok = STEPCMD_NULL;
  uint16_t numByteNow = SerialCmdUSB->available();
  for (uint16_t i = 0; i < numByteNow; i++) {
    // condizione di stop verificata quando il messaggio è stato letto correttamente
    if (multiserial_ok)
      break;

    uint8_t dataread = SerialCmdUSB->read();

    switch (buff_step) {
    case 0:
      if (dataread == idCmd.boat[0])
        buff_step++;
      else {
        buff_step = 0;
        buff_ok = STEPCMD_1;
      }
      break;
    case 1:
      if (dataread == idCmd.boat[1])
        buff_step++;
      else {
        buff_step = 0;
        buff_ok = STEPCMD_2;
      }
      break;
    case 2:
      if (dataread == idCmd.boat[2])
        buff_step++;
      else {
        buff_step = 0;
        buff_ok = STEPCMD_3;
      }
      break;
    case 3:
      lenCmd = dataread;
      indexByte = 0;
      bufferRead[indexByte] = dataread;
      buff_step++;
      break;
    case 4:
      indexByte++;
      bufferRead[indexByte] = dataread;

      if (indexByte == lenCmd - 1) {
        if (cksumTest(bufferRead)) {
          analBuffCallback(bufferRead, id_portaCmd);
          multiserial_ok = 1; // serve a fermare la lettura del messaggio se ci fossero altri dati disponibili. in quel caso sarebbe l'inizio di un nuovo messaggio.
          buff_ok = STEPCMD_END;
        } else {
          Serial.println("USB_CKS_ERR");
        }
        buff_step = 0;
      }
      break;
    }
  }
  return buff_ok;
}

uint8_t MagicBoat_CmdRW::CmdRWSocket() {
  multiserial_ok = 0;
  uint8_t buff_ok = STEPCMD_NULL;
  client = SerialCmdServer->accept();
  if (client.connected()) {
    uint32_t numByteNow = client.available();
    for (uint32_t i = 0; i < numByteNow; i++) {
      if (multiserial_ok)
        break;

      uint8_t dataread = client.read();

      switch (buff_step) {
      case 0:
        if (dataread == idCmd.boat[0]) {
          buff_step++;
        } else {
          buff_step = 0;
          buff_ok = STEPCMD_1;
        }
        break;
      case 1:
        if (dataread == idCmd.boat[1]) {
          buff_step++;
        } else {
          buff_step = 0;
          buff_ok = STEPCMD_2;
        }
        break;
      case 2:
        if (dataread == idCmd.boat[2]) {
          buff_step++;
        } else {
          buff_step = 0;
          buff_ok = STEPCMD_3;
        }
        break;
      case 3:
        lenCmd = dataread;
        indexByte = 0;
        bufferRead[indexByte] = dataread;
        buff_step++;
        break;
      case 4:
        indexByte++;
        bufferRead[indexByte] = dataread;

        if (indexByte == lenCmd - 1) {
          if (cksumTest(bufferRead)) {
            analBuffCallback(bufferRead, id_portaCmd);
            multiserial_ok = 1; // serve a fermare la lettura del messaggio se ci fossero altri dati disponibili. in quel caso sarebbe l'inizio di un nuovo messaggio.
            buff_ok = STEPCMD_END;
          } else {
            Serial.println("SOCK_CKS_ERR");
          }
          buff_step = 0;
        }
        break;
      }
    }
  } else {
    client.stop();
  }
  return buff_ok;
}

uint8_t MagicBoat_CmdRW::cksumTest(uint8_t buff[]) {
  uint8_t cksum = 0;

  for (uint8_t i = 0; i < (buff[0] - 1); i++)
    cksum += buff[i];

  //controlla se il checksum calcolato è uguale a quello ricevuto
  return cksum == buff[(buff[0] - 1)] ? 1 : 0;
}