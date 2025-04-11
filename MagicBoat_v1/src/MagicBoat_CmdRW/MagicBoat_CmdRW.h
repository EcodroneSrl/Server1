#ifndef MagicBoat_CmdRW_h
#define MagicBoat_CmdRW_h

#include "Arduino.h"
#include "HardwareSerial.h"
#include "NativeEthernet.h"
#if not defined(ARDUINO_TEENSY41)
#define usb_serial_class HardwareSerial
#endif
// #if defined(__IMXRT1062__) || defined (ARDUINO_TEENSY40) || defined (ARDUINO_TEENSY41)

class MagicBoat_CmdRW {
public:
  struct __attribute__((packed)) id {
    uint8_t boat[3] = {0x10, 0x11, 0x12};
  } idCmd;

  uint8_t (*analBuffCallback)(uint8_t bufferAnal[], uint8_t porta);

  void begin(HardwareSerial &SerialNow, uint8_t id_porta, uint8_t (*analBuff)(uint8_t bufferAnal[], uint8_t porta), uint8_t idCmd1, uint8_t idCmd2, uint8_t idCmd3);
  void beginUSB(usb_serial_class &SerialNow, uint8_t id_porta, uint8_t (*analBuff)(uint8_t bufferAnal[], uint8_t porta), uint8_t idCmd1, uint8_t idCmd2, uint8_t idCmd3);
  void beginSocket(EthernetServer &SerialNowServer, uint8_t id_porta, uint8_t (*analBuff)(uint8_t bufferAnal[], uint8_t porta), uint8_t idCmd1, uint8_t idCmd2, uint8_t idCmd3);
  void beginRs485(HardwareSerial &SerialNow, uint8_t id_porta, uint8_t (*analBuff)(uint8_t bufferAnal[], uint8_t porta), uint8_t idCmd1, uint8_t idCmd2, uint8_t idCmd3, uint8_t enPin);

  uint8_t CmdRW();
  uint8_t CmdRWUSB();
  uint8_t CmdRWSocket();

  uint8_t cksumTest(uint8_t buff[]);

  uint8_t lenCmd = 0;
  uint8_t indexByte = 0;
  uint8_t buff_step = 0;
  uint8_t multiserial_ok = 0;
  uint8_t bufferRead[255];
  uint8_t client_conn = 0;

  uint8_t id_portaCmd;

  HardwareSerial *SerialCmd;
  usb_serial_class *SerialCmdUSB;
  EthernetServer *SerialCmdServer;
  EthernetClient client;

  byte mac[6] = {0xE8, 0x2A, 0xEA, 0x4B, 0x1F, 0xC3};
  uint8_t ip[4] = {192, 168, 1, 213};
  IPAddress gateway = IPAddress(192, 168, 1, 1);

private:
};

#endif