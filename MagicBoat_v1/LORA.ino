// #include <EBYTE.h>
// EBYTE LoRa(&Serial7, 41, 40, 39);

void LoRaInit() {
  LoRa.init(2);
  LoRa_getConfig();
}
#define PWR_21dBm 1
#define PWR_24dBm 2
#define PWR_27dBm 3
#define PWR_30dBm 4

void LoRa_getConfig() {
  lora_settings_data.addressH = LoRa.GetAddressH();
  lora_settings_data.addressL = LoRa.GetAddressL();
  lora_settings_data.channel = LoRa.GetChannel();

  switch (LoRa.GetAirDataRate()) {
  case ADR_19200:
    lora_settings_data.airDataRate = 0;
    break;
  case ADR_9600:
    lora_settings_data.airDataRate = 1;
    break;
  case ADR_4800:
    lora_settings_data.airDataRate = 2;
    break;
  case ADR_2400:
    lora_settings_data.airDataRate = 3;
    break;
  }

  switch (LoRa.GetTransmitPower()) {
  case PWR_21dBm:
    lora_settings_data.txPower = 0;
    break;
  case PWR_24dBm:
    lora_settings_data.txPower = 1;
    break;
  case PWR_27dBm:
    lora_settings_data.txPower = 2;
    break;
  case PWR_30dBm:
    lora_settings_data.txPower = 3;
    break;
  }
}

void LoRa_setConfig() {
  LoRa.SetUARTBaudRate(UDR_9600);                // Set LoRa-MCU serial baudrate. CHECK ALSO Serial7.Begin(...)
  LoRa.SetMode(MODE_NORMAL);                     // Set TX + RX mode
  LoRa.SetAddressH(lora_settings_data.addressH); // Set ID (MSB byte)
  LoRa.SetAddressL(lora_settings_data.addressL); // Set ID (LSB byte)
  LoRa.SetChannel(lora_settings_data.channel);   // Set channel 11
  LoRa.SetParityBit(PB_8N1);                     // Set parity bits. packet => [1 startBit + 8 dataBits + 1 parityBit 1 + stopBit]

  switch (lora_settings_data.airDataRate) {
  case 0:
    LoRa.SetAirDataRate(ADR_19200);
    break;
  case 1:
    LoRa.SetAirDataRate(ADR_9600);
    break;
  case 2:
    LoRa.SetAirDataRate(ADR_4800);
    break;
  case 3:
    LoRa.SetAirDataRate(ADR_2400);
    break;
  }

  switch (lora_settings_data.txPower) {
  case 0:
    LoRa.SetTransmitPower(PWR_21dBm);
    break;
  case 1:
    LoRa.SetTransmitPower(PWR_24dBm);
    break;
  case 2:
    LoRa.SetTransmitPower(PWR_27dBm);
    break;
  case 3:
    LoRa.SetTransmitPower(PWR_30dBm);
    break;
  }

  LoRa.SaveParameters(PERMANENT); // Save parameters to LoRa's EEPROM
  //LoRa.PrintParameters();
  Serial.println("Configurazione LoRa aggiornata.");
  
}
