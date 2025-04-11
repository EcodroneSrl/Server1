uint8_t bufferDeb[255];
uint16_t indexByteSendDeb = 0;

void Serialprint(char *car) {
  costructBuffDeb(car, strlen(car));
}

void Serialprint(const char *car) {
  costructBuffDeb((char *)car, strlen(car));
}

/*
void SerialprintS(char *car)
{
  if(enInfoDeb)
  {
  costructBuffDeb(car,strlen(car));
  }
  else{Serialprint(F(";"));}
}
*/

void Serialprint(float flo, uint8_t prec) {
  char x[20];
  //prec = prec + 3;
  dtostrf(flo, 4, prec, x);
  costructBuffDeb(&x, prec);
}

void Serialprint(float flo) {
  char x[20];
  dtostrf(flo, 4, 6, x);
  costructBuffDeb(&x, 6);
}

void Serialprint(double doub, uint8_t prec) {
  char x[20];
  //prec = prec + 3;
  dtostrf(doub, 4, prec, x);
  costructBuffDeb(&x, prec);
}

void Serialprint(double doub) {
  char x[20];
  dtostrf(doub, 4, 6, x);
  costructBuffDeb(&x, 6);
}

void Serialprint(uint32_t num) {
  char x[20];
  itoa(num, x, 10);
  costructBuffDeb(&x, strlen(x));
}
void Serialprint(uint16_t num) {
  char x[20];
  itoa(num, x, 10);
  costructBuffDeb(&x, strlen(x));
}
void Serialprint(uint8_t num) {
  char x[20];
  itoa(num, x, 10);
  costructBuffDeb(&x, strlen(x));
}

void Serialprint(int32_t num) {
  char x[20];
  itoa(num, x, 10);
  costructBuffDeb(&x, strlen(x));
}

void Serialprint(int16_t num) {
  char x[20];
  itoa(num, x, 10);
  costructBuffDeb(&x, strlen(x));
}

void Serialprint(int8_t num) {
  char x[20];
  itoa(num, x, 10);
  costructBuffDeb(&x, strlen(x));
}

void Serialprint(int num) {
  char x[20];
  itoa(num, x, 10);
  costructBuffDeb(&x, strlen(x));
}

void Serialprintln(char string) {
  costructBuffDeb(&string, strlen(&string));
  sendBuffDeb();
}
void Serialprintln(char *string) {
  costructBuffDeb(string, strlen(string));
  sendBuffDeb();
}
void Serialprintln(void) {
  sendBuffDeb();
}

uint8_t dim = 0;
uint8_t ifor = 0;

void Serialprint(const __FlashStringHelper *ifsh) {
  char x[64];
  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  dim = strlen(p);
  for (ifor = 0; ifor < dim; ifor++) {
    ((unsigned char *)x)[ifor] = ((unsigned char *)p)[ifor];
  }
  x[dim] = '\0';
  costructBuffDeb(x, strlen(x));
}

void Serialprintln(const __FlashStringHelper *ifsh) {
  Serialprint(F(ifsh));
  sendBuffDeb();
}
void costructBuffDeb(void *ptr1, uint16_t lenVar) {
  for (uint16_t i = 0; i < lenVar; i++) {
    bufferDeb[indexByteSendDeb] = ((unsigned char *)ptr1)[i];
    indexByteSendDeb++;
  }
}
void sendBuffDeb() {
  //costructBuffDeb(final,strlen(final));
  bufferDeb[indexByteSendDeb] = '\r';
  indexByteSendDeb++;
  bufferDeb[indexByteSendDeb] = '\n';
  indexByteSendDeb++;
  // bufferDeb[indexByteSendDeb] = '\0';
  // indexByteSendDeb++;

  Serial.write(bufferDeb, indexByteSendDeb);

  if (socketActive) {
    SerialRWSocket.client.write(bufferDeb, indexByteSendDeb);
  }

  indexByteSendDeb = 0;
}