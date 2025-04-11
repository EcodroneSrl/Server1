int flow_frequency;  // Measures flow sensor pulsesunsigned

float l_hour;  // Calculated litres/hour
unsigned long currentTime;
unsigned long cloopTime;

TimerOne timerFlow;

void coolInit() {
  pinMode(pinPwmCool, OUTPUT);
  pinMode(pinPwmFlow, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinPwmFlow), flow, RISING);  // Setup Interrupt
  timerFlow.initialize(1000000);
  //sei(); // Enable interrupts
  currentTime = millis();
  cloopTime = currentTime;
}

uint32_t coolTime = 0;
void computeCool() {
  coolTime = millis();
  if (millis() - time_cool > timer_cool) {
    time_cool = millis();
    setCoolPwm(pwmCool);
  }
  readFlow();
  coolTime = millis() - coolTime;
}

void setCoolPwm(uint8_t pwm) {
  analogWrite(pinPwmCool, pwm);
}

void flow()  // Interrupt function
{
  flow_frequency++;
}

float kFlow = 20.0 / 60.0;

void readFlow() {
  currentTime = millis();
  // Every second, calculate and print litres/hour
  if (currentTime >= (cloopTime + 1000)) {
    cloopTime = currentTime;  // Updates cloopTime
    // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
    l_hour = (flow_frequency)*kFlow;  // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour
    // Serial.print(l_hour, DEC); // Print litres/hour
    // Serial.println(" L/min");
    flow_frequency = 0;  // Reset Counter
  }
}