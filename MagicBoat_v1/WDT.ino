struct __attribute__((packed)) wdt_str {
  uint8_t wdt_alarm_cmd_timer = 14;
  uint8_t wdt_timeout = 15;
} wdt;

void wdtAlarmCmd() {
  for (int i = 0; i < 25; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  Serial.println(F("\nWATCHDOG TIMER NEAR TIMEOUT!"));
}

void wdtInit() {
  WDT_timings_t config;
  config.trigger = wdt.wdt_alarm_cmd_timer; /* in seconds, 0->128 */
  config.timeout = wdt.wdt_timeout;         /* in seconds, 0->128 */
  config.callback = wdtAlarmCmd;
  wdt1.begin(config);
}

void wdtReset() {
  wdt1.feed();
}
