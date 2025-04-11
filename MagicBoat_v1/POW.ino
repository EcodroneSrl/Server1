uint32_t time_power_sel = 0;
uint16_t timer_power_sel = 10000;
#define INDEX_POWER_RUT955 5


void powerInit() {
  powerSel.begin();
  powerSelectSet();
}

void powerSelectSet() 
{
  for (uint8_t i = 0; i < 16; i++) 
  {
    if (power_en[i] == PWR_ON) 
    {
      powerSel.write(i, HIGH);
    } else 
    {
      powerSel.write(i, LOW);
    }
  }
      if(power_en[INDEX_POWER_RUT955] == 0)
    {
      delay(5000);
      powerSel.write(INDEX_POWER_RUT955, HIGH);
      power_en[INDEX_POWER_RUT955] = 1;
    }
}
void powerSelectAllOff()
  {
      for (uint8_t i = 0; i < 16; i++) 
      {
      powerSel.write(i, LOW);
      }
  }
void computePower()
{
  if((millis()-time_power_sel)>timer_power_sel)
  {
    time_power_sel = millis();
    powerSelectSet();
  }
}
