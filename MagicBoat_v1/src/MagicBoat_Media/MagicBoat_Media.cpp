#include "MagicBoat_Media.h"

MagicBoat_Media::MagicBoat_Media() {
}

void MagicBoat_Media::setNum(uint16_t num) {
  num_values = num;
}

uint8_t MagicBoat_Media::putValues(float value) {
  if (!index) {
    put_end = 0;
  }
  if (index < num_values) {
    if (DEBUG_MAX_MIN) {
      Serial.print("Value:");
      Serial.print(value);
      Serial.print(",");
    }

    sum += value;
    // Max
    if (value > Max) {
      Max = value;
    }
    // Min
    if (value < Min) {
      Min = value;
    }
    if (DEBUG_MAX_MIN) {
      Serial.print("Max:");
      Serial.print(Max);
      Serial.print(",");
      Serial.print("Min:");
      Serial.println(Min);
    }
    index++;
  } else {
    computeMedia();
    put_end = 1;
    index = 0;
  }
  return put_end;
}

void MagicBoat_Media::computeMedia() {
  Med = (sum) / (num_values);
}

float MagicBoat_Media::getMed() {
  float act_med = Med;
  sum = 0;
  Med = 0;
  return act_med;
}

float MagicBoat_Media::getMax() {
  float act_max = Max;
  Max = -999;
  return act_max;
}

float MagicBoat_Media::getMin() {
  float act_min = Min;
  Min = 999;
  return act_min;
}