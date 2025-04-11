#ifndef MagicBoat_Media_h
#define MagicBoat_Media_h

#include "Arduino.h"

#define DEBUG_MAX_MIN	false

class MagicBoat_Media
{
	private:
	
	public:
	
	MagicBoat_Media();
	uint16_t index = 0;
	uint8_t put_end = 0;
	uint16_t num_values = 10;
	uint32_t time_values_int = 0;
	uint16_t timer_values_int = 10;
	
	uint32_t time_values_float = 0;
	uint16_t timer_values_float = 10;
	
	uint8_t putValues(float value);
	void computeMedia();
	float getMed();
	float getMin();
	float getMax();
	void setNum(uint16_t num);
	void resetCounter();
	uint8_t full();

	float Min = 999;
	float Max = -999;
	float Med = 0;
	float sum = 0;
	
};

#endif