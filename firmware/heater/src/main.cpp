#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#include "dht22.h"

float Kp = 10;
float Ki = 0.3;
float Kd = 3;

float Ki_time = 0;
float Kd_time = 0;

#define MAX_PWM 255

uint16_t calculatePID(uint16_t target, uint16_t is){
	static uint16_t oldError = 0;
	static float iterm = 0;

	int16_t error = target - is;

	float pterm = Kp * error;
	float dterm = Kd_time * (int16_t)((int16_t)error - (int16_t)oldError);

	if(((pterm + iterm + dterm) < MAX_PWM) && ((pterm + iterm + dterm) > -MAX_PWM)){
		iterm += (Ki_time * error);
	}

	if(error > ::abs(20)){ //set I to 0 if error is too big
		iterm = 0;
	}

	float output = pterm + iterm + dterm;

	if(output > MAX_PWM) output = MAX_PWM;
	if(output < 0) output = 0;

	oldError = error;

	return (uint16_t)output;
}

int main() {
	dht22_t dht_sensor;
	dht_init(&dht_sensor);
	uint32_t time = 0;
	while(42) {
		dht_update(&dht_sensor, time);
		uint16_t target = dht_get_dew_point(&dht_sensor);
		calculatePID(target, 20);
		++time;
	}
}
