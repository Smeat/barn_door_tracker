#include "dht22.h"

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

static int is_checkum_valid(dht22_t* sensor) {
	return sensor->data[0] == ((sensor->data[1] + sensor->data[2] + sensor->data[3] + sensor->data[4]) & 0xFF);
}

void dht_init(dht22_t* sensor) {
	memset(sensor, 0, sizeof(dht22_t));
}

// using polling for now. maybe IC or INT later
void dht_update(dht22_t* sensor) {
	switch(sensor->state){
		case DHT_IDLE:
			// only update every 2 seconds
			if(micros() - sensor->last_update > 2000000) {
				pinMode(sensor->pin, OUTPUT);
				digitalWrite(sensor->pin, HIGH);
				sensor->state = DHT_REQUESTING;
				sensor->last_update = micros();
			}
			break;
		case DHT_REQUESTING:
			if(micros() - sensor->last_update > DHT_MIN_HIGH_US) {
				pinMode(sensor->pin, INPUT_PULLUP);
				sensor->state = DHT_WAITING_HEADER;
				sensor->last_update = micros();
			}
			break;
		case DHT_WAITING_HEADER:
			if(sensor->cycles_wait == 0) {
				if(digitalRead(sensor->pin) == 0) {
					sensor->cycles_wait += 1;
				}
			} else if(sensor->cycles_wait == 1) {
				if(digitalRead(sensor->pin) == 1) {
					sensor->cycles_wait += 1;
				}
			} else if(sensor->cycles_wait == 2) {
				if(digitalRead(sensor->pin) == 1) {
					sensor->cycles_wait = 0;
					sensor->state = DHT_WAITING_DATA;
					// Init sequence done
				}
			}
		break;
		case DHT_WAITING_DATA:
			{
				uint8_t val = digitalRead(sensor->pin);
				if(val) {
					sensor->cycles_data += 1;
				} else {
					if(sensor->cycles_data > 0) {
						// next bit
						sensor->temp_data[0] <<=1;
						if(sensor->cycles_wait < sensor->cycles_data) {
							sensor->temp_data[0] |= 1;
						}
						sensor->bits_read += 1;
						sensor->cycles_wait = 0;
						sensor->cycles_data = 0;
					}
					sensor->cycles_wait += 1;
				}
				if(sensor->bits_read == 40) {
					// data complete
					if(is_checkum_valid(sensor)) {
						memcpy(sensor->data, sensor->temp_data, 5);
						sensor->bits_read = 0;
						sensor->state = DHT_IDLE;
						sensor->last_update = micros();
					}
				}
			}
			break;
		default:
			break;
	}
}

int16_t dht_get_temperature(dht22_t* sensor) {
	return sensor->data[2] << 8 | sensor->data[1];
}

int16_t dht_get_humidity(dht22_t* sensor) {
	return sensor->data[4] << 8 | sensor->data[3];
}

int16_t dht_get_dew_point(dht22_t* sensor) {
	// +-1°C accurate for humidity > 50%. A more accurate formula includes ln, so we might want to avoid that
	// with < 50% there shouldn't be problems anyways
	return dht_get_temperature(sensor) - 1000* ((100 - dht_get_humidity(sensor))/5);
}
