#ifndef __DHT_22_H_
#define __DHT_22_H_

#include <stdint.h>

#define DHT_MIN_HIGH_US 2000
enum dht22_state {
	DHT_IDLE,
	DHT_REQUESTING,
	DHT_WAITING_HEADER,
	DHT_WAITING_DATA
};

typedef struct dht22_s {
	int state;
	uint32_t pin;
	uint32_t last_update;
	uint32_t cycles_wait;
	uint32_t cycles_data;
	uint8_t data[5];
	uint8_t temp_data[5];
	uint8_t bits_read;
} dht22_t;

void dht_init(dht22_t* sensor);
void dht_update(dht22_t* sensor);
int16_t dht_get_dew_point(dht22_t* sensor);

#endif // __DHT_22_H_
