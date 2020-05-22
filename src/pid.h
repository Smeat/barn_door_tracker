#ifndef __PID_H_
#define __PID_H_

#include <stdint.h>

#include "config.h"

typedef struct pid_state_s {
	// constantly changing variables
	uint16_t oldError = 0;
	float iterm = 0;
	// precalculated values
	float kd_time = 0;
	float ki_time = 0;

	// actual pid
	float Kp = DEFAULT_P;
	float Ki = DEFAULT_I;
	float Kd = DEFAULT_D;
	uint32_t last_micros = 0;
	uint32_t pwm_output_pin = 0;
	void (*get_values)(uint16_t* target, uint16_t* is);
} pid_state_t;

void update_pid_time(pid_state_t* pid, uint32_t time_us);
uint16_t calculatePID(pid_state_t* pid, uint16_t target, uint16_t is);

#endif // __PID_H_
