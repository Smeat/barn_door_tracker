#include "pid.h"

#include <stdint.h>


void update_pid_time(pid_state_t* pid, uint32_t time_us) {
	float time_s = time_us * 1e6;
	pid->ki_time = pid->Ki * time_s;
	pid->kd_time = pid->Kd / time_s;
}

uint16_t calculatePID(pid_state_t* pid, uint16_t target, uint16_t is){
	int16_t error = target - is;

	float pterm = pid->Kp * error;
	float dterm = pid->kd_time * (error - pid->oldError);

	if(((pterm + pid->iterm + dterm) < MAX_PWM) && ((pterm + pid->iterm + dterm) > -MAX_PWM)){
		pid->iterm += (pid->ki_time * error);
	}

	if(error > abs(20)){ //set I to 0 if error is too big
		pid->iterm = 0;
	}

	float output = pterm + pid->iterm + dterm;

	if(output > MAX_PWM) output = MAX_PWM;
	if(output < 0) output = 0;

	pid->oldError = error;

	return (uint16_t)output;
}
