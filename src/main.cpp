/*
 * Copyright (c) 2020 smeat.
 *
 * This file is part of barn_door_tracker 
 * (see https://github.com/Smeat/barn_door_tracker).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#include <Arduino.h>
#include <CheapStepper.h>
#include <Bounce2.h>
#include <cstdint>
#include <stdlib.h>

#include "config.h"

CheapStepper stepper(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4);
Bounce start_debounce = Bounce();

enum STATE {
	IDLE,
	TRACKING,
	REVERSING,
	STOPPED
};

int current_state = IDLE;
uint32_t last_micros = 0;
int32_t absolute_pos = 0;

#ifdef USE_HEATER
typedef struct temp_sensor_s {
	uint32_t pin;
	float factor;
	float constant;
} temp_sensor_t;

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

temp_sensor_t temp_outside = {TEMP_OUSIDE_PIN, TEMP_OUTSIDE_FACTOR, TEMP_OUTSIDE_CONSTANT};
temp_sensor_t temp_heater = {TEMP_HEATER_PIN, TEMP_HEATER_FACTOR, TEMP_HEATER_CONSTANT};
pid_state_t heater_pid; // TODO: init and load from eeprom
void update_pid_time(pid_state_t* pid, uint32_t time_us) {
	float time_s = time_us * 1e6;
	pid->ki_time = pid->Ki * time_s;
	pid->kd_time = pid->Kd / time_s;
}

// temperture in m°C
uint16_t read_temperature(temp_sensor_t* sensor) {
	uint16_t temperature = analogRead(sensor->pin) * sensor->factor + sensor->constant;
	return temperature;
}

void get_heater_values(uint16_t* target, uint16_t* is) {
	*target = read_temperature(&temp_outside) + 2;
	*is = read_temperature(&temp_heater);
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
#endif // USE_HEATER

void setup() {
	Serial1.begin(115200);
	pinMode(PC13, OUTPUT);
	digitalWrite(PC13, HIGH);

	pinMode(ENDSTOP_PIN, INPUT_PULLUP);
	pinMode(REVERSE_PIN, INPUT_PULLUP);
	start_debounce.attach(START_PIN, INPUT_PULLUP);
	start_debounce.interval(25);
	Serial1.printf("Starting with %d max_steps\n", MAX_STEPS);
	stepper.setTotalSteps(MOTOR_STEPS_PER_REV);
#ifdef USE_HEATER
	pinMode(TEMP_HEATER_PIN, INPUT);
	pinMode(TEMP_OUSIDE_PIN, INPUT);
	heater_pid.get_values = get_heater_values;
	heater_pid.pwm_output_pin = HEATER_PIN;
#endif // USE_HEATER
}


void set_idle() {
	stepper.off();
	Serial1.printf("Idling with pos %d and steps left %d\n", absolute_pos, stepper.getStepsLeft());
	// adjust current position
	absolute_pos -= stepper.getStepsLeft();
	Serial1.printf("New pos is %d\n", absolute_pos);
	stepper.stop();
	current_state = IDLE;
}

void update() {
	if(current_state == IDLE) {
		if(start_debounce.fell()) {
			int steps = MAX_STEPS - absolute_pos;
			if(!digitalRead(REVERSE_PIN)) {
				stepper.setRpm(REVERSE_RPM);
				stepper.newMove(false, steps); // Endstop will stop the motor
				absolute_pos -= steps;
				current_state = REVERSING;
				Serial1.printf("Reversing with pos %d and steps left %d...\n", absolute_pos, stepper.getStepsLeft());
			} else {
				stepper.setRpm(CALCULATED_MOTOR_RPM);
				stepper.newMove(true, steps);
				absolute_pos += steps;
				current_state = TRACKING;
				Serial1.printf("Tracking with pos %d and steps left %d...\n", absolute_pos, stepper.getStepsLeft());
			}
		}
	} else if(current_state == TRACKING || current_state == REVERSING) {
		stepper.run();
		if(start_debounce.fell()) {
			set_idle();
			Serial1.printf("Stopping with pos %d and steps left %d\n", absolute_pos, stepper.getStepsLeft());
		}
	}
	if(!digitalRead(ENDSTOP_PIN)) {
		Serial1.println("Endstop hit!!");
		set_idle();
		absolute_pos = 0;
	}
}

#ifdef USE_HEATER
void update_pid(pid_state_t* pid) {
	uint16_t is = 0;
	uint16_t target = 0;
	pid->get_values(&target, &is);
	uint16_t output = calculatePID(pid, target, is);
	analogWrite(pid->pwm_output_pin, output);
}
#endif // USE_HEATER

void loop() {
	start_debounce.update();
	update();
#ifdef USE_HEATER
	if(micros() - heater_pid.last_micros > PID_FREQUENCY_US) {
		update_pid(&heater_pid);
		heater_pid.last_micros = micros();
	}
#endif // USE_HEATER
}
