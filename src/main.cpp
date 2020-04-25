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

void loop() {
	start_debounce.update();
	update();
}
