#include "HardwareSerial.h"
#include <Arduino.h>
#include <CheapStepper.h>
#include <Bounce2.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MOTOR_STEPS_PER_REV 4096
#define EXTERNAL_GEAR_RATIO 4.3
#define DESIRED_MAIN_GEAR_RPM 1.09

#define CALCULATED_MOTOR_RPM (DESIRED_MAIN_GEAR_RPM * EXTERNAL_GEAR_RATIO)

#define REVERSE_RPM 10

#define SCREW_PITCH 0.8
#define STEPS_PER_MM (int((MOTOR_STEPS_PER_REV * EXTERNAL_GEAR_RATIO) * SCREW_PITCH))

#define CIRCLE_RADIUS_MM 200
#define DEGREE_TO_MM(x) (2.0*M_PI*CIRCLE_RADIUS_MM*(x/360.0))
#define MAX_STEPS (int(DEGREE_TO_MM(90))*STEPS_PER_MM)

#define MOTOR_IN1 PA0
#define MOTOR_IN2 PA1
#define MOTOR_IN3 PA2
#define MOTOR_IN4 PA3

#define ENDSTOP_PIN PA4
#define REVERSE_PIN PA5
#define START_PIN PA6

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
