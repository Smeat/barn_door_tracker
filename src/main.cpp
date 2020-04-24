#include <Arduino.h>
#include <CheapStepper.h>
#include <Bounce2.h>

#define MOTOR_STEPS_PER_REV 4096
#define EXTERNAL_GEAR_RATIO 4.3
#define DESIRED_MAIN_GEAR_RPM 1.09

#define CALCULATED_MOTOR_RPM (DESIRED_MAIN_GEAR_RPM * EXTERNAL_GEAR_RATIO)

#define REVERSE_RPM 20

#define SCREW_PITCH 0.8
#define STEPS_PER_MM ((MOTOR_STEPS_PER_REV * EXTERNAL_GEAR_RATIO) * SCREW_PITCH)

#define CIRCLE_RADIUS_MM 200
#define DEGREE_TO_MM (x) (2*M_PI*CIRCLE_RADIUS_MM*(x/360))
#define MAX_STEPS DEGREE_TO_MM(90)

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

void setup() {
	pinMode(PC13, OUTPUT);
	digitalWrite(PC13, LOW);

	pinMode(ENDSTOP_PIN, INPUT_PULLUP);
	pinMode(REVERSE_PIN, INPUT_PULLUP);
	start_debounce.attach(START_PIN, INPUT_PULLUP);
}
uint32_t last_micros = 0;

void set_idle() {
	stepper.stop();
	stepper.off();
	current_state = IDLE;
}

void update() {
	if(current_state == IDLE) {
		if(start_debounce.rose()) {
			if(digitalRead(REVERSE_PIN)) {
				stepper.setRpm(REVERSE_RPM);
				stepper.newMoveTo(false, -MAX_STEPS); // Endstop will stop the motor
				current_state = REVERSING;
			} else {
				stepper.setRpm(CALCULATED_MOTOR_RPM);
				stepper.newMoveTo(true, MAX_STEPS);
				current_state = TRACKING;
			}
		}
	} else if(current_state == TRACKING || current_state == REVERSING) {
		stepper.run();
		if(start_debounce.rose()) {
			set_idle();
		}
	}
	if(digitalRead(ENDSTOP_PIN)) {
		set_idle();
	}
}

void loop() {
	update();
}
