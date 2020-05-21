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
#define MOTOR_STEPS_PER_REV 4096
#define EXTERNAL_GEAR_RATIO 4.3
#define DESIRED_MAIN_GEAR_RPM 1.09

#define REVERSE_RPM 15

#define SCREW_PITCH 0.8

#define CIRCLE_RADIUS_MM 200

#define MOTOR_IN1 PA0
#define MOTOR_IN2 PA1
#define MOTOR_IN3 PA2
#define MOTOR_IN4 PA3

#define ENDSTOP_PIN PA4
#define REVERSE_PIN PA5
#define START_PIN PA6

#ifdef USE_HEATER
#define TEMP_OUSIDE_PIN PB0
#define TEMP_HEATER_PIN PB1

#define TEMP_OUTSIDE_CONSTANT 1
#define TEMP_OUTSIDE_FACTOR 1
#define TEMP_HEATER_CONSTANT 1
#define TEMP_HEATER_FACTOR 1

#define HEATER_PIN PA7

#define PID_FREQUENCY_US 100000
#define MAX_PWM ((1 << PWM_RESOLUTION) - 1)

#define DEFAULT_P 10
#define DEFAULT_I 0.1
#define DEFAULT_D 5
#endif // USE_HEATER

// Calculations
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CALCULATED_MOTOR_RPM (DESIRED_MAIN_GEAR_RPM * EXTERNAL_GEAR_RATIO)
#define STEPS_PER_MM (int((MOTOR_STEPS_PER_REV * EXTERNAL_GEAR_RATIO) * SCREW_PITCH))
#define DEGREE_TO_MM(x) (2.0*M_PI*CIRCLE_RADIUS_MM*(x/360.0))
// steps until 90°
#define MAX_STEPS (int(DEGREE_TO_MM(90))*STEPS_PER_MM)
