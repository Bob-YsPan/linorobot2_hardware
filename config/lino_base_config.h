// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define LED_PIN 13 //used for debugging status

//uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER            // 4WD robot
// #define LINO_BASE MECANUM               // Mecanum drive robot

//uncomment the motor driver you're using
// define USE_GENERIC_2_IN_MOTOR_DRIVER      // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
// #define USE_GENERIC_1_IN_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
// ** 2025/03/06 We use BTS7960
#define USE_BTS7960_MOTOR_DRIVER        // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER            // Motor ESC for brushless motors

//uncomment the IMU you're using
// #define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// ** 2025/03/06 We have MPU9250 on the board
// ** 2025/03/31 IMU ignored, cloned forked repo
// 250501 Remove on board imu and test
//#define USE_MPU9250_IMU

// ** 2025/03/06 Default PID is 0.6 0.8 0.5, this is the value of our car
#define K_P 0.6                             // P constant
#define K_I 0.3                             // I constant
#define K_D 0.5                             // D constant

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

//define your robot' specs here
// ** 2025/03/06 Default max 140rpm, this is the value of our car
// ** 2025/03/12 Actual spec: Max 159.375rpm (after gearbox)
#define MOTOR_MAX_RPM 159.375                   // motor's max RPM          
#define MAX_RPM_RATIO 0.85                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 24          // motor's operating voltage (used to calculate max RPM)
// ** 2025/03/12 The value is motor at 24v, so use this
#define MOTOR_POWER_MAX_VOLTAGE 24          // max voltage of the motor's power source (used to calculate max RPM)
// ** 2025/03/13 I already know the ticks of the motor, so use just same as the power voltage
#define MOTOR_POWER_MEASURED_VOLTAGE 24     // current voltage reading of the power connected to the motor (used for calibration)
// ** 2025/03/06 Default 144000 tick, this is the value of our car
// ** 2025/03/13 Calculated: 1200 pulses/rev -> will do 4 step: A+, B+, A-, B- in about one pulse's time, so need x4 = 4800
#define COUNTS_PER_REV1 4800              // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 4800              // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 4800              // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 4800              // wheel4 encoder's no of ticks per rev
// ** 2025/03/06 Default 0.152m, this is the value of our car: observe method: https://baike.pcauto.com.cn/356655.html
#define WHEEL_DIAMETER 0.355                // wheel's diameter in meters
// ** 2025/03/06 Default 0.271m, this is the value of our car(56.5cm)
#define LR_WHEELS_DISTANCE 0.565            // distance between left and right wheels
// ** 2025/03/06 Default 10bit, the original value of our car is 8 bit, higher seems not work
#define PWM_BITS 8                          // PWM Resolution of the microcontroller
// ** 2025/03/06 Can look https://www.pjrc.com/teensy/td_pulse.html to improve performance, 
// ** Or just keep default (20000)
// Teensy 3.1 = 96 MHz, 11718.75 Hz at 12 bit or 187500 at 8 bit (Too high!, limit is 25k)
#define PWM_FREQUENCY 18750                 // PWM Frequency

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV false 
#define MOTOR2_ENCODER_INV false 
#define MOTOR3_ENCODER_INV false 
#define MOTOR4_ENCODER_INV false 

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

// ENCODER PINS
// ** 2025/03/06 orig: 1 = 14 15; 2 = 11 12
// ** 2025/03/11 swap the like for Motor 2 >> so 15, 14 to 14, 15
#define MOTOR1_ENCODER_A 11
#define MOTOR1_ENCODER_B 12 

#define MOTOR2_ENCODER_A 14
#define MOTOR2_ENCODER_B 15 

#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16 

#define MOTOR4_ENCODER_A 9
#define MOTOR4_ENCODER_B 10

// MOTOR PINS
#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x, you can swap it with pin no 1 instead.
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B 1 

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B 8

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B 0

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 3
  #define MOTOR4_IN_B 2

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef USE_GENERIC_1_IN_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 instead.
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 3
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

// ** 2025/03/06 For our car's driver
// ** Orig: Motor 1 = 21 20; Motor 2 = 5 6
#ifdef USE_BTS7960_MOTOR_DRIVER
  #define MOTOR1_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 5 // Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 instead.
  #define MOTOR1_IN_B 6 // Pin no 20 is not a PWM pin on Teensy 4.x, you can use pin no 0 instead.

  #define MOTOR2_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 9
  #define MOTOR2_IN_B 10

  #define MOTOR3_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23

  #define MOTOR4_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 4
  #define MOTOR4_IN_B 3

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#ifdef USE_ESC_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x. You can use pin no 1 instead.
  #define MOTOR1_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR3_PWM 22 
  #define MOTOR3_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define PWM_MAX 400
  #define PWM_MIN -PWM_MAX
#endif

#endif
