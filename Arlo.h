#ifndef ARLO_H
#define ARLO_H

//#define SIMULATE_ENCODER

// Pins
// Interrupt 0 is on pin 2
// Interrupt 1 is on pin 3
#define LEFT_HB25_CONTROL 4 
#define RIGHT_HB25_CONTROL 5

#ifdef SIMULATE_ENCODER
#define LEFT_ENCODER_PIN = 6;
#define RIGHT_ENCODER_PIN = 7;
#endif

// Test LED
#define TEST_LED 8

#define SPEED_100	2000					// * 1 = 100 percent duty cycle (Al)
#define SPEED_75	1875					// * 3 = 75 percent duty cycle (75uS)
#define SPEED_50	1750					// * 2 = 50 percent duty cycle (50uS)
#define SPEED_25	1625					// * 1 = 25 percent duty cycle (25uS)
#define SPEED_0	        1500      				// * 0 = 0 percent duty cycle (Always low)

#define METRIC_CIRCUMFERENCE 478779  // Wheel circumference in mm * 1000. 6inch wheels * PI =18.85"
#define METRIC_SPEED 1000
#define METRIC_WHEEL_BASE (float)213
#define MAX_METRIC_SPEED 500        // In mm/sec
#define RADIAN_CONVERSION 0.0174533
#define MM_PER_RADIAN 76.2

#define MM_PER_TICK 13299408
#define SERVO_ZERO 1500
#define MIN_SERVO 1000
#define MAX_SERVO 2000
#endif
