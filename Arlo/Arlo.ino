/*******************************************************************************
 * FileName:    Arlo.ino
 * Product:	ClipBoardBot
 * Processor:   Arduino Mega 2560 AVR
 * Purpose:	Main setup and loop code
 *
 * (c)2013 David Adkins
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Revision History: 
 *
 * Revision 0.1 09/13 by David Adkins: 
 * 		1. Creation of code base
 ******************************************************************************/
#include "iocfg.h"
#include "Arlo.h"
#include <Servo.h>
#include "HB25_motor.h"

//#include "behaviors.h"

// the setup routine runs once when you press reset:
void setup () 
{
  motor_initialize();
  led_initialize();
  uartio_intialize();
  //init_compass();
  init_gyro();

  #if 0
  SonarServo.attach(53);
  SonarServo.write(90);
  #endif
  
  // A relay switches the Kinect power
  // Initialize it to off
  pinMode(KINECT_POWER, OUTPUT);
  digitalWrite(KINECT_POWER, LOW);
}

// the loop routine runs over and over again forever:
void loop () 
{
  vMotorMonitor();
  
  #if 0
  vSonarIO();
  #endif
  
  vUARTRxControl();
  vArbitrate();
  //compass_task();
  gyro_task();
  led_blink_task();
}


