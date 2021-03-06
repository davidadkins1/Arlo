/*********************************************************************
 *
 *      Metal Detector Robot
 *
 *********************************************************************
 * FileName:        Compass.c
 * Processor:       Arduino Mega 2560 AVR
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       03/02/12    Original.
 ********************************************************************/

/** I N C L U D E S **********************************************************/
// Reference the I2C Library
#include <Wire.h>
// Reference the HMC5883L Compass Library
#include <HMC5883L.h>
#include "Arlo.h"

// Store our compass as a variable.
HMC5883L compass;

/** Defines ********************************************************/
#define WRITE_DATA  0x3C
#define READ_DATA   0x3D
#define MODE        0x02
#define X_MSB       0x03

/** Global Variables ********************************************************/
extern SENSORS sensorPacket;

/** Local Variables ********************************************************/
#pragma udata
static unsigned long compass_timer;
static WORD_VAL x;
static WORD_VAL y;
static WORD_VAL z;
static int heading_degrees;
static int last_heading_degrees;
MagnetometerScaled scaled;
float azimuth;

/*-----------------------------------------------------------*/
#pragma code
void init_compass(void)
{
  // Record any errors that may occur in the compass.
  int error = 0;
  Wire.begin(); // Start the I2C interface.
  heading_degrees = 0;
  last_heading_degrees = 0xffffu;  // Flag for first update
  error = compass.setScale(1); // Set the scale of the compass.
  #ifdef SERIAL_DEBUG
  if(error != 0) // If there is an error, print it out.
  {
    DebugPort.println(compass.getErrorText(error));
  }
  #endif
  // Setting measurement mode to continous.
  error = compass.setMeasurementMode(MEASUREMENT_CONTINUOUS); // Set the measurement mode to Continuous
  #ifdef SERIAL_DEBUG
  if(error != 0) // If there is an error, print it out.
  {
    DebugPort.println(compass.getErrorText(error));
  }
  #endif
  compass_timer = set_timer(100);
}


void get_raw_reading(void)
{
  // Retrived the scaled values from the compass (scaled to the configured scale).
  scaled = compass.readScaledAxis();
}

void convert_azimuth(void)
{
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -2ï¿½ï¿½37' which is -2.617 Degrees, or (which we need) -0.0456752665 radians, I will use -0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = DECLINATION_CORRECTION;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += THREE_SIXTY_DEGREES;
    
  // Check for wrap due to addition of declination.
  if(heading > THREE_SIXTY_DEGREES)
    heading -= THREE_SIXTY_DEGREES;
   
  // Convert radians to degrees for readability.
  heading_degrees = heading * DEGREE_CONVERSION;
  // compass is mounted upside down
  //heading_degrees -= 360;
  //heading_degrees = -heading_degrees;
  
  //if(360 == heading_degrees)
  //{
  //  heading_degrees = 0;
  //}
}

void update_turn_angle(void)
{
  int turn_delta;
  WORD_VAL turn_delta_accumulator;
  DWORD_VAL turn_delta_test;

    // Update and return if first time through
    if(0xffffu == last_heading_degrees)
    {
        last_heading_degrees = heading_degrees;
    }
    
    // Calculate the heading delta
    if(last_heading_degrees == heading_degrees)
    {
        // Exit if no change in heading
        return;
    }
    
    // Counter-clockwise angles are positive and clockwise angles are negative.
    turn_delta = last_heading_degrees - heading_degrees;
    
    // check for zero cross
    if(turn_delta > 180)
    {
        turn_delta -= 360;
    }
    else if(turn_delta < -180)
    {
        turn_delta += 360;
    }
/*    
    // Protect from interrupts
    noInterrupts();    
    //Convert from big endian to little
    turn_delta_accumulator.byte.HB = sensorPacket.sensor.iAngleTraveled.byte.LB;
    turn_delta_accumulator.byte.LB = sensorPacket.sensor.iAngleTraveled.byte.HB;
    interrupts(); 
*/    
    // Convert turn accumulator to long for overflow testing
    turn_delta_test.sVal = turn_delta_accumulator.sVal + turn_delta;
     
    // Limit result to +32767 -32768
    if(turn_delta_test.sVal > 32767)
    {
        turn_delta_accumulator.sVal = 32767;
    }
    else if(turn_delta_test.sVal < -32768)
    {
        turn_delta_accumulator.sVal = -32768;
    }
    else
    {
        turn_delta_accumulator.byte.HB = turn_delta_test.byte.HB;
        turn_delta_accumulator.byte.LB = turn_delta_test.byte.LB;
    }
/*    
    #ifdef SERIAL_DEBUG
    DebugPort.print(heading_degrees);
    DebugPort.print(" ");
    DebugPort.print(last_heading_degrees);
    DebugPort.print(" ");
    DebugPort.println(turn_delta_accumulator.sVal);
    #endif
*/
 /*   
    noInterrupts();    
    //Convert from little endian to big
    sensorPacket.sensor.iAngleTraveled.byte.LB = turn_delta_accumulator.byte.HB;
    sensorPacket.sensor.iAngleTraveled.byte.HB = turn_delta_accumulator.byte.LB;
    interrupts(); 
*/    
    // Save current heading as last
    last_heading_degrees = heading_degrees;
}

void compass_task(void)
{
  if(time_out(compass_timer))
  {
    compass_timer = set_timer(66);
    get_raw_reading();
    convert_azimuth();
    
    if((MOTORS_STOPPED != LeftMotor->State) || (MOTORS_STOPPED != RightMotor->State))
    {
      update_turn_angle();
    }
  }
}

unsigned int current_heading(void)
{
    return heading_degrees;
}
/*end of module*/


