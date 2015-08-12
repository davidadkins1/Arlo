#include "ITG3200.h"

ITG3200 gyro;
#define SAMPLE_RATE       20
#define CONVERSION_OFFSET 512
#define CONVERSION_RANGE  250

static unsigned long gyro_timer;
float gyro_rate_monitor;
static float gyro_heading_degrees;
static unsigned long last_gyro_read_time; 

static float gyro_sensitivity;
static float rotation_threshold;   //Minimum deg/sec to keep track of - helps with gyro
static float rotation_accumulator;         // this is needed to avoid accumulating rounding error.

void init_gyro(void)
{
  delay(50);
  gyro.init();
  gyro.zeroCalibrate(200,10);     //sample 200 times to calibrate and it will take 200*10ms
  gyro_timer = set_timer(SAMPLE_RATE);
  last_gyro_read_time = millis();

  // An absolute heading can be maintained.
  // But this is relative to the startup facing.
  gyro_heading_degrees = 0;
  gyro_sensitivity = 14.375;
  rotation_threshold = 1;          //Minimum deg/sec to keep track of - helps with gyro 
  rotation_accumulator = 0;
}

void gyro_task(void)
{
  if(time_out(gyro_timer))
  {
    int16_t x;
    int16_t y;
    int16_t z;
    WORD_VAL gyro_t;

    gyro.getXYZ(&x, &y, &z);
    unsigned long gyro_read_time = millis();

    // Only process the gyro if the motors are running
    if( true == motors_running() )
    {
      // Convert z rotation to degress per second.
      float gyro_rate = z / gyro_sensitivity;   // Calculate degrees per second
      gyro_rate_monitor = gyro_rate;            // Save a copy.
      
      // Convert the degress per second rate to the turtlebot analog equivalent
      // 250 degrees/sec centered around 512.
      float gyro_conversion = gyro_rate / CONVERSION_RANGE;
      gyro_conversion *= CONVERSION_OFFSET;
      gyro_conversion += CONVERSION_OFFSET;
      gyro_t.sVal = (int)gyro_conversion;
      
      //Ignore the gyro if our angular velocity does not meet our threshold
      if (gyro_rate >= rotation_threshold || gyro_rate <= -rotation_threshold)
      {        
        //This line calculates degrees motion over the timespan  (1000ms/timespan)
        gyro_rate /= (1000 / (gyro_read_time - last_gyro_read_time));
  
        // An absolute heading can be maintained.
        gyro_heading_degrees += gyro_rate;
  
        // left spin should be positive so a sign change is required
        gyro_rate *= -1;
        noInterrupts();
        // an accumlator is used for the sensor packet 
        rotation_accumulator += gyro_rate;
        interrupts();
      }
  
      //Keep our heading angle between 0-359 degrees
      if(gyro_heading_degrees < 0)
      {
        gyro_heading_degrees += 360;
      }
      else if(gyro_heading_degrees > 359)
      {
        gyro_heading_degrees -= 360;
      }
      
      #ifdef SERIAL_DEBUG
      //DebugPort.print("Gyro value of Z: ");
      //DebugPort.print(z);
      //DebugPort.print(" , ");
      //DebugPort.println(gyro_t.sVal);
      #endif
      
      noInterrupts();
      //Convert from little endian to big
      sensorPacket.sensor.uiAnalogSignal.byte.LB = gyro_t.byte.HB;
      sensorPacket.sensor.uiAnalogSignal.byte.HB = gyro_t.byte.LB;
      interrupts();
    }
    else  // motors are stopped
    {
      noInterrupts();
      sensorPacket.sensor.uiAnalogSignal.Val = 2; // 512(0 rotation) in big endian world
      rotation_accumulator = 0;
      interrupts();
    }
    
    last_gyro_read_time = gyro_read_time;
    gyro_timer = set_timer(SAMPLE_RATE);
  } //gyro timeout
}

unsigned int current_gyro_heading(void)
{
    return gyro_heading_degrees;
}

void read_current_rotatation(void)
{
    WORD_VAL rotation_delta;
    
    noInterrupts();
    rotation_delta.sVal = (int)rotation_accumulator;

    //subratct the value instead of zeroing to help with rounding errors
    rotation_accumulator -= (float)rotation_delta.sVal;
    
    //Convert from little endian to big
    sensorPacket.sensor.iAngleTraveled.byte.LB = rotation_delta.byte.HB;
    sensorPacket.sensor.iAngleTraveled.byte.HB = rotation_delta.byte.LB;
    interrupts();
}
  
