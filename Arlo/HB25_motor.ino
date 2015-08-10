/*******************************************************************************
 * FileName:    motor.ino
 * Product:	ClipBoardBot
 * Processor:   Arduino AVR
 * Purpose:	Motor control and arbitration. 
 *
 * (c)2013 David Adkins
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Revision History: 
 *
 * Revision 0.1 09/13 by David Adkins: 
 * 		1. Creation of code base
 ******************************************************************************/
#include <Servo.h>
#include "Arlo.h"
#include "HB25_motor.h"
#include "oi.h"
#include "iocfg.h"

/** C O N F I G U R A T I O N *************************************************/
#define ENCODER_TIMEOUT 200
//#define ENCODER_TEST
//#define ROTATION_TEST

//** M O T O R S ***********************************************************/

/** Global Variables ********************************************************/
//extern unsigned char g_script[48];
extern SENSORS sensorPacket;

/** Local Varibles ********************************************************/
//static int velocityLeft[MAX_BEHAVIOR];
//static int velocityRight[MAX_BEHAVIOR];

static struct MOTOR_CONTROL_REQUESTS motorControl[MAX_BEHAVIOR];
//static enum BEHAVIORS behaviorControl[MAX_BEHAVIOR];
static enum BEHAVIORS behaviorPriority[MAX_BEHAVIOR];

static xMotorControlParameters *LeftMotor;
static xMotorControlParameters *RightMotor;

//static unsigned char avgCount;
//static unsigned int motorCurrent;
//static unsigned long timeOut;
	
static enum ESCAPE_STATES escapeState = NORMAL_CRUISE;

//static long avg_emf;
//static unsigned long delta_time;
//static unsigned long current_velocity;
static DWORD_VAL current_distance;
static DWORD_VAL distance_delta_accumulator;

//Variables for RPM calculations        
static Servo left_motor;
static volatile boolean left_started;
static volatile unsigned long left_start_time;
static volatile unsigned long left_end_time;
static volatile unsigned long left_elapsed_time;
static volatile boolean left_timing_ready;
static unsigned long left_elapsed_time_mail;

static Servo right_motor;
static volatile boolean right_started;
static volatile unsigned long right_start_time;
static volatile unsigned long right_end_time;
static volatile unsigned long right_elapsed_time;
static volatile boolean right_timing_ready;
static unsigned long right_elapsed_time_mail;
//static uint16_t rpm;//revolutions per minute

//odometry
static volatile int left_odometer_counter;
static volatile int right_odometer_counter;
static WORD_VAL left_odometer_mail;
static WORD_VAL right_odometer_mail;
static WORD_VAL left_odometer;
static WORD_VAL right_odometer;
static long odometer;
static long distance_to_move;
float angle_accumulator;
float motion_accumulator;

/*-----------------------------------------------------------*/
static void motor_drive( struct MOTOR_CONTROL_REQUESTS *motor_parameters );
static void vMotorDriveLeft( int speed, unsigned long distance );
static void vMotorDriveRight( int speed, unsigned long distance );

// interrupt service routine
void left_encoder()
{
  left_end_time = micros();
  ++left_odometer_counter;
  
  if(left_started)
  {  
    left_elapsed_time = left_end_time - left_start_time;
    left_timing_ready = true;
  }

  left_start_time = left_end_time;
  left_started = true;  
} // end of shutter

// interrupt service routine
void right_encoder()
{
  right_end_time = micros();
  ++right_odometer_counter;
  
  if(right_started)
  {  
    right_elapsed_time = right_end_time - right_start_time;
    right_timing_ready = true;
  }

  right_start_time = right_end_time;
  right_started = true;  
} // end of shutter

void motor_initialize( void )
{
  left_motor.attach(LEFT_HB25_CONTROL);
  left_motor.writeMicroseconds(SERVO_ZERO);  // set servo to mid-point

  right_motor.attach(RIGHT_HB25_CONTROL);
  right_motor.writeMicroseconds(SERVO_ZERO);  // set servo to mid-point
  
  left_started = false;
  left_timing_ready = false;
  attachInterrupt (0, left_encoder, RISING);
  
  right_started = false;
  right_timing_ready = false;
  attachInterrupt (1, right_encoder, RISING);
  
  left_odometer_counter = 0;
  right_odometer_counter = 0;
  left_odometer_mail.Val = 0;
  right_odometer_mail.Val = 0;
  left_odometer.Val = 0;
  right_odometer.Val = 0;
  
  // Next setup the motor current measurement

  // Behavior initialization
  behaviorPriority[0] = REMOTE_CONTROL;
  behaviorPriority[1] = ESCAPE;
  behaviorPriority[2] = CRUISE;

  for( int i = NO_CONTROL_REQUEST; i < MAX_BEHAVIOR; i++ )
  {
    motorControl[i].BehaviorControl = NO_CONTROL_REQUEST;			// No behavior asking for control
    motorControl[i].left_motor.RequestedSpeed = SPEED_0;
    motorControl[i].left_motor.RequestedDistance = 0;
    motorControl[i].right_motor.RequestedSpeed = SPEED_0;
    motorControl[i].right_motor.RequestedDistance = 0;
  }

  // Create the structures used to pass parameters to the left motor task.
  LeftMotor = ( xMotorControlParameters * ) malloc( sizeof( xMotorControlParameters ) );
	
  if( LeftMotor != NULL )
  {
    LeftMotor->Select = LEFT_MOTOR;
    LeftMotor->State = MOTORS_STOPPED;
    LeftMotor->SpeedSetting = SERVO_ZERO;
    LeftMotor->RequestedSpeed = 0;
    LeftMotor->CurrentSpeed = 0;

    #if 0
    LeftMotor->MotorCurrent = 0;
    LeftMotor->LastEMFTime = 0;
    LeftMotor->EMFTime = 0;
    LeftMotor->MotorEMF = 0;
    LeftMotor->LastEMF = 0;
    LeftMotor->MotorEMFA_channel = ADC_CH4;
    LeftMotor->MotorEMFB_channel = ADC_CH8;
    LeftMotor->MotorEMFA = 0;
    LeftMotor->MotorEMFB = 0;
    #endif
  
    LeftMotor->RequestedDistance = 0;
  }
		
  // Create the structures used to pass parameters to the left motor task.
  RightMotor = (xMotorControlParameters *)malloc(sizeof( xMotorControlParameters));
	
  if( RightMotor != NULL )
  {
    RightMotor->Select = RIGHT_MOTOR;
    RightMotor->State = MOTORS_STOPPED;
    RightMotor->SpeedSetting = SERVO_ZERO;
    RightMotor->RequestedSpeed = 0;
    RightMotor->CurrentSpeed = 0;
    
    #if 0
    RightMotor->MotorCurrent = 0;
    RightMotor->LastEMFTime = 0;
    RightMotor->EMFTime = 0;
    RightMotor->MotorEMF = 0;
    RightMotor->LastEMF = 0;
    RightMotor->MotorEMFB_channel = ADC_CH1;
    RightMotor->MotorEMFA = 0;
    RightMotor->MotorEMFB = 0;
    #endif
    
    RightMotor->RequestedDistance = 0;
  }	

  odometer = 0;
  distance_to_move = 0;
  angle_accumulator = 0;
  motion_accumulator = 0;
  
  // Force Remote control
  //vMotorDriveRequest( 200, -200, 0, CRUISE );
}

static void vArbitrate( void )
{
  enum BEHAVIORS winner = NO_CONTROL_REQUEST;
	
  // Step through the behaviors. Highest priority first
  for( int i = 0; i < MAX_BEHAVIOR; i++ )
  {
    // if this priority behavior requires control
    // then grant it.
    if( motorControl[behaviorPriority[i]].BehaviorControl != NO_CONTROL_REQUEST )
    {
      // motor control code goes here
      winner = motorControl[behaviorPriority[i]].BehaviorControl;
      motor_drive(&(motorControl[winner]));
      break;
    }
  }
}

// Called by arbtrator. Converts speed from mm/s to a motor PWM setting
static void motor_drive( struct MOTOR_CONTROL_REQUESTS *motor_parameters )
{
  vMotorDriveRight( motor_parameters->right_motor.RequestedSpeed, motor_parameters->right_motor.RequestedDistance );
  vMotorDriveLeft( motor_parameters->left_motor.RequestedSpeed, motor_parameters->left_motor.RequestedDistance );
/*
        odometer = motor_parameters->left_motor.Distance;
	if( motor_parameters->right_motor.Speed == 0 )
	{
		vMotorDriveRight( motor_parameters->right_motor.Speed, motor_parameters->right_motor.Distance );
	}
	//else if( motor_parameters->right_motor.Speed >= METRIC_SPEED )
	//{
	//	vMotorDriveRight( 1, motor_parameters->right_motor.Distance );
	//}
	else
	{
    	// speed is in mm/s
    	// each back EMF count = METRIC_TO_SPEED mm
    	// so back EMF target = speed / METRIC_TO_SPEED
    	// METRIC_TO_SPEED is scaled by 10 so the requested speed must be too
		vMotorDriveRight((motor_parameters->right_motor.Speed * 10) / METRIC_TO_SPEED, motor_parameters->right_motor.Distance );
		//vMotorDriveRight(36, motor_parameters->right_motor.Distance );		
	}
    
	if( motor_parameters->left_motor.Speed == 0 )
	{
		vMotorDriveLeft( motor_parameters->left_motor.Speed, motor_parameters->left_motor.Distance );
	}
	//else if( motor_parameters->left_motor.Speed >= METRIC_SPEED )
	//{
	//	vMotorDriveLeft( 1, motor_parameters->left_motor.Distance );
	//}-
	else
	{
    	// speed is in mm/s
    	// each back EMF count = METRIC_TO_SPEED mm
    	// so back EMF target = speed / METRIC_TO_SPEED
    	// METRIC_TO_SPEED is scaled by 10 so the requested speed must be too
		vMotorDriveLeft((motor_parameters->left_motor.Speed * 10) / METRIC_TO_SPEED, motor_parameters->left_motor.Distance);
		//vMotorDriveLeft(36, motor_parameters->left_motor.Distance);
	}
		
	if( 0 != odometer)
	{
    	if( LeftMotor->CurrentDistance >= odometer)
    	{
        	// Execute an immediate stop
        	vMotorDriveLeft(0,0);
        	vMotorDriveRight(0,0);
        	vMotorDriveRequest(0, 0, 0, REMOTE_CONTROL);
        	odometer = 0;
    	}
	}
*/
}

/********************************************************************
*    Function Name:	vMotorMonitor				    *
*    Return Value:	none					    *
*    Parameters:	data:                         		    *
*    Description:	Read motor current and speed. Run motor	    *
*			control Arbiter.	                    *				
********************************************************************/
void vMotorMonitor( void )
{
  static enum MOTOR_STATE motor_state = MOTOR_START;
  static unsigned long motor_timer;
  static unsigned long left_motor_timer;
  static unsigned long right_motor_timer;
  int left_speed_error;
  int right_speed_error;
  WORD_VAL turn_delta;
  DWORD_VAL delta_motion;
      
  switch(motor_state)
  {
    case MOTOR_START:
      // Start the default CRUISE behavior after a two second delay
      motor_timer = set_timer(2000); 
      motor_state = POWER_ON_DELAY;
    break;
        
    case POWER_ON_DELAY:
      if(time_out(motor_timer))
      {
        motor_state = MOTORS_STOPPED;
      }      
    break;

    case MOTORS_STOPPED:
      noInterrupts();
      left_started = false;
      right_started = false;
      interrupts();
      left_odometer.sVal = 0;
      right_odometer.sVal = 0;
      odometer = 0;
      angle_accumulator = 0;
      motion_accumulator = 0;

      left_motor_timer = set_timer(ENCODER_TIMEOUT);
      right_motor_timer = set_timer(ENCODER_TIMEOUT);
      //vMotorDriveRequest( -270, 270, 440, CRUISE );
#ifdef ROTATION_TEST
      vMotorDriveRequest(-128, 128, 0, CRUISE );
#endif
      //vMotorDriveRequest(128, 128, 440, CRUISE );
      //vMotorDriveRadius( -1, 270, 0, REMOTE_CONTROL );
      motor_state = MOTORS_RUNNING;
    break;
        
    case MOTORS_RUNNING:
      if((left_timing_ready) || time_out(left_motor_timer))
      {
        if(left_timing_ready)
        {
          noInterrupts();
          left_timing_ready = false;
          left_elapsed_time_mail = left_elapsed_time;
          left_odometer_mail.Val = left_odometer_counter;
          left_odometer_counter = 0;
          interrupts();

          // Calculate current speed
          LeftMotor->CurrentSpeed = MM_PER_TICK / left_elapsed_time_mail;
        }
        else
        {
          // Timeout implies current speed = 0
          LeftMotor->CurrentSpeed = 0;
          left_odometer_mail.sVal = 0;
        }
        
        // Restart the timer
        left_motor_timer = set_timer(ENCODER_TIMEOUT);

        // If driving in reverse the speed and distance is negative
        if(LeftMotor->SpeedSetting < SERVO_ZERO)
        {
          LeftMotor->CurrentSpeed = -LeftMotor->CurrentSpeed;
          left_odometer_mail.sVal = -left_odometer_mail.sVal;
        }
        //#ifdef SERIAL_DEBUG
        //Serial.println (left_odometer_mail.sVal);
        //#endif

        // For proportional control Error = SP - PV or Set Point - Process Variable
        // SP = requested speed
        // PV = measured speed
        left_speed_error = LeftMotor->RequestedSpeed - LeftMotor->CurrentSpeed;
    
        if(left_speed_error != 0)
        {
          LeftMotor->SpeedSetting += left_speed_error / 2;
    
          if(LeftMotor->SpeedSetting < MIN_SERVO)
          {
            LeftMotor->SpeedSetting = MIN_SERVO;
          }
          else if(LeftMotor->SpeedSetting > MAX_SERVO)
          {
            LeftMotor->SpeedSetting = MAX_SERVO;
          }

          left_motor.writeMicroseconds(LeftMotor->SpeedSetting);  // set servo to mid-point
          
          #ifdef SERIAL_DEBUG
          //DebugPort.print (left_elapsed_time_mail);
          //DebugPort.print (", ");
          //DebugPort.print (LeftMotor->CurrentSpeed);
          //DebugPort.print (", ");
          //DebugPort.print (left_speed_error);
          //DebugPort.print (", ");
          //DebugPort.print ("Left motor set point ");
          //DebugPort.println (LeftMotor->SpeedSetting);
          #endif
        }
      }
      
      if((right_timing_ready) || time_out(right_motor_timer))
      {
        if(right_timing_ready)
        {
          noInterrupts();
          right_timing_ready = false;
          right_elapsed_time_mail = right_elapsed_time;
          right_odometer_mail.Val = right_odometer_counter;
          right_odometer_counter = 0;
          interrupts();
        
          // Calculate current speed
          RightMotor->CurrentSpeed = MM_PER_TICK / right_elapsed_time_mail;
        }
        else
        {
          RightMotor->CurrentSpeed = 0;
          right_odometer_mail.Val = 0;
        }
    
        // Restart the timer
        right_motor_timer = set_timer(ENCODER_TIMEOUT);
   
        if(RightMotor->SpeedSetting < SERVO_ZERO)
        {
          RightMotor->CurrentSpeed = -RightMotor->CurrentSpeed;
          right_odometer_mail.sVal = -right_odometer_mail.sVal;
        }

        // For proportional control Error = SP - PV or Set Point - Process Variable
        // SP = requested speed
        // PV = measured speed
        right_speed_error = RightMotor->RequestedSpeed - RightMotor->CurrentSpeed;
    
        if(right_speed_error != 0)
        {
          RightMotor->SpeedSetting += right_speed_error / 2;
    
          if(RightMotor->SpeedSetting < MIN_SERVO)
          {
            RightMotor->SpeedSetting = MIN_SERVO;
          }
          else if(RightMotor->SpeedSetting > MAX_SERVO)
          {
            RightMotor->SpeedSetting = MAX_SERVO;
          }

          right_motor.writeMicroseconds(RightMotor->SpeedSetting);  // set servo to mid-point
          
          #ifdef SERIAL_DEBUG
          //DebugPort.print (right_elapsed_time_mail);
          //DebugPort.print (", ");
          //DebugPort.print (RightMotor->CurrentSpeed);
          //DebugPort.print (", ");
          //DebugPort.print (right_speed_error);
          //DebugPort.print (", ");
          //DebugPort.print ("Right motor set point ");
          //DebugPort.println (RightMotor->SpeedSetting);
          #endif
        }
      }
      
      if((0 != right_odometer_mail.sVal) && (0 != left_odometer_mail.sVal))
      {
        //Serial.print (right_odometer_mail.sVal);
        //Serial.print (",");
        //Serial.print (left_odometer_mail.sVal);
        //Serial.print (",");
        int current_odometer = ((right_odometer_mail.sVal + left_odometer_mail.sVal) / 2);
        odometer += (current_odometer * SCALED_MM_PER_TICK);
        
        // Protect from interrupts
        /*
        noInterrupts();            
        // The mm traveled per encoder count was scaled by 256 to deal with the fraction
        // Here we truncate the fraction by just using the High Byte for the accumulation of mm traveled.
        right_odometer.byte.LB = sensorPacket.sensor.iDistanceTraveled.byte.HB;
        right_odometer.byte.HB = sensorPacket.sensor.iDistanceTraveled.byte.LB;
        interrupts(); 
        */
        
        int actualAngularMovement = (right_odometer_mail.sVal - left_odometer_mail.sVal);
        //right_odometer.sVal += current_odometer;
        
        #ifdef SERIAL_DEBUG
        //DebugPort.print(right_odometer_mail.sVal);
        //DebugPort.print(" ");
        //DebugPort.print(left_odometer_mail.sVal);
        //DebugPort.print(" ");
        //DebugPort.print(turn_delta.sVal);
        //DebugPort.print(" ");
        //DebugPort.println(right_odometer.sVal);
        #endif

        
        noInterrupts();        
        angle_accumulator += actualAngularMovement;
        motion_accumulator += current_odometer;
        
       // Scale the encoder counts
        //turn_delta.sVal = (int)(angle_accumulator * 1.974358974);
        //delta_motion.sVal = (int)(motion_accumulator * SCALED_MM_PER_TICK);
        turn_delta.sVal = angle_accumulator * 1.974358974;
        delta_motion.sVal = motion_accumulator * SCALED_MM_PER_TICK;
        
        //Convert from little endian to big
        //sensorPacket.sensor.iAngleTraveled.byte.LB = turn_delta.byte.HB;
        //sensorPacket.sensor.iAngleTraveled.byte.HB = turn_delta.byte.LB;
        //This divides the motion by 256 since SCALED_MM_PER_TICK is * 256
        sensorPacket.sensor.iDistanceTraveled.byte.HB = delta_motion.byte.HB;
        sensorPacket.sensor.iDistanceTraveled.byte.LB = delta_motion.byte.UB;
        interrupts();
        
        left_odometer_mail.sVal = 0;
        right_odometer_mail.sVal = 0;
        
#ifdef ROTATION_TEST        
      if(turn_delta.sVal >= 360)
      {
        // Execute an immediate stop
        vMotorDriveLeft(0,0);
        vMotorDriveRight(0,0);
        vMotorDriveRequest(0, 0, 0, REMOTE_CONTROL);
      }
#endif        
      }
      
      if(0 != distance_to_move)
      {
        if(odometer >= distance_to_move)
        {
          // Execute an immediate stop
          vMotorDriveLeft(0,0);
          vMotorDriveRight(0,0);
          vMotorDriveRequest(0, 0, 0, REMOTE_CONTROL);
          distance_to_move = 0;
        }
      }

      if((MOTORS_STOPPED == LeftMotor->State) && (MOTORS_STOPPED == RightMotor->State))
      {
        motor_state = MOTORS_STOPPED;
      }
    break;
        
    default:
    break;
  }
}


#if 0
unsigned int uiLeftCurrent( void )
{
  return LeftMotor->MotorCurrent;
}

unsigned int uiRightCurrent( void )
{
  return RightMotor->MotorCurrent;
}

unsigned int uiLeftEMF( void )
{
  return LeftMotor->MotorEMF;
}

unsigned int uiRightEMF( void )
{
  return RightMotor->MotorEMF;
}
#endif

int iVelocityLeft( void )
{
  return LeftMotor->CurrentSpeed;
}

int iVelocityRight( void )
{
  return RightMotor->CurrentSpeed;
}

static void vMotorDrive(int speed, MOTOR_CONTROL_PARAMETERS *motor)
{
  // If motor is stopping reset the motor state
  if(0 == speed)
  {
    motor->State = MOTORS_STOPPED;
  }

  switch(motor->State)
  {
    case MOTORS_STOPPED:
      motor->RequestedSpeed = speed;

      // Kick start the speed to get the motor running.
      motor->SpeedSetting = SERVO_ZERO + speed;
      motor->servo_setting.writeMicroseconds(motor->SpeedSetting);  // set servo

      if(0 != motor->RequestedSpeed)
      {
        motor->State = MOTORS_RUNNING;
        #ifdef SERIAL_DEBUG
        //DebugPort.print ("Left speed request ");
        //DebugPort.print (motor->RequestedSpeed);
        //DebugPort.print (", ");
        //DebugPort.print ("Left motor set point ");
        //DebugPort.println (motor->SpeedSetting);
        #endif
      }
    break;
    
    case MOTORS_RUNNING:
      if(speed != motor->RequestedSpeed)
      {
        motor->RequestedSpeed = speed;
      }
    break;
  
    case MOTOR_STALLED:
    break;
  
    default:
    break;
  }
}

static void vMotorDriveLeft( int speed, unsigned long distance )
{
  // If motors are stopping reset the motor state
  if(0 == speed)
  {
    LeftMotor->State = MOTORS_STOPPED;
  }

  switch(LeftMotor->State)
  {
    case MOTORS_STOPPED:
      LeftMotor->RequestedSpeed = speed;
      
      // Kick start the speed to get the motor running.
      LeftMotor->SpeedSetting = SERVO_ZERO + speed;
      left_motor.writeMicroseconds(LeftMotor->SpeedSetting);  // set servo

      if(0 != LeftMotor->RequestedSpeed)
      {
        LeftMotor->State = MOTORS_RUNNING;
        #ifdef SERIAL_DEBUG
        //DebugPort.print ("Left speed request ");
        //DebugPort.print (LeftMotor->RequestedSpeed);
        //DebugPort.print (", ");
        //DebugPort.print ("Left motor set point ");
        //DebugPort.println (LeftMotor->SpeedSetting);
        #endif
      }
    break;
    
    case MOTORS_RUNNING:
      if(speed != LeftMotor->RequestedSpeed)
      {
        LeftMotor->RequestedSpeed = speed;
      }
    break;
  
    case MOTOR_STALLED:
    break;
  
    default:
    break;
  }
}

static void vMotorDriveRight( int speed, unsigned long distance )
{
  // If motors are stopping reset the motor state
  if(0 == speed)
  {
    RightMotor->State = MOTORS_STOPPED;
  }

  switch(RightMotor->State)
  {
    case MOTORS_STOPPED:
      RightMotor->RequestedSpeed = speed;
      RightMotor->SpeedSetting = SERVO_ZERO + speed;
      right_motor.writeMicroseconds(RightMotor->SpeedSetting);  // set servo to mid-point

      if(0 != RightMotor->RequestedSpeed)
      {
        RightMotor->State = MOTORS_RUNNING;
      }
    break;
    
    case MOTORS_RUNNING:
      if(speed != RightMotor->RequestedSpeed)
      {
        RightMotor->RequestedSpeed = speed;
      }
    break;
  
    case MOTOR_STALLED:
    break;
  
    default:
    break;
  }
#if 0
  // Save the current back EMF as last (T1 EMF)
  //RightMotor->LastEMF = RightMotor->MotorEMF;
    
  // For proportional control Error = SP - PV or Set Point - Process Variable
  // SP = requested speed
  // PV = Motor back EMF
  //vError = abs_speed - RightMotor->MotorEMF;
  RightMotor->CurrentVelocity += vError;

  // Now calculate the distance moved between T1 EMF and T2 EMF readings
/*
	if(0 == RightMotor->LastEMFTime)
	{
    	RightMotor->LastEMFTime = RightMotor->EMFTime;
	}
	
	avg_emf = (RightMotor->LastEMF + RightMotor->MotorEMF) / 2;
	
	if(avg_emf < 0)
	{
    	avg_emf = -avg_emf;
	}

    // convert the average EMF to the current velocity
    // METRIC_TO_SPEED is scaled by 10 so the current velocity must be too	
	current_velocity = (avg_emf * METRIC_TO_SPEED) / 10;
	
	// To calculate distance traveled between T1 and T2
	// we need the delta time
	delta_time = RightMotor->EMFTime - RightMotor->LastEMFTime;
    
    //LeftMotor->CurrentDistance += (((avg_emf * 7) + (avg_emf / 2)) * delta_time) / 1000;
    RightMotor->CurrentDistance += ((current_velocity * delta_time) / 1000);
*/

	if( RightMotor->CurrentVelocity < -SPEED_100 )
	{
		RightMotor->CurrentVelocity = -SPEED_100;
	}
	else if( RightMotor->CurrentVelocity > SPEED_100 ) 
	{
		RightMotor->CurrentVelocity = SPEED_100;
	}

	if( speed == 0 )
	{
		digitalWrite(RMF, RM_FORWARD_DISABLE);
		digitalWrite(RMR, RM_REVERSE_DISABLE);
    analogWrite(RIGHT_HB25_CONTROL, SPEED_0);
		RightMotor->CurrentVelocity = 0;
		RightMotor->CurrentDistance = 0;
		RightMotor->State = MOTORS_STOPPED;
	}
	else if( speed > 0 )
	{
		digitalWrite(RMF, RM_FORWARD_ENABLE);
    digitalWrite(RMR, RM_REVERSE_DISABLE);
	}
	else
	{
		digitalWrite(RMF, RM_FORWARD_DISABLE);
		digitalWrite(RMR, RM_REVERSE_ENABLE);
	}

	if( RightMotor->CurrentVelocity < 0 )
	{
		RightMotorSpeed( -RightMotor->CurrentVelocity );
                //RightMotorSpeed( 128 );
	}
	else if( RightMotor->CurrentVelocity >= 0 )
	{
		RightMotorSpeed( RightMotor->CurrentVelocity );
                //RightMotorSpeed( 128 );
	}
#endif
}


// Speed is in mm/sec
void vMotorDriveRequest( int leftSpeed, int rightSpeed, long distance, enum BEHAVIORS behaviorID )
{
  if( leftSpeed > MAX_METRIC_SPEED )
  {
    leftSpeed = MAX_METRIC_SPEED;
  }
  else if( leftSpeed < -MAX_METRIC_SPEED )
  {
    leftSpeed = -MAX_METRIC_SPEED;
  }
		
  if( rightSpeed > MAX_METRIC_SPEED )
  {
    rightSpeed = MAX_METRIC_SPEED;
  }
  else if( rightSpeed < -MAX_METRIC_SPEED )
  {
    rightSpeed = -MAX_METRIC_SPEED;
  }
		
  motorControl[behaviorID].left_motor.RequestedSpeed = leftSpeed;
  motorControl[behaviorID].left_motor.RequestedDistance = distance;
  motorControl[behaviorID].right_motor.RequestedSpeed = rightSpeed;
  motorControl[behaviorID].right_motor.RequestedDistance = distance;
  motorControl[behaviorID].BehaviorControl = behaviorID;
  distance_to_move = distance;
}

// radius:
//   0      = Straight, also allowed are 32767 and -32768 (0x7fff and 0x8000)
//   1      = Spin counter clockwise if positive spped
//  -1	    = Spin clockwise if positive speed
//            negative speed is reverse, positive speed is forward
void vMotorDriveRadius( int radius, int speed, unsigned long distance, enum BEHAVIORS behaviorID )
{
  #ifdef SERIAL_DEBUG
  //DebugPort.print("Radius ");
  //DebugPort.print(radius);
  //DebugPort.print(" Speed ");
  //DebugPort.print(speed);
  //DebugPort.print(" Distance: ");
  //DebugPort.println(distance);
  #endif
  
  if( (radius == 0) || (radius == (int)32767) || (radius == (int)-32768) )
  {
    vMotorDriveRequest( speed, speed, distance, behaviorID );
  }
  else if( radius == (int)-1 )
  {
    vMotorDriveRequest( speed, -speed, distance, behaviorID );
  }
  else if( radius == (int)1 )
  {
    vMotorDriveRequest( -speed, speed, distance, behaviorID );
  }
  else
  {
    float b2r = METRIC_WHEEL_BASE / (2 * radius);
    int speedLeft = speed * (1 - b2r);
    int speedRight = speed * (1 + b2r);
    vMotorDriveRequest( speedLeft, speedRight, distance, behaviorID );
  }
}

/********************************************************************
*    Function Name:	vMotorEscape									*
*    Return Value:	none											*
*    Parameters:	data:                         					*
*    Description:	Monitor motor current and speed.				*
*					Backup if current too high.						*				
********************************************************************/
void vMotorEscape(void)
{
  static int leftEscapeSpeed = 0;
  static int rightEscapeSpeed = 0;

  if(MOTOR_STALLED == RightMotor->State)
  {
    //if( digitalRead(RMF) == RM_FORWARD_ENABLE )				// Determine stall direction
    {
      leftEscapeSpeed = -40;
      rightEscapeSpeed = -20;
    }
    //else
    {
      leftEscapeSpeed = 27;
      rightEscapeSpeed = 27;
    }

    escapeState = STALL;
  }
  else if(MOTOR_STALLED == LeftMotor->State)
  {
    //if( digitalRead(RMF) == RM_FORWARD_ENABLE )				// Determine stall direction
    {
      leftEscapeSpeed = -20;
      rightEscapeSpeed = -40;
    }
    //else
    {
      leftEscapeSpeed = 27;
      rightEscapeSpeed = 27;
    }
    
    escapeState = STALL;
  }

  if( escapeState == STALL )
  {
    vMotorDriveRequest( 0, 0, 0, ESCAPE );			// Stop
    escapeState = STALL_STOP;
  }
  else if( escapeState == STALL_STOP )
  {
    //if( (digitalRead(RMF) == digitalRead(RMR)) && (digitalRead(LMF) == digitalRead(LMR)) )			// Wait for stop
    //{
    //  escapeState = MOVE_AWAY;
    //  vMotorDriveRequest( leftEscapeSpeed, rightEscapeSpeed, 0, ESCAPE );	// Move and turn away
    //}
  }
  else if( escapeState == MOVE_AWAY )
  {
    //if( (digitalRead(RMF) != digitalRead(RMR)) && (digitalRead(LMF) != digitalRead(LMR)) )			// Wait for go
    //{
    //  vMotorDriveRequest( -rightEscapeSpeed, -leftEscapeSpeed, 0, ESCAPE );	// Drive foward and turn more
    //  escapeState = TURN_FORWARD;
    //}
  }
  else if( escapeState == TURN_FORWARD )
  {
    motorControl[ESCAPE].BehaviorControl = NO_CONTROL_REQUEST;
    //behaviorControl[ESCAPE] = NO_CONTROL_REQUEST;	// Escape behavior termination
    escapeState = NORMAL_CRUISE;
  }
}

/*
void vMotorTurnHeading( int radius, int speed, unsigned int new_heading, enum BEHAVIORS behaviorID )
{
	int new_radius = 0;
	unsigned int degrees_turn;
	float radians_turn;
	unsigned long distance;
	
	unsigned int heading = current_heading();
	
	if(heading < new_heading)
	{
    	if((new_heading - heading) > 180)
    	{
        	new_radius = (int)1;
        	degrees_turn = (heading + 360) - new_heading; 
    	}
    	else
    	{
        	new_radius = (int)-1;
        	degrees_turn = new_heading - heading; 
    	}
	}
	else if(heading > new_heading)
	{
    	if((heading - new_heading) > 180)
    	{
        	new_radius = (int)-1;
        	degrees_turn = (new_heading + 360) - heading; 
    	}
    	else
    	{
        	new_radius = (int)1;
        	degrees_turn = heading - new_heading; 
    	}
	}
	else if(heading == new_heading)
	{
    	return;
	}
	
   	radians_turn = (float)degrees_turn * RADIAN_CONVERSION;
   	
	// radius = 0 is just a simple spin
    if(radius == 0)
    {
        radius = new_radius;
    	distance = radians_turn * MM_PER_RADIAN;
    }	
	
	if( radius == (int)-1 )
	{
		vMotorDriveRequest( speed, -speed, distance, behaviorID );
	}
	else if( radius == (int)1 )
	{
		vMotorDriveRequest( -speed, speed, distance, behaviorID );
	}
	else
	{
		b2r = METRIC_WHEEL_BASE / (2 * radius);
		speedLeft = speed * (1 - b2r);
		speedRight = speed * (1 + b2r);
		distance = radians_turn * radius;
		vMotorDriveRequest( speedLeft, speedRight, distance, behaviorID );
	}
}
*/

void stop()//
{
  #ifdef SERIAL_DEBUG
  //DebugPort.println("All stop");
  #endif
  analogWrite(RIGHT_HB25_CONTROL,SPEED_0);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
  analogWrite(LEFT_HB25_CONTROL, SPEED_0);
}

void vReleaseControl( enum BEHAVIORS behaviorID )
{
  motorControl[behaviorID].BehaviorControl = NO_CONTROL_REQUEST;
}

/*-----------------------------------------------------------*/



