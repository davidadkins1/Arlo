#ifndef MOTOR_H
#define MOTOR_H
#include "behaviors.h"

void vMotorInitialize( void );
unsigned int uiLeftCurrent( void );
unsigned int uiRightCurrent( void );
unsigned int uiLeftEMF( void );
unsigned int uiRightEMF( void );
int iVelocityLeft( void );
int iVelocityRight( void );
void vMotorMonitor( void );
void vMotorDrive( int leftSpeed, int rightSpeed );
void vMotorDriveRequest( int leftSpeed, int rightSpeed, long distance, enum BEHAVIORS behaviorID );
void vMotorDriveRadius( int radius, int speed, unsigned long distance, enum BEHAVIORS behaviorID );
void vMotorTurnHeading( int radius, int speed, unsigned int new_heading, enum BEHAVIORS behaviorID );
void vReleaseControl( enum BEHAVIORS behaviorID );
unsigned int current_heading(void);

// Motor PWM speed defines
//#define	PRESCALE2	4
//#define PWM_PERIOD	74					// PWM_PERIOD = (1/20000000)*4*4*1 =  50 us or 20 khz
//#define STOP		PWM_PERIOD + 2		// PWM duty cycle is inverted 

enum MOTOR_SELECT
{
  LEFT_MOTOR,
  RIGHT_MOTOR
};

enum MOTOR_STATE
{
  MOTOR_START,
  POWER_ON_DELAY,
  MOTORS_STOPPED,
  WAIT_ENCODER,
  MOTORS_RUNNING,
  MOTOR_STALLED,
  MOTORS_COMPLETED
};


typedef struct MOTOR_CONTROL_PARAMETERS
{
  enum MOTOR_SELECT 	Select;
  enum MOTOR_STATE      State;
  int16_t               SpeedSetting;
  int16_t		RequestedSpeed;
  //uint8_t             MotorCurrent_channel;
  //unsigned char       MotorEMFA_channel;
  //unsigned char       MotorEMFB_channel;
  //unsigned int        MotorCurrent;
  //int                 LastEMF;
  //int                 MotorEMF;
  //unsigned int        MotorEMFA;
  //unsigned int        MotorEMFB;
  //unsigned long       LastEMFTime;
  //unsigned long       EMFTime;
  uint8_t		RequestedDirection;
  uint32_t              RequestedDistance;
  int16_t               CurrentSpeed;
  uint8_t               CurrentDirection;
  uint32_t              CurrentDistance;
}xMotorControlParameters;

/*
typedef struct BEHAVIOR_PARAMETERS
{
  int16_t      Speed;
  uint32_t     Distance;
};//MOTOR_CONTROL_PARAMETERS;
*/

struct MOTOR_CONTROL_REQUESTS
{
  enum BEHAVIORS BehaviorControl;
  MOTOR_CONTROL_PARAMETERS left_motor;
  MOTOR_CONTROL_PARAMETERS right_motor;
};
#endif

