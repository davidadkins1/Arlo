#include "ITG3200.h"

ITG3200 gyro;
static unsigned long gyro_timer;

void init_gyro(void)
{
  gyro.init();
  gyro_timer = set_timer(30);
}

void gyro_task(void)
{
  if(time_out(gyro_timer))
  {
    int x,y,z;
    WORD_VAL gyro_t;
    
    gyro.getXYZ(&x,&y,&z);
    // Convert z to turtlebot format
    //250 degrees/sec centered around 512.
    float real_z = z;
    gyro_t.sVal = int(real_z / 13.33333) + 512;
    
    #ifdef SERIAL_DEBUG
    //DebugPort.print("Gyro value of Z: ");
    //DebugPort.print(z);
    //DebugPort.print(" , ");
    //DebugPort.println(gyro_t.sVal);
    #endif
    
    noInterrupts();
    sensorPacket.sensor.uiAnalogSignal.byte.LB = gyro_t.byte.HB;
    sensorPacket.sensor.uiAnalogSignal.byte.HB = gyro_t.byte.LB;
    interrupts();
    
    gyro_timer = set_timer(30);
  }
}
