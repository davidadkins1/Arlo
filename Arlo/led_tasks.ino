/*******************************************************************************
 * FileName:    led_tasks.ino
 * Product:	ClipBoardBot
 * Processor:   Arduino AVR
 * Purpose:	Main setup and loop code
 *
 * (c)2013 David Adkins
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Revision History: 
 *
 * Revision 0.1 09/2013 by David Adkins: 
 * 		1. Creation of code base
 ******************************************************************************/

void led_initialize(void)
{
  // initialize the digital pin as an output.
  pinMode(TEST_LED, OUTPUT);
  
  // Flash the test LED rapidly a few times
  // to indicate a reset occured.
  for(int flash_count = 0; flash_count < 5; flash_count++)
  {
    digitalWrite(TEST_LED, HIGH);
    delay(200);    
    digitalWrite(TEST_LED, LOW);
    delay(200);    
  }
}

// Flash the test LED once a second
void led_blink_task(void)
{
  enum LED_STATES
  {
    LED_OFF,
    LED_ON,
    LED_WAIT_ON,
    LED_WAIT_OFF
  };

  static char led_task_state = LED_OFF;
  static unsigned long led_timer;

  switch(led_task_state)
  {
    case LED_OFF:   
      digitalWrite(TEST_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      led_task_state = LED_WAIT_ON;
      led_timer = set_timer(1000);
    break;

    case LED_ON:   
      digitalWrite(TEST_LED, LOW);    // turn the LED off by making the voltage LOW
      led_task_state = LED_WAIT_OFF;
      led_timer = set_timer(1000);
    break;

  case LED_WAIT_ON:
    if(time_out(led_timer))
    {
      led_task_state = LED_ON;
    }      
    break;

  case LED_WAIT_OFF:
    if(time_out(led_timer))
    {
      led_task_state = LED_OFF;
    }
    break;

  }
}



