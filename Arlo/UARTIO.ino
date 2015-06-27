/*******************************************************************************
 * FileName:    UARTIO.ino
 * Product:	ClipBoardBot
 * Processor:   Arduino AVR
 * Purpose:	Drivers for serial I/O
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

//String retSymb = "+RTINQ=";//start symble when there's any return
//String slaveName = ";SeeedBTSlave";//Set the Slave name ,caution that ';'must be included
//int nameIndex = 0;
//int addrIndex = 0;

//String recvBuf;
//String slaveAddr;

//String connectCmd = "\r\n+CONN=";

void uartio_intialize( void )
{  
  #ifdef BLUETOOTH_SUPPORTED
  setupBlueToothConnection();
  #else
  blueToothSerial.begin(57600);
  #endif
  
  //wait 1s and flush the serial buffer
  delay(1000);
  
  blueToothSerial.flush();
}

void vUARTRxControl()
{
  const char NO_CMD = 0;
  const char PDA_CMD = 1;
  const char IROBOT_CMD = 2;

  static char rx_task_state = NO_CMD;
 
  while(blueToothSerial.available() > 0)
  {
    byte incomingByte = blueToothSerial.read();

    if(NO_CMD == rx_task_state)
    {
      //Serial.println("PDA command 1");
      if(0x01 == incomingByte)
      {
        #ifdef SERIAL_DEBUG
        Serial.print("PDA command 1 ");
        #endif
        rx_task_state = PDA_CMD;
      }
      else if(incomingByte > 127)
      {
        #ifdef SERIAL_DEBUG
        Serial.print("Roomba command ");
        #endif
        rx_task_state = IROBOT_CMD;
      }
    }
    
    else if(PDA_CMD == rx_task_state)
    {
      if(true == vPDACmd(incomingByte))
      {
        rx_task_state = NO_CMD;
      }
    }
    
    if(IROBOT_CMD == rx_task_state)
    {
      iRobotCmd(incomingByte);
      rx_task_state = NO_CMD;
    }
  }
}
  
boolean xUARTIOGetChar(byte port_number, byte *pcRxedChar, long timeout)
{
  /* Get the next character from the buffer.  Return false if no characters
     are available, or arrive before xBlockTime expires. */
  while(blueToothSerial.available() == 0);
  *pcRxedChar = blueToothSerial.read();
  return(true);
}


void setupBlueToothConnection()
{
  blueToothSerial.begin(38400); //Set BluetoothBee BaudRate to default baud rate 38400
  blueToothSerial.print("\r\n+STWMOD=0\r\n"); //set the bluetooth work in slave mode
  blueToothSerial.print("\r\n+STNA=ClipBoardBot\r\n"); //set the bluetooth name
  blueToothSerial.print("\r\n+STPIN=0000\r\n");//Set SLAVE pincode"0000"
  blueToothSerial.print("\r\n+STOAUT=1\r\n"); // Permit Paired device to connect me
  blueToothSerial.print("\r\n+STAUTO=0\r\n"); // Auto-connection should be forbidden here
  delay(2000); // This delay is required.
  blueToothSerial.print("\r\n+INQ=1\r\n"); //make the slave bluetooth inquirable 
  Serial.println("The slave bluetooth is inquirable!");
  delay(2000); // This delay is required.
  blueToothSerial.flush();
  //blueToothSerial.print("\r\n+CONN=d8:b3:77:ee:48:7b\r\n"); // Auto-connection should be forbidden here
}

/********************************************************************
*    Function Name:  vUARTIOWriteBT                                	*
*    Return Value:   none                                           *
*    Parameters:     data: data to transmit                         *
*    Description:    This routine transmits a byte out the USART.   *
********************************************************************/
void vUARTIOWriteBT( const unsigned char *data, unsigned int byteCount )
{
  blueToothSerial.write(data, byteCount);
}




