/* iocfg.h
 *
 * Definitions for the I/O pins
 */
#ifndef IO_CFG_H
#define IO_CFG_H


// if sending out debug information
//#define SERIAL_DEBUG

// Port selection

// if using Bluetooth
//#define BLUETOOTH_SUPPORTED
//#define BLUETOOTH_CONTROL

#ifdef BLUETOOTH_SUPPORTED
  #define BluetoothPort Serial2
  
  #ifdef BLUETOOTH_CONTROL
    #define SerialPort Serial2
  #else
    #define SerialPort Serial
    #define DebugPort Serial2
  #endif
#else
  #define SerialPort Serial
#endif

// Sonar
#define	pingPin 22

#endif // IO_CFG_H



