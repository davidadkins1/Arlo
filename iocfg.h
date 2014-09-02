/* iocfg.h
 *
 * Definitions for the I/O pins
 */
#ifndef IO_CFG_H
#define IO_CFG_H


// if sending out debug information
#define SERIAL_DEBUG

// Port selection

// if using Bluetooth
//#define BLUETOOTH_SUPPORTED

#ifdef BLUETOOTH_SUPPORTED
// Serial1 on the Mega used for Bluetooth serial port
//#define TxD 18
//#define RxD 19
#define blueToothSerial Serial1
#else
#define blueToothSerial Serial
#endif

// Sonar
#define	pingPin 22

#endif // IO_CFG_H


