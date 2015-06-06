/*
   OpTh.h 
   Copyright 2009 Martijn van den Burg, martijn@[remove-me-first]palebluedot . nl
   
   This file is part of the OpTh library for reading OpenTherm (TM)
   communications with Arduino.

   OpTh is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   OpTh is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with OpTh.  If not, see <http://www.gnu.org/licenses/>.

*/


/* This library is written for the Arduino Duemilanova and Arduino software version 0017.
 *
 * It's purpose is to make data transmitted with the OpenTherm (TM) protocol available
 * for other applications.
 *
 * This library may work with other hardware and/or software. YMMV.
 */

 

#ifndef OpThLib_h
#define OpThLib_h

#if (ARDUINO >= 100)
   #include "Arduino.h"
#else
   #include "WProgram.h"
#endif
#include <util/parity.h>

#define VERSION 0.1               // Version of this library

//changed sample size to 4 so that we can do a sample in one reply...as far as I understand, it samples SAMPLE_SIZE^2 times
#define SAMPLE_SIZE 3            // Manchester period sample size [number of samples]
#define TICK_TO_USEC 4            // factor to convert clock ticks to us (prescaler dependent)


/* Protocol-specific definitions */
#define FRAME_LENGTH 34           // OpenTherm protocol frame length [bits]
#define MIN_WAIT_FOR_SLAVE 20     // minimum time between master and slave frame [ms]
#define MIN_WAIT_FOR_MASTER 100   // minimum time between slave and (next) master frame [ms]


#define NO_ERROR (0)
#define WRITE_TIMER (0)
#define READ_TIMER (1)

// Master to Slave Messages
#define READ_DATA (0b000)
#define WRITE_DATA (0b001)
#define INVALID_DATA (0b010)
// Slave to Master Messages
#define READ_ACK (0b100)
#define WRITE_ACK (0b101)
#define DATA_INVALID (0b110)
#define UNKNOWN_DATAID (0b111)



class OpThLib {
   public:
      OpThLib();               // constructor
      void init(uint8_t inPin, uint8_t outPin);           // initialise timer and input pin
      void measureOtPeriod();           // measure the period of the OpenTherm signal
	   void setPeriod(int32_t per);		 // set the period of the OpenTherm signal - added by amvv
      int32_t getPeriod();              // return the period of the OT Manchester signal [us]
      void getHightLow();

      uint8_t sendMessage(uint8_t msg_type, 
                          uint8_t data_id, 
                          uint16_t data_value);
      uint8_t sendMessage(uint32_t frame);
      uint8_t sendMessage();
      
      void setFrame(uint32_t frame);    // Set the frame value manually
      uint32_t getFrame();              // Get the frame value: all 32 bits
      
      char *errmsg();                   // Contains error message in case a function returned an error
      
      uint8_t getParity();              // Return the frame parity
      uint8_t getMsgType();             // Return the type of message
      uint8_t getDataId();              // Return the data ID
      uint16_t getDataValue();          // Return the data value
      
      uint8_t isMaster();               // Returns 1 for master frame, 0 for slave frame

   private:
      void _sendFrame();
      void _initTimer(uint8_t mode);
      int8_t waitFrame();               // wait for a master or slave frame     
      int8_t readFrame();               // Read the frame from the wire 
      uint16_t _low;
      uint16_t _hight;
      uint16_t _average;
      char *_errorMsg;                  // can contain an error message (text)
      uint8_t _inputPin;                // Arduino pin where we read the signal
      uint8_t _outputPin;               // Arduino pin where we write the signal
      int16_t _preload;                 // timer preload value for interrupt-based sampling
      int32_t _otPeriod;                // period of one bit in the frame [ms]
      uint32_t _masterframe;
      uint32_t _frame;                  // a frame contains FRAME_LENGTH - 2 bits (start/stop are discarded)
};


#endif
