/*
  RF.h - Simple library to get started with RF links.
  Created by Carl Friess, 2013

  Inspired by Kevin Darrah. http://www.kevindarrah.com
  This library includes the "PinChangeInterrupt" library by Rowdy Dog Software.
  

  Copyright 2013 Carl Friess

  This library is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RF_h
#define RF_h

#include "Arduino.h"

class RF_TX
{
  
  public:
    RF_TX();
    void write(char input[]);
    int sizeofArray(char in[]);
  
  private:
    void sendbyte(byte data_in);
    int _pin;
  
};

class RF_RX
{
  
  public:
    RF_RX();
    boolean available();
    char read();
    byte receivebyte();
    void attachInt();
    void detachInt();
    void writeToBuffer(char in);
    int _pin;

  private:
    volatile int _buffer_read_pos;
    volatile char _buffer [255];
    volatile int _buffer_write_pos;
  
};

void data_incoming();

#endif





/*==============================================================================
                     SUPPORT FOR INTERRUPTS ON ATTINY
==============================================================================*/



#if defined( __AVR_ATtiny2313__ ) || defined( __AVR_ATtinyX4__ ) || defined( __AVR_ATtinyX5__ )

/*============================================================================

  PinChangeInterrupt.h - Enhanced pin change interrupts for tiny processors.

  Copyright 2010 Rowdy Dog Software.

  This file is part of Arduino-Tiny.

  Arduino-Tiny is free software: you can redistribute it and/or modify it 
  under the terms of the GNU Lesser General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Arduino-Tiny is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License 
  along with Arduino-Tiny.  If not, see <http://www.gnu.org/licenses/>.

==============================================================================*/  

#ifndef PinChangeInterrupt_h
#define PinChangeInterrupt_h

#include <inttypes.h>

#include <core_build_options.h>
// rmv #include "wiring_private.h"


#if defined( __AVR_ATtiny2313__ )
#define NUMBER_PIN_CHANGE_INTERRUPT_HANDLERS (1)
#define NUMBER_PIN_CHANGE_INTERRUPT_PORTS 1
#endif

#if defined( __AVR_ATtinyX4__ )
#define NUMBER_PIN_CHANGE_INTERRUPT_HANDLERS (3)
#define NUMBER_PIN_CHANGE_INTERRUPT_PORTS 2
#endif

#if defined( __AVR_ATtinyX5__ )
#define NUMBER_PIN_CHANGE_INTERRUPT_HANDLERS (3)
#define NUMBER_PIN_CHANGE_INTERRUPT_PORTS 1
#endif

void attachPcInterrupt( uint8_t pin, void (*callback)(void), uint8_t mode );
void detachPcInterrupt( uint8_t pin );
void detachPcInterrupt( uint8_t pin, void (*callback)(void), uint8_t mode );


#endif


#endif