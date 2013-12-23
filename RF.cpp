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

#include "Arduino.h"
#include "RF.h"


RF_RX* rfRXptr1;    //Define a pointer to be asigned to an instance of the RX class.


RF_TX::RF_TX ()  {
  
  _pin = 2;  //Setting the TX pin to D2.
  
  pinMode(_pin, OUTPUT);  //Making the TX pin an output.
  
}

void RF_TX::write (char input[])  {  //This function is used to send data.
  
  int data_length = sizeofArray(input);      //Get size  of data, which is to be sent.
  byte vericode1 = random(0x05, 0xF9);       //Generate a random number for connection verification.
  byte vericode2 = 0xFF - vericode1;         //Create a second number for connection verification.
  byte checksum = vericode2 ^ data_length;   //Create a third number for connection verification.

  for(int i = 0; i < 20; i++)  {  //"Warm up" the connection. This helps to make the receiver pick up the signal.

    digitalWrite(_pin, HIGH);
    delayMicroseconds(500);
    digitalWrite(_pin, LOW);
    delayMicroseconds(500);

  }

  digitalWrite(_pin, HIGH);  //Send ready status pattern to start syncronization.
  delayMicroseconds(3000);
  digitalWrite(_pin, LOW);
  delayMicroseconds(500);

  sendbyte(vericode1);    //Send connection verification numbers and data size.
  sendbyte(vericode2);
  sendbyte(checksum);
  sendbyte(data_length);

  for (int i = 0; i < data_length; i++)  {  //Send data byte by byte.

    sendbyte(input[i]);

  }

  digitalWrite(_pin, LOW);  //Setting TX pin to low to save power.
  
}

void RF_TX::sendbyte (byte data_in)  {  //This function should not be used directly. To send data use write(). This function only transmitts a single byte.

  for(int i = 0; i < 8; i++)  {  //Send the byte bit by bit.

    int thisbit = bitRead(data_in,i);  //Get the next bit.

    digitalWrite(_pin, thisbit);    //Write the bit to TX pin.
    delayMicroseconds(248);         //Wait 0,25ms (=250us).
    digitalWrite(_pin, !thisbit);   //Invert the bit to insure, that the connection remains active.
    delayMicroseconds(248);         //Wait 0,25ms (=250us).

  }

}

int RF_TX::sizeofArray (char in[])  {

  for (int i = 0; true; i++)  {

    if (in[i] == '\0')
      return (i);
    
    if (i >= 255)
      return 255;

  }

}


RF_RX::RF_RX ()  {
  
  _pin = 3;  //Set RX pin to D3.

  rfRXptr1 = this;  //Set a pointer to this instance of the RX class.
  
  attachInt();  //Attach interrupt to RX pin.
  
  pinMode(_pin, INPUT);  //Set the RX pin as an input.
  
}

void data_incoming()  {  //This function is called everytime the RX pin goes from low to high.
  
  int _pin = 3;  //Set RX pin to D3.

  for(int i = 0; i < 100; i++)  {  //Test for ready status pattern: HIGH for 3ms

    delayMicroseconds(20);

    if(digitalRead(_pin) == LOW)  {

      return;
      break;

    }

  }

  rfRXptr1->detachInt();  //Detach interrupt, so that the data transfereprocess isn't interrupted.

  while(true)  {  //Wait for RX pin to go LOW.

    if(digitalRead(_pin) == LOW)  {

      delayMicroseconds(600);  //Syncronize.
      
      byte vericode1 = rfRXptr1->receivebyte();    //Receive codes to verify connection. This is necessary, nois may have been interpreted as a connection.
      byte vericode2 = rfRXptr1->receivebyte();
      byte checksum = rfRXptr1->receivebyte();
      int data_length = rfRXptr1->receivebyte();
      
      //Verify connection and exit if connection cannot be varified.
      
      if(vericode1 + vericode2 != 0xFF || (vericode2 ^ data_length) != checksum)  {
        
        rfRXptr1->attachInt();
        return;
        
      }
      
      for (int i = 0; i < data_length; i++)  {  //Receive data byte by byte and write it to the buffer.

        rfRXptr1->writeToBuffer(rfRXptr1->receivebyte());

      }

      break;

    }

  }

  rfRXptr1->attachInt();



}

byte RF_RX::receivebyte ()  {  //This function is used to receive data. It should not be called directly use available() and read().
  
  byte data_in;
  
  for (int i = 0; i < 8; i++)  {  //Read next byte bit by bit.

    if(digitalRead(_pin) == HIGH)
      bitWrite(data_in, i, 1);
    else
      bitWrite(data_in, i, 0);

    delayMicroseconds(500);

  }
  
  return data_in;

}

boolean RF_RX::available  ()  {  //Use this function to check if data has been received.
  
  if (_buffer_write_pos == _buffer_read_pos)  {
    
    return false;
    
  }
  else  {
    
    return true;
    
  }
  
}

char RF_RX::read ()  {  //Use this function to read the next byte of received data.
  
  if(_buffer_write_pos != _buffer_read_pos)  {
    
    _buffer_read_pos++;
    return (_buffer[_buffer_read_pos%255]);
    
  }
  else  {
    
    return -1;
    
  }
  
}

void RF_RX::writeToBuffer (char in)	{

  _buffer_write_pos++;
  _buffer[_buffer_write_pos%255] = in;

}


//Detect hardware (ATmega or ATtiny) and configure interrupts accordingly.

#if defined( __AVR_ATtiny2313__ ) || defined( __AVR_ATtinyX4__ ) || defined( __AVR_ATtinyX5__ )

void RF_RX::attachInt ()  {
  
  attachPcInterrupt(3,data_incoming,RISING);
  
}

void RF_RX::detachInt ()  {

  detachPcInterrupt(3);
  
}

#else

void RF_RX::attachInt ()  {
  
  attachInterrupt(1,data_incoming,RISING);
  
}

void RF_RX::detachInt ()  {

  detachInterrupt(1);
  
}

#endif





/*==============================================================================
                     SUPPORT FOR INTERRUPTS ON ATTINY
==============================================================================*/



#if defined( __AVR_ATtiny2313__ ) || defined( __AVR_ATtinyX4__ ) || defined( __AVR_ATtinyX5__ )

/*==============================================================================

  PinChangeInterrupt.cpp - Enhanced pin change interrupts for tiny processors.

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


#include <wiring.h>
#include <pins_arduino.h>
#include <core_macros.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include <PinChangeInterrupt.h>


typedef struct 
{
#if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
  uint8_t           port;
#endif
  uint8_t           changeMask;
  uint8_t           risingMask;
  uint8_t           fallingMask;
  void              (*callback)(void);
//rmv  voidFuncPtr       callback;
}
pcint_entry_t;

typedef volatile pcint_entry_t* pcint_entry_p;


volatile static pcint_entry_t pcint[NUMBER_PIN_CHANGE_INTERRUPT_HANDLERS];


static volatile uint8_t previousPinValue0;

#if defined( PCMSK1 )
static volatile uint8_t previousPinValue1;
#endif


void attachPcInterrupt( uint8_t pin, void (*callback)(void), uint8_t mode ) 
{
#if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
  uint8_t           port;
#endif
  uint8_t           bitMask;
  uint8_t           changeMask;
  uint8_t           risingMask;
  uint8_t           fallingMask;
  pcint_entry_p     hole;
  pcint_entry_p     entry;

  changeMask = 0;
  risingMask = 0;
  fallingMask = 0;

  bitMask = digitalPinToBitMask( pin );
  
  switch ( mode )
  {
  case CHANGE:
    changeMask = bitMask;
    break;

  case FALLING:
    fallingMask = bitMask;
    break;

  case RISING:
    risingMask = bitMask;
    break;

  default:
    return;
  }

#if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
  port = digitalPinToPort( pin );
#endif

tryAgain:

  hole = 0;

  entry = pcint;
  for ( uint8_t i=0; i < NUMBER_PIN_CHANGE_INTERRUPT_HANDLERS; ++i )
  {
    uint8_t SaveSREG = SREG;
    cli();

    if ( (hole == 0) && (entry->callback == 0) )
    {
      hole = entry;
    }
    else
    {
      if ( 
          #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
            (port == entry->port) &&
          #endif
          (changeMask == entry->changeMask)
          && (risingMask == entry->risingMask)
          && (fallingMask == entry->fallingMask)
          && (callback == entry->callback) )
      {
        SREG = SaveSREG;
        return;
      }
    }

    SREG = SaveSREG;

    ++entry;
  }

  if ( hole == 0 )
    return;

  uint8_t SaveSREG = SREG;
  cli();

  if ( hole->callback == 0 )
  {
    #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
      hole->port = port;
    #endif
    hole->changeMask = changeMask;
    hole->risingMask = risingMask;
    hole->fallingMask = fallingMask;
    hole->callback = callback;

    #if defined( __AVR_ATtinyX4__ )

      if ( port == PORT_A_ID )
      {
        GIMSK |= MASK1( PCIE0 );

        if ( (PCMSK0 & bitMask) == 0 )
        {
          previousPinValue0 |= (PINA & bitMask);
          PCMSK0 |= bitMask;
        }
      }
      else // if ( port = PORT_ID_B )
      {
        GIMSK |= MASK1( PCIE1 );

        if ( (PCMSK1 & bitMask) == 0 )
        {
          previousPinValue1 |= (PINB & bitMask);
          PCMSK1 |= bitMask;
        }
      }

    #endif


    #if defined( __AVR_ATtinyX5__ ) || defined( __AVR_ATtiny2313__ )

      GIMSK |= MASK1( PCIE );

      if ( (PCMSK & bitMask) == 0 )
      {
        previousPinValue0 |= (PINB & bitMask);
        PCMSK |= bitMask;
      }

    #endif
  }
  else
  {
    SREG = SaveSREG;
    goto tryAgain;
  }

  SREG = SaveSREG;
}


static void ClearEntry( pcint_entry_p entry )
{
#if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
  entry->port = 0;
#endif
  entry->changeMask = 0;
  entry->risingMask = 0;
  entry->fallingMask = 0;
  entry->callback = 0;
}

static void InterruptOff( 
    #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
      uint8_t port, 
    #endif
    uint8_t bitMask )
{
  #if defined( __AVR_ATtinyX4__ )

    if ( port == PORT_A_ID )
    {
      previousPinValue0 &= bitMask;
      PCMSK0 &= bitMask;
      
      if ( PCMSK0 == 0 )
      {
        GIMSK &= ~ MASK1( PCIE0 );
      }
    }
    else // if ( port = PORT_ID_B )
    {
      previousPinValue1 &= bitMask;
      PCMSK1 &= bitMask;
      
      if ( PCMSK1 == 0 )
      {
        GIMSK &= ~ MASK1( PCIE1 );
      }
    }

  #endif


  #if defined( __AVR_ATtinyX5__ ) || defined( __AVR_ATtiny2313__ )

    previousPinValue0 &= bitMask;
    PCMSK &= bitMask;

    if ( PCMSK == 0 )
    {
      GIMSK &= ~ MASK1( PCIE );
    }

  #endif
}

void detachPcInterrupt( uint8_t pin )
{
#if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
  uint8_t           port;
#endif
  uint8_t           bitMask;
  pcint_entry_p     entry;

#if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
  port = digitalPinToPort( pin );
#endif
  bitMask = digitalPinToBitMask( pin );

  InterruptOff( 
      #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
        port, 
      #endif
      ~ bitMask );

  entry = pcint;
  for ( uint8_t i=0; i < NUMBER_PIN_CHANGE_INTERRUPT_HANDLERS; ++i )
  {
    uint8_t SaveSREG = SREG;
    cli();

    if ( 
        #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
          (entry->port == port) &&
        #endif
        ( (entry->changeMask == bitMask) || (entry->risingMask == bitMask) || (entry->fallingMask == bitMask) ) )
    {
      ClearEntry( entry );
    }

    SREG = SaveSREG;

    ++entry;
  }
}


void detachPcInterrupt( uint8_t pin, void (*callback)(void), uint8_t mode )
{
#if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
  uint8_t           port;
#endif
  uint8_t           bitMask;
  uint8_t           changeMask;
  uint8_t           risingMask;
  uint8_t           fallingMask;
  uint8_t           otherMask;
  pcint_entry_p     entry;

  changeMask = 0;
  risingMask = 0;
  fallingMask = 0;

#if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
  port = digitalPinToPort( pin );
#endif
  bitMask = digitalPinToBitMask( pin );
  
  switch ( mode )
  {
  case CHANGE:
    changeMask = bitMask;
    break;

  case FALLING:
    fallingMask = bitMask;
    break;

  case RISING:
    risingMask = bitMask;
    break;

  default:
    return;
  }

  otherMask = 0;

  entry = pcint;
  for ( uint8_t i=0; i < NUMBER_PIN_CHANGE_INTERRUPT_HANDLERS; ++i )
  {
    uint8_t SaveSREG = SREG;
    cli();

    #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
      if ( (entry->port == port) )
    #endif
    {
      if ( (callback == entry->callback)
          && (changeMask == entry->changeMask)
          && (risingMask == entry->risingMask)
          && (fallingMask == entry->fallingMask) )
      {
        ClearEntry( entry );
      }
      else
      {
        otherMask |= entry->changeMask | entry->risingMask | entry->fallingMask;
      }
    }

    SREG = SaveSREG;

    ++entry;
  }

  if ( (otherMask & bitMask) == 0 )
  {
    InterruptOff( 
        #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
          port, 
        #endif
        ~ bitMask );
  }
}


static void PCINTX_common( 
    #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
      uint8_t port, 
    #endif
    uint8_t currentPinValue, volatile uint8_t& previousPinValue )
{
  pcint_entry_p     entry;
  uint8_t           changeMask;
  uint8_t           risingMask;
  uint8_t           fallingMask;

  changeMask = previousPinValue ^ currentPinValue;
  risingMask = (~previousPinValue) & currentPinValue;
  fallingMask = previousPinValue & (~currentPinValue);
  previousPinValue = currentPinValue;


  entry = pcint;
  for ( uint8_t i=0; i < NUMBER_PIN_CHANGE_INTERRUPT_HANDLERS; ++i )
  {
    if ( 
        #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
          (entry->port == port) &&
        #endif
        (entry->callback != 0)
        && ( (entry->changeMask & changeMask) || (entry->risingMask & risingMask) || (entry->fallingMask & fallingMask) ) )
    {
      (entry->callback)();
    }
    ++entry;
  }
}

#if defined( PCINT1_vect )
ISR( PCINT1_vect )
{
  #if defined( __AVR_ATtinyX4__ )
    PCINTX_common( 
        #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
          PORT_B_ID, 
        #endif
        PINB & PCMSK1, previousPinValue1 );
  #endif
}
#endif

ISR( PCINT0_vect )
{
  #if defined( __AVR_ATtinyX4__ )
    PCINTX_common( 
        #if NUMBER_PIN_CHANGE_INTERRUPT_PORTS > 1
          PORT_A_ID, 
        #endif
        PINA & PCMSK0, previousPinValue0 );
  #endif

  #if defined( __AVR_ATtinyX5__ ) || defined( __AVR_ATtiny2313__ )
    PCINTX_common( PINB & PCMSK, previousPinValue0 );
  #endif
}



/* 
    GIMSK |= MASK2( PCIE1, PCIE0 );

    volatile uint8_t* pcMaskRegister;
    pcMaskRegister = portPcMaskRegister( port );

    if ( (*pcMaskRegister & bitMask) == 0 )
    {
      volatile uint8_t* inputRegister;
      inputRegister = portInputRegister( port );

      portInputRegister( port );
      previousPin[port-1] |= (*inputRegister & bitMask);
      *pcMaskRegister |= bitMask;
    }
*/

/*
'2313...

  PCMSK  PCINT[7:0]  PINB (bits line up)

  GIMSK
    PCIE  PCINT[7:0]

  PCINT


'84...

  PCMSK1  PCINT[11:8]  PINB (bits line up)

  PCMSK0  PCINT[7:0]   PINA (bits line up)

  GIMSK  
    PCIE1  PCINT[11:8]
    PCIE0  PCINT[7:0]

  PCINT1
  PCINT0


'85...

  PCMSK  PCINT[5:0]  PINB (bits line up)

  GIMSK
    PCIE  PCINT[5:0]

  PCINT0


'328...

  PCMSK2  PCINT[23:16]  PIND (bits line up)

  PCMSK1  PCINT[14:8]   PINC (bits line up)

  PCMSK0  PCINT[7:0]    PINB (bits line up)

  PCICR
    PCIE2  PCINT[23:16] 
    PCIE1  PCINT[14:8]
    PCIE0  PCINT[7:0]

  PCINT2
  PCINT1
  PCINT0
*/


#endif