/*
  Basic transmitter
  
  
  This is an example for the RF library.
  
  Data line of TX module connected to digital pin 2.
  
  
  Created by Carl Friess, 2013
  Released into the public domain.
*/

#include <RF.h>

RF_TX tx;

void setup ()  {

}

void loop ()  {
  
  tx.write("It works!\n");
  
  delay(1000);
  
}
