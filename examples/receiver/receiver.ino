/*
  Serial receiver
  
  
  This is an example for the RF library.
  
  Data line of RX module connected to digital pin 3.
  
  
  Created by Carl Friess, 2013
  Released into the public domain.
*/


#include <RF.h>

RF_RX rx;

void setup ()  {
  
  Serial.begin(9600);
  
}

void loop ()  {
  
  if(rx.available())
    Serial.print(rx.read());
  
}
