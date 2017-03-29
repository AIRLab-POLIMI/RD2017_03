/*
 * IRremote: IRsendDemo - demonstrates sending IR codes with IRsend
 * An IR LED must be connected to Arduino PWM pin 3.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */


#include <IRremote.h>

IRsend irsend;
unsigned long timer;
void setup()
{
  pinMode(4,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  timer=millis();int i = 0;
  while(millis() - timer < 4000){  
    irsend.sendSony(0xE0E0906F, 32); i++; delay(40);
  }
	
  Serial.print("Emissione Fatta " );
  Serial.println(i);
	delay(5000); //5 second delay between each signal burst
 
  
}
