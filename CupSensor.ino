#include <SR04.h>

//www.elegoo.com
//2016.12.08
#include "SR04.h"

#define TRIG_PIN 12
#define ECHO_PIN 11 
#define LIGHT_PIN 3
#define CUP_DISTANCE 7
#define TIME_DELAY 250

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

void setup() {
  pinMode(3, OUTPUT);
}

void loop() {
   if(sr04.Distance() < CUP_DISTANCE){
    digitalWrite(3, HIGH);
   } else {
    digitalWrite(3, LOW);
   }
   
   delay(TIME_DELAY);
}