#include "SR04.h"
#include <Wire.h>
// #include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#include <Servo.h>
#include <Stepper.h>
#include <dht_nonblocking.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#define TRIG_PIN 6
#define ECHO_PIN 7 
#define LIGHT_PIN 3
#define CUP_DISTANCE 7
#define TIME_DELAY 250
#define I2C_ADDR 0x27 //I2C address, you should use the code to scan the adress first (0x27) here
#define BACKLIGHT_PIN 3 // Declaring LCD Pins
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7
#define ENABLE 5
#define DIRA 3
#define DIRB 4

const float STEPS_PER_REV = 32;
const float GEAR_RED = 64;
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
const int RELAY_PIN = 6;    // control to relay (normally open mode)
const int tempMargin = 5;   // +/- margin of error for setTemp eval
int setTemp = 100;     // temp set by user (degrees F)
int state = 0;              // status of device: 0=off, 1=heat on, -1=fan on
int state0 = 1;             // previous state
int pulse = 15000;          // pulse period in ms
int pulseState = 0;         // while heater on, pulse: 0=off, 1=on

Servo steeringServo;
Stepper steppermotor(STEPS_PER_REV, 10, 12, 11, 13);
LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
IRrecv irrecv(RECEIVER);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

// function prototypes
void heat(int state, int &pulseState); // evals pulseState and turns heat on/off
void cool(int &state);      // evals state and turns "fan" on/off
void LCD_write(int temperature); // 

void setup() {
    // setup for fan control (Luke)
    pinMode(ENABLE,OUTPUT);
    pinMode(DIRA,OUTPUT);
    pinMode(DIRB,OUTPUT);
    
    // setup for LCD (Luke)
    mlx.begin();
    lcd.begin (16,2);
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH); //Lighting backlight
    lcd.home ();
    
    // set for relay (nick)
    pinMode(RELAY_PIN, OUTPUT);
    
    // setup for IR_Receiver
    irrecv.enableIRIn();
    
    Serial.begin(9600);
}

void loop() {
    // Call main program loops
    float temperature;   // had invalid values on startup, so i gave it initial value
    temperature = mlx.readObjectTempF()
    LCD_write(temperature);
    
    // while(cup is present)
    while(sr04.Distance() < CUP_DISTANCE){
        // turn temp measurement into int state value
        if (temperature < (setTemp - tempMargin)) {
          state = 1;  // turn heater on, fan off
        }
        else if (temperature >= (setTemp - tempMargin) && temperature <= (setTemp + tempMargin)) {
          state = 0;  // turn everything off
        }
        else if (temperature > (setTemp + tempMargin)) {
          state = -1; // turn fan on, heat off
        }
        else {
          Serial.print("invalid state value: ");
          Serial.print(state, 1);
          Serial.println(" ");
        }
      
        /* switch controlled by state values only
         * indicate and operate based on given state value. */
        switch (state) {
          case 0:
            heat(state, pulseState);
            cool(state);
            if (state0 != state) {Serial.println("Target temp reached");}
            break;
          case 1:
            if (state0 != state) {Serial.println("The heat is on");}
            Serial.println("heat on");
            heat(state, pulseState);
            Serial.println("heat off");
            heat(state, pulseState);
            break;
          case -1:
            heat(state, pulseState);
            cool(state);
            if (state0 != state) {Serial.println("Cooling is on");}
            break;
          default:
            digitalWrite(RELAY_PIN, LOW);
            pulseState = 0;
            Serial.println("Something went wrong...");
            break;
        }
        state0 = state;   // set previous state to current state after evaluation
    }
}

/* heater control: 75% duty cycle pulse
 * ON if state=1
 * all statements must wait for loop to finish before updating
 */
void heat(int state, int &pulseState) {
  if (pulseState == 0 && state == 1) {
    digitalWrite(RELAY_PIN, HIGH);
    pulseState = 1;
    delay(pulse);
  }
  else if (pulseState == 1 || state != 1) {
    digitalWrite(RELAY_PIN, LOW);
    pulseState = 0;
    delay(.33*pulse);
  }
}

// fan control function
void cool(int &state) {
  if (state == -1) {
    analogWrite(ENABLE,255); //255 is max fan speed
    digitalWrite(DIRA,LOW); 
    digitalWrite(DIRB,HIGH);
  }
  else {
    analogWrite(ENABLE,0); //speed of 0 to turn fan off
    digitalWrite(DIRA,LOW); 
    digitalWrite(DIRB,HIGH);
  }
}
//LCD control
void LCD_write(int temperature){
    lcd.setCursor(0,0);
    lcd.print("TEMP:  ");
    lcd.print(temperature);
    lcd.print(" F");
}