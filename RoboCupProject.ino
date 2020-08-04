#include "SR04.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#include <Servo.h>
#include <Stepper.h>
#include <dht_nonblocking.h>

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
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 7;
const float STEPS_PER_REV = 32;
const float GEAR_RED = 64;
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
const int BUTTON = 2;       // TESTING: pulse override trigger
const int COOLLED = 4;      // TESTING: led indicates temp above set temp
const int HEATLED = 5;      // TESTING: indication led (on if below temp)
const int RELAY_PIN = 6;    // control to relay (normally open mode)
const int setTemp = 40;     // temp set by user (degrees C)
const int tempMargin = 1;   // +/- margin of error for setTemp eval
int state = 0;              // status of device: 0=off, 1=heat on, -1=fan on
int state0 = 1;             // previous state
int pulse = 15000;          // pulse period in ms
int pulseState = 0;         // while heater on, pulse: 0=off, 1=on


Servo steeringServo;
Stepper steppermotor(STEPS_PER_REV, 10, 12, 11, 13);
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );
LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// function prototypes
void heat(int state, int &pulseState); // evals pulseState and turns heat on/off
void cool(int &state);      // evals state and turns "fan" on/off

void setup() {
    // Define pin outs
}

void loop() {
    // Call main program loops
}