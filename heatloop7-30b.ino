/* 
 * RELAY CONTROL FOR HEATER
 * Nick Porter
 * Test with Serial Monitor open
 */

// temp sensor stuff: just used for testing purposes
#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 7;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

// globals
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

// function prototypes
void heat(int state, int &pulseState); // evals pulseState and turns heat on/off
void cool(int &state);      // evals state and turns "fan" on/off

// I/O setup
void setup() {
  // initialize pins
  pinMode(BUTTON,INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(COOLLED,OUTPUT);
  pinMode(HEATLED,OUTPUT);
  Serial.begin( 9600);    // Serial Monitor for testing purposes
}

/* Poll for a temp measurement, keeping the state machine alive.
 * Returns true if a measurement is available. */
static bool measure_environment( float *temperature, float *humidity ) {
  static unsigned long measurement_timestamp = millis( );
  /* 5 sec delay doesn't seem to affect timing, but I was having
   * issues with using no delay -> 5 sec-> vvv    */
  if( millis( ) - measurement_timestamp > 5000ul ) {
    if( dht_sensor.measure( temperature, humidity ) == true ) {
      measurement_timestamp = millis( );
      return( true );
    }
  }
  return( false );
}

// Program loop
void loop() {
  float temperature;   // had invalid values on startup, so i gave it initial value
  float humidity;     // only here because testing sensor functions use it

  // while there is a temperature value, run loop
  while(measure_environment( &temperature, &humidity ) == true ) {

    // testing: print temp to serial monitor
    Serial.print( "T = " );
    Serial.print( temperature, 1 );
    Serial.print( " deg. C" );
    Serial.println( " " );
  
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
        digitalWrite(HEATLED,LOW);
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(COOLLED,LOW);
        pulseState = 0;
        Serial.println("Something went wrong...");
        break;
    }
    state0 = state;   // set previous state to current state after evaluation
  }
}

/* heater control: 75% duty cycle pulse
 * ON if state=1 OR button is pressed
 * all statements must wait for loop to finish before updating
 */
void heat(int state, int &pulseState) {
  if ((pulseState == 0 && state == 1) || digitalRead(BUTTON)==LOW) {
    digitalWrite(HEATLED,HIGH);
    digitalWrite(RELAY_PIN, HIGH);
    pulseState = 1;
    delay(pulse);
  }
  else if (pulseState == 1 || state != 1) {
    digitalWrite(HEATLED,LOW);
    digitalWrite(RELAY_PIN, LOW);
    pulseState = 0;
    delay(.33*pulse);
  }
}

// fan control function: just turns led on/off
void cool(int &state) {
  if (state == -1) {
    digitalWrite(COOLLED,HIGH);
  }
  else {
    digitalWrite(COOLLED,LOW);
  }
}
