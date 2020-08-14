#include <LiquidCrystal_I2C.h>
#include <IRremote.h>
#include <Servo.h>
#include <HX711.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Stepper.h>
#include "SR04.h"

//---------------------------LCD SCREEN-----------------------------
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
//------------------------------------------------------------------

//---------------------------DC MOTOR-------------------------------
#define ENABLE 5
#define DIRA 3
#define DIRB 4
//------------------------------------------------------------------

//----------------------RELAY HEATER--------------------------------
int relayHeaterPin = 2;
//------------------------------------------------------------------

//-----------------------REMOTE AND IR RECEIVER---------------------
int receiver = 8;
IRrecv irrecv(receiver);
decode_results results;
//------------------------------------------------------------------

//----------------------SERVO MOTOR--------------------------------
Servo steeringServo;
//-----------------------------------------------------------------

//---------------------STEPPER MOTOR-------------------------------
const float STEPS_PER_REV = 32;
const float GEAR_RED = 64;
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
Stepper steppermotor(STEPS_PER_REV, 10, 12, 11, 13);
//-----------------------------------------------------------------

//---------------------ULTRASONIC SENSOR--------------------------
#define TRIG_PIN 6
#define ECHO_PIN 7
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long cupDistance;
//----------------------------------------------------------------

//---------------------TEMP SENSOR--------------------------------
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int temperature = 0;
int setTemp = 115;
//----------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  //-------------------REMOTE AND IR RECEIVER SETUP-----------------
  Serial.println("IR Receiver Button Decode");
  irrecv.enableIRIn();
  //----------------------------------------------------------------

  //------------------------TEMP SENSOR SETUP-----------------------
  mlx.begin();
  //----------------------------------------------------------------

  //-------------------LCD SETUP------------------------------------
  lcd.begin(16, 2);
  lcd.backlight();
  temperature = mlx.readObjectTempF();
  LCDWriteCurrent(temperature);
  //----------------------------------------------------------------

  //-------------------RELAY HEATER SETUP---------------------------
  pinMode(relayHeaterPin, OUTPUT);
  //----------------------------------------------------------------

  //------------------SERVO MOTOR SETUP-----------------------------
  steeringServo.attach(9);
  //----------------------------------------------------------------

  //----------------------DC MOTOR SETUP---------------------------
  pinMode(ENABLE, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  //---------------------------------------------------------------
}

void loop()
{
  Serial.println("in");
  for (int timing = 0; timing < 30000; timing++)
  {
    if (irrecv.decode(&results))
    {
      translateIR();
      irrecv.resume();
    }
  }
  Serial.println("out");
  cupDistance = sr04.Distance();
  temperature = mlx.readObjectTempF();
  LCDWriteCurrent(temperature);
  if (temperature < setTemp - 5 && cupDistance < 5)
  {
    digitalWrite(relayHeaterPin, HIGH);
    analogWrite(ENABLE, LOW); // Fan is off
    digitalWrite(DIRA, HIGH);
    digitalWrite(DIRB, LOW);
  }
  if (temperature > setTemp + 5 && cupDistance < 5)
  {
    digitalWrite(relayHeaterPin, LOW);
    analogWrite(ENABLE, 90); //255 is max fan speed, can be adjusted as needed
    digitalWrite(DIRA, HIGH);
    digitalWrite(DIRB, LOW);
  }

}

//-------------------------FUNCTIONS-----------------------------------

void translateIR()
{
  switch (results.value)
  {
    case 0xFFA25D: Serial.println("POWER"); break;
    case 0xFFE21D: Serial.println("FUNC/STOP"); break;
    case 0xFF629D: Serial.println("VOL+"); break;
    case 0xFF22DD: Serial.println("FAST BACK");    break;
    case 0xFF02FD: Serial.println("PAUSE"); steerDrink();    break;
    case 0xFFC23D: Serial.println("FAST FORWARD");   break;
    case 0xFFE01F: Serial.println("DOWN"); armDown();   break;
    case 0xFFA857: Serial.println("VOL-");    break;
    case 0xFF906F: Serial.println("UP"); armUp();    break;
    case 0xFF9867: Serial.println("EQ"); setTemperature();  break;
    case 0xFFB04F: Serial.println("ST/REPT");    break;
    case 0xFF6897: Serial.println("0");    break;
    case 0xFF30CF: Serial.println("1");    break;
    case 0xFF18E7: Serial.println("2");    break;
    case 0xFF7A85: Serial.println("3");    break;
    case 0xFF10EF: Serial.println("4");    break;
    case 0xFF38C7: Serial.println("5");    break;
    case 0xFF5AA5: Serial.println("6");    break;
    case 0xFF42BD: Serial.println("7");    break;
    case 0xFF4AB5: Serial.println("8");    break;
    case 0xFF52AD: Serial.println("9");    break;
    case 0xFFFFFFFF: Serial.println(" REPEAT"); break;
    default:
      Serial.println(" other button : ");
      Serial.println(results.value);
  }
  delay(500);
}

void LCDWriteCurrent(int temperature)
{
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Temperature:  ");
  lcd.setCursor(6, 1);
  lcd.print(temperature);
  lcd.setCursor(10, 1);
  lcd.print("*F");
}

void steerDrink()
{
  for (int numberOfSteer = 0; numberOfSteer < 5; numberOfSteer++)
  {
    digitalWrite(relayHeaterPin, LOW);
    analogWrite(ENABLE, LOW); // Fan is off
    steeringServo.write(10); // CHANGE VALUE WHEN TESTING
    delay(500);
    steeringServo.write(100); // CHANGE VALUE WHEN TESTING
    delay(500);
  }
}

void armUp()
{
  digitalWrite(relayHeaterPin, LOW);
  analogWrite(ENABLE, LOW); // Fan is off
  steppermotor.setSpeed(1550);
  steppermotor.step(4000); // CHANGE VALUE WHEN TESTING
}

void armDown()
{
  digitalWrite(relayHeaterPin, LOW);
  analogWrite(ENABLE, LOW); // Fan is off
  steppermotor.setSpeed(1550);
  steppermotor.step(-4000); // CHANGE VALUE WHEN TESTING
}

void setTemperature()
{
  digitalWrite(relayHeaterPin, LOW);
  analogWrite(ENABLE, LOW); // Fan is off
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set Temperature:");
  lcd.setCursor(4, 1);
  lcd.print(setTemp);
  lcd.setCursor(9, 1);
  lcd.print("*F");
  while (setTemp <= 200)
  {
    if (irrecv.decode(&results))
    {
      if (results.value == 0xFFE01F)
      {
        Serial.println("DOWN");
        setTemp = setTemp - 1 ;
      }
      if (results.value == 0xFF906F)
      {
        Serial.println("UP");
        setTemp = setTemp + 1;
      }
      if (results.value == 0xFF02FD)
      {
        Serial.println("PAUSE");
        break;
      }
      irrecv.resume();
      lcd.setCursor(4, 1);
      lcd.print("   ");
      lcd.setCursor(4, 1);
      lcd.print(setTemp);
    }
  }
  Serial.println(setTemp);
  delay(1000);
  lcd.clear();
}
