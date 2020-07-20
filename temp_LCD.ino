#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>

#define I2C_ADDR 0x27 //I2C adress, you should use the code to scan the adress first (0x27) here
#define BACKLIGHT_PIN 3 // Declaring LCD Pins
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//Pin Assignments
//For Arduino Uno: SCL for sensor and LCD go to A5, SDA for sensor and LCD go to A4 

void setup() {
  
  mlx.begin();
  lcd.begin (16,2);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH); //Lighting backlight
  lcd.home ();

}

void loop() {
 lcd.setCursor(0,0);
 lcd.print("TEMP:  ");
 lcd.print(mlx.readObjectTempC());
 lcd.print(" C");

 delay(1000);

}