/*
Liquid Crystal library downloaded from:
https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/
https://www.banggood.com/IIC-I2C-2004-204-20-x-4-Character-LCD-Display-Module-Blue-p-908616.html?rmmds=myorder&cur_warehouse=CN

Keypad info:
http://playground.arduino.cc/Code/Keypad#Download

*/

#include "Keypad.h" // For keypad
#include "Wire.h" // For I2C
#include "LCD.h" // For LCD
#include "LiquidCrystal_I2C.h" // Added library
#include <Servo.h>

#define ledpin 13

String box = ""; // variable for box entry
String boxsel = ""; // variable for box selection
bool startscreentog = 1; // define if it is on start screen
long keypress = millis();
const byte ROWS = 4; // Four rows
const byte COLS = 4; // Four columns
int motora0p = 90;
int motorb0p = 80;

// Define the Keymap
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'#','0','*','D'}
};
// Connect keypad COL1, COL2, COL3 and COL4 to these Arduino pins.
byte colPins[COLS] = { 52,50,48,46 }; 
// Connect keypad ROW1, ROW2, ROW3 and ROW4 to these Arduino pins.
byte rowPins[ROWS] = { 53,51,49,47 };

// Create the Keypad
Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// Create servo object to control a servo
Servo motorA;
Servo motorB;

//Set the pins on the I2C chip used for LCD connections
//ADDR,EN,R/W,RS,D4,D5,D6,D7
LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7); // 0x27 is the default I2C bus 

void(* resetFunc) (void) = 0;//declare reset function at address 0

void setup()
{
  motorA.attach(2);  // attaches the servo on pin 2 to the servo object
  // motorA.write(motora0p); // ZERO position
  motorB.attach(3);  // attaches the servo on pin 2 to the servo object
  // motorB.write(motorb0p); // ZERO position
  pinMode(ledpin,OUTPUT);
  digitalWrite(ledpin, HIGH);
  // Serial.begin(9600);
  // Set off LCD module
  lcd.begin (20,4); // 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE); // BL, BL_POL
  lcd.setBacklight(HIGH);
  StartScreen();
}

void loop()
{
  char key = kpd.getKey();
  if(key)  // Check for a valid key.
  {
  SelectScreen();
    switch (key)
    {
      case '*': // clear selection
        // digitalWrite(ledpin, LOW);
        // Serial.println(key);
        box = "";
        lcd.setCursor(0,3);
        lcd.print("                    ");
        lcd.setCursor(2,3);
        break;
      case '#': // Enter selection
        // digitalWrite(ledpin, HIGH);
        // Serial.println(key);
        boxsel = box;
        break;
      default: // default action if case is not met
        // Serial.println(key);
        box = box + key;
        lcd.setCursor(2,3);
        lcd.print(box);
    }
  }
  // executing key presses
  if (boxsel == "11") {
    MotorAUp();
    boxsel = "";
    box = "";
    StartScreen();
  }
  else if (boxsel == "22") {
    MotorADown();
    boxsel = "";
    box = "";
    StartScreen();
  }
  else if (boxsel == "33") {
    MotorBUp();
    boxsel = "";
    box = "";
    StartScreen();
  }
  else if (boxsel == "44") {
    MotorBDown();
    boxsel = "";
    box = "";
    StartScreen();
  }
  else if (boxsel != ""){
    box = "";
    boxsel = "";
    lcd.setCursor(0,3);
    lcd.print("  Invalid selection ");
    delay(2000);
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(2,3);
  }
  // if no key is pressed within 5 seconds, clear line 4
  if (box != "") {
    if (millis() - keypress > 5000) {
      box = "";
      lcd.setCursor(0,3);
      lcd.print("                    ");
      lcd.setCursor(2,3);
    }
  }
  // if nothing happens in 15 sec. go to start screen
  if (box == "") {
    if (startscreentog == 0) {
      if (millis() - keypress > 15000) {
        keypress = 86400000;
        StartScreen();
      }
    }
  }
  // call reset every 24 hours
  if (millis() >= 86400000) {
  resetFunc();
  }
}

void StartScreen() {
  startscreentog = 1;
  lcd.clear();
  lcd.home();
  lcd.noBlink();
  lcd.print("      Kalman's      ");
  lcd.setCursor(0,2);
  lcd.print("  Vending Machine   ");
  lcd.setCursor(0,3);
  lcd.print("                V0.5");
}

void SelectScreen() {
    startscreentog = 0; // indicate out of start screen
    lcd.clear();
    lcd.home();
    lcd.blink();
    lcd.print("      Kalman's      ");
    lcd.setCursor(0,1);
    lcd.print("  Vending Machine   ");
    lcd.setCursor(0,2);
    lcd.print("  Select product:   ");
    keypress = millis();
}

void MotorAUp() {
  motorA.write(10); // Motor UP
  delay(500);
  motorA.write(85); // ZERO position
}

void MotorADown() {
  motorA.write(170); // Motor Down
  delay(500);
  motorA.write(88); // ZERO position
}

void MotorBUp() {
  motorB.write(10); // Motor UP
  delay(500);
  motorB.write(84); // ZERO position
}

void MotorBDown() {
  motorB.write(170); // Motor Down
  delay(500);
  motorB.write(88); // ZERO position
}

