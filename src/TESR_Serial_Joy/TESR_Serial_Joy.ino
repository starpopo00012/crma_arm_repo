#include "Emakefun_MotorDriver.h"
#include <PS2X_lib.h>
#include "EEPROM.h"
#include <Servo.h>

#define  PS2_DAT    12     // MISO
#define  PS2_CMD    11     // MOSI
#define  PS2_ATT    10     // SS
#define  PS2_CLK    13     // CLK

PS2X ps2x; // create PS2 Controller Class
Emakefun_MotorDriver mMotorDriver = Emakefun_MotorDriver(0x60);

Emakefun_Servo *mServo1 = mMotorDriver.getServo(1); //Base
Emakefun_Servo *mServo2 = mMotorDriver.getServo(2); //Joint 1
Emakefun_Servo *mServo3 = mMotorDriver.getServo(3); //Joint 2
Emakefun_Servo *mServo4 = mMotorDriver.getServo(4); //Joint 3
Emakefun_Servo *mServo5 = mMotorDriver.getServo(5); //Joint 4
Emakefun_Servo *mServo6 = mMotorDriver.getServo(6); //Griper

Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo
Servo servo3;  // create servo object to control a servo
Servo servo4;  // create servo object to control a servo
Servo servo5;  // create servo object to control a servo
Servo servo6;  // create servo object to control a servo


int S1_pos = 90, S2_pos = 90, S3_pos = 120, S4_pos = 90, S5_pos = 90, S6_pos = 10;
int Home_S1_pos = 90, Home_S2_pos = 90, Home_S3_pos = 90, Home_S4_pos = 90, Home_S5_pos = 90, Home_S6_pos = 10;

const int S1_Upper_Bound = 180 , S1_Lower_Bound = 0;
const int S2_Upper_Bound = 165 , S2_Lower_Bound = 15;
const int S3_Upper_Bound = 180 , S3_Lower_Bound = 0;
const int S4_Upper_Bound = 180 , S4_Lower_Bound = 0;
const int S5_Upper_Bound = 180 , S5_Lower_Bound = 0;
const int S6_Upper_Bound = 60 , S6_Lower_Bound = 0;

String inputString = "";         // a String to hold incoming data
String Encoder_FeedBack = "";
bool stringComplete = false;     // whether the string is complete
int error = 0;
byte type = 0;

void setup()
{
  Serial.begin(115200);
  delay(500);
  joystickSetup();

  mMotorDriver.begin(50);

  //  Home_S1_pos = EEPROM.read(0);
  //  Home_S2_pos = EEPROM.read(1);
  //  Home_S3_pos = EEPROM.read(2);
  //  Home_S4_pos = EEPROM.read(3);
  //  Home_S5_pos = EEPROM.read(4);
  //  Home_S6_pos = EEPROM.read(5);

  S1_pos = Home_S1_pos;
  S2_pos = Home_S2_pos;
  S3_pos = Home_S3_pos;
  S4_pos = Home_S4_pos;
  S5_pos = Home_S5_pos;
  S6_pos = Home_S6_pos;

  mServo1->writeServo(S1_pos);
  mServo2->writeServo(S2_pos);
  mServo3->writeServo(S3_pos);
  mServo4->writeServo(S4_pos);
  mServo5->writeServo(S5_pos);
  mServo6->writeServo(S6_pos);

  servo1.attach(0);  // attaches the servo on pin 0 
  servo2.attach(1);  // attaches the servo on pin 3 
  servo3.attach(2);  // attaches the servo on pin 4 
  servo4.attach(3);  // attaches the servo on pin 7
  servo5.attach(4);  // attaches the servo on pin 7
  servo6.attach(6);  // attaches the servo on pin 7 

  Serial.print("Home\r");
}

void loop()
{
  ps2x.read_gamepad();
  if (ps2x.Button(PSB_PAD_RIGHT)) S1_pos = S1_pos - 1; // Rotate Right
  if (ps2x.Button(PSB_PAD_LEFT)) S1_pos = S1_pos + 1; // Rotate Left
  if (ps2x.Button(PSB_PAD_DOWN)) S2_pos = S2_pos - 1; // Backward
  if (ps2x.Button(PSB_PAD_UP)) S2_pos = S2_pos + 1;   // Forward
  if (ps2x.Button(PSB_PINK)) S3_pos = S3_pos + 1; // Perk
  if (ps2x.Button(PSB_BLUE)) S3_pos = S3_pos - 1; // Bent
  if (ps2x.Button(PSB_GREEN))S4_pos = S4_pos + 1; // Bent
  if (ps2x.Button(PSB_RED))  S4_pos = S4_pos - 1; // Perk
  if (ps2x.Button(PSB_L1)) S5_pos = S5_pos + 1; // Rotate Left
  if (ps2x.Button(PSB_R1)) S5_pos = S5_pos - 1; // Rotate Right
  if (ps2x.Button(PSB_L3)) S6_pos = S6_pos + 1; // Close Gripper
  if (ps2x.Button(PSB_R3)) S6_pos = S6_pos - 1; // Open Gripper

  if (ps2x.Button(PSB_SELECT)) // Home
  {
    S1_pos = Home_S1_pos;
    S2_pos = Home_S2_pos;
    S3_pos = Home_S3_pos;
    S4_pos = Home_S4_pos;
    S5_pos = Home_S5_pos;
    S6_pos = Home_S6_pos;
  }

  if (stringComplete == true)
  {
    // S,degree_Servo0,degree_Servo1,degree_Servo2,degree_Servo3,degree_Servo3,degree_Servo4,degree_Servo5,\r
    if (inputString.length() == 27 && inputString[0] == 'S' && inputString[26] == '\r')
    {
      S1_pos = ((inputString[2] - 48) * 100) + ((inputString[3] - 48) * 10) + (inputString[4] - 48);
      S2_pos = ((inputString[6] - 48) * 100) + ((inputString[7] - 48) * 10) + (inputString[8] - 48);
      S3_pos = ((inputString[10] - 48) * 100) + ((inputString[11] - 48) * 10) + (inputString[12] - 48);
      S4_pos = ((inputString[14] - 48) * 100) + ((inputString[15] - 48) * 10) + (inputString[16] - 48);
      S5_pos = ((inputString[18] - 48) * 100) + ((inputString[19] - 48) * 10) + (inputString[20] - 48);
      S6_pos = ((inputString[22] - 48) * 100) + ((inputString[23] - 48) * 10) + (inputString[24] - 48);
    }
    else if (inputString.length() == 6 && inputString[0] == 'H' && inputString[5] == '\r') // HOME,\r
    {
      S1_pos = Home_S1_pos;
      S2_pos = Home_S2_pos;
      S3_pos = Home_S3_pos;
      S4_pos = Home_S4_pos;
      S5_pos = Home_S5_pos;
      S6_pos = Home_S6_pos;
    }
    // SH,degree_Servo0,degree_Servo1,degree_Servo2,degree_Servo3,degree_Servo3,degree_Servo4,degree_Servo5,\r
    else if (inputString.length() == 28 && inputString[0] == 'S' && inputString[27] == '\r')
    {
      Home_S1_pos = ((inputString[3] - 48) * 100) + ((inputString[4] - 48) * 10) + (inputString[5] - 48);
      Home_S2_pos = ((inputString[7] - 48) * 100) + ((inputString[8] - 48) * 10) + (inputString[9] - 48);
      Home_S3_pos = ((inputString[11] - 48) * 100) + ((inputString[12] - 48) * 10) + (inputString[13] - 48);
      Home_S4_pos = ((inputString[15] - 48) * 100) + ((inputString[16] - 48) * 10) + (inputString[17] - 48);
      Home_S5_pos = ((inputString[19] - 48) * 100) + ((inputString[20] - 48) * 10) + (inputString[21] - 48);
      Home_S6_pos = ((inputString[23] - 48) * 100) + ((inputString[24] - 48) * 10) + (inputString[25] - 48);

      S1_pos = Home_S1_pos;
      S2_pos = Home_S2_pos;
      S3_pos = Home_S3_pos;
      S4_pos = Home_S4_pos;
      S5_pos = Home_S5_pos;
      S6_pos = Home_S6_pos;

      EEPROM.write(0, Home_S1_pos);
      EEPROM.write(1, Home_S2_pos);
      EEPROM.write(2, Home_S3_pos);
      EEPROM.write(3, Home_S4_pos);
      EEPROM.write(4, Home_S5_pos);
      EEPROM.write(5, Home_S6_pos);
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  if (S1_pos > S1_Upper_Bound) S1_pos = S1_Upper_Bound;
  else if (S1_pos < S1_Lower_Bound) S1_pos = S1_Lower_Bound;
  if (S2_pos > S2_Upper_Bound) S2_pos = S2_Upper_Bound;
  else if (S2_pos < S2_Lower_Bound) S2_pos = S2_Lower_Bound;
  if (S3_pos > S3_Upper_Bound) S3_pos = S3_Upper_Bound;
  else if (S3_pos < S3_Lower_Bound) S3_pos = S3_Lower_Bound;
  if (S4_pos > S4_Upper_Bound) S4_pos = S4_Upper_Bound;
  else if (S4_pos < S4_Lower_Bound) S4_pos = S4_Lower_Bound;
  if (S5_pos > S5_Upper_Bound) S5_pos = S5_Upper_Bound;
  else if (S5_pos < S5_Lower_Bound) S5_pos = S5_Lower_Bound;
  if (S6_pos > S6_Upper_Bound) S6_pos = S6_Upper_Bound;
  else if (S6_pos < S6_Lower_Bound) S6_pos = S6_Lower_Bound;

  mServo1->writeServo(S1_pos);
  servo1.write(S1_pos);
  mServo2->writeServo(S2_pos);
  mServo3->writeServo(S3_pos);
  mServo4->writeServo(S4_pos);
  mServo5->writeServo(S5_pos);
  mServo6->writeServo(S6_pos);

  Encoder_FeedBack = "FB,";

  if (S1_pos < 10) Encoder_FeedBack += "00";
  else if (S1_pos < 100) Encoder_FeedBack += "0";
  Encoder_FeedBack += String(S1_pos) + ",";

  if (S2_pos < 10) Encoder_FeedBack += "00";
  else if (S2_pos < 100) Encoder_FeedBack += "0";
  Encoder_FeedBack += String(S2_pos) + ",";

  if (S3_pos < 10) Encoder_FeedBack += "00";
  else if (S3_pos < 100) Encoder_FeedBack += "0";
  Encoder_FeedBack += String(S3_pos) + ",";

  if (S4_pos < 10) Encoder_FeedBack += "00";
  else if (S4_pos < 100) Encoder_FeedBack += "0";
  Encoder_FeedBack += String(S4_pos) + ",";

  if (S5_pos < 10) Encoder_FeedBack += "00";
  else if (S5_pos < 100) Encoder_FeedBack += "0";
  Encoder_FeedBack += String(S5_pos) + ",";

  if (S6_pos < 10) Encoder_FeedBack += "00";
  else if (S6_pos < 100) Encoder_FeedBack += "0";
  Encoder_FeedBack += String(S6_pos) + ",";

  Encoder_FeedBack += "\r";

  Serial.println(Encoder_FeedBack);
  delay(10);
}

void joystickSetup() {
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);
  if (error == 0)
  {
    Serial.println("Found Joy Stick, configured successful");
  }
  else if (error == 1)
  {
    Serial.println("No controller found, check wiring,");
  }
  else if (error == 2)
  {
    Serial.println("Controller found but not accepting commands.");
  }
  else if (error == 3)
  {
    Serial.println("Controller refusing to enter Pressures mode, may not support it.");
  }
  else // Error with Joy Stick
  {
    Serial.println("Not Found Joy Stick");
    while (1);
  }
  type = ps2x.readType();
  switch (type)
  {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }
}

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\r')
    {
      stringComplete = true;
    }
  }
}
