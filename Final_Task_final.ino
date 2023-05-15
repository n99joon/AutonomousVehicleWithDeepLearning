#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#include <Servo.h>
Servo myServo;
//*****************************************************************************//
//talkie and weight scale
#include "HX711.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Talkie.h>
#include "Vocab_US_Large.h"
#include "Vocab_US_TI99.h"
Talkie voice;

const int LOADCELL_DOUT_PIN = 6;
const int LOADCELL_SCK_PIN = 10;
const int OLED_RESET_PIN = -1;

HX711 scale;
Adafruit_SSD1306 oled(OLED_RESET_PIN);
///////////////////////////////////////////////////////////////////////////////////////
//Declaration for Ultrasonic sensor
#define echoPin 22
#define trigPin 24
long distance_in_cm=100;
/////////////////////////////////////////////////////////////////////////////////
//IR sensors
const int leftSensor = A0;
const int rightSensor = A15;
// Threshold value for the line-tracking sensors (adjust as needed)
const int threshold = 300;
int stage=0;
int pills[2];
float powder=0;
int opened = 0;
//****************************************************************************//
const int buttonPin =4;
const int grnLedPin = 13;
//int butUp=11;
//int butDown =13;
//int butOk=14;
volatile int lastButtonState = HIGH;
volatile int buttonState;
//volatile int ledToggle = LOW;
////////////////////////////////////////////////////////////////////////////////////
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 1900;


//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}


/*Voltage Readings transmitter
Sends them via Serial3*/
void sendVolt(){
    newV = analogRead(A0);
    if(newV!=oldV) {
      if (!Serial3.available()) {
        //Serial3.println(newV);
        //Serial.println(newV);
      }
    }
    oldV=newV;
}


//Where the program starts
void setup()
{
  ////////////////////////////////////////////////////////
  //speaker
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  // Initialize library with data output pin, clock input pin and gain factor.
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Set the scale factor and tare the scale.
  scale.set_scale(2280.f); // adjust this value to calibrate your load cell
  scale.tare();
  //////////////////////////////////////////////////////////
  //button
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(grnLedPin, OUTPUT);
//  pinMode(butUp, INPUT_PULLUP);
//  pinMode(butDown, INPUT_PULLUP);
//  pinMode(butOk, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(buttonPin), button_ISR, CHANGE);
  ////////////////////////////////////////////////////////////
  //Ultrasonic pins
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  ////////////////////////////////////////////////////////////
  myServo.attach(33);
  myServo.write(100); //Set position as lock (15=open)
  //////////////////////////////////////////////////////
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  STOP(); // Stop the robot
  //Serial3.begin(9600); // BT serial setup
  //Pan=PL4=>48, Tilt=PL5=>47
//   servo_pan.attach(48);
//   servo_tilt.attach(47);
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("AI Robot");
  display.display();


  //Setup Voltage detector
  pinMode(A0, INPUT);
  powder = scale.get_units(10);
}


////////////////////////////////////////////////////////////////////////////

void imageDetection(){
  //send signal to Jetson Nano to start image detection
  STOP();
  Serial.println(1);
  delay(3000);
  if(Serial.available()>0){
    //Get Data from Jetson Nano (Update the number of pills)
      String mid="";
     char letter;
      while(Serial.available()){
       letter=Serial.read();
       mid += letter;       
      }
      //finished update, move to next stage
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
     display.println(mid.substring(0));
     display.display();
     //Serial.println(mid);
     int spaceIndex = mid.indexOf(' '); // find the index of the space character
     if (spaceIndex != -1) { // if the space character was found
        pills[0] = mid.substring(0, spaceIndex).toInt(); // extract the first integer and convert it to an int
        pills[1] = mid.substring(spaceIndex + 1).toInt(); // extract the second integer and convert it to an int
      }
      stage = 3;
   }
   delay(10000);
}
//*****************************************************************//
//void button_ISR()
//{
//  Serial.println("reached push button");
//  buttonState = digitalRead(buttonPin);
//  // The variables are reset back to it's initial state once released. Hence, we use this condition.
//  if (buttonState == LOW && lastButtonState == HIGH) {
//    stage = 1;
//    //ledToggle = !ledToggle;
//    //digitalWrite(grnLedPin, ledToggle);
//  }
//  lastButtonState = buttonState;
//}
void check(){
  myServo.write(100);
  if (opened == 0){
    //password
    Serial.println(2);
    if(Serial.available()>0){
    //Get Data from Jetson Nano (Update the number of pills)
      String mid="";
     char letter;
      while(Serial.available()){
       letter=Serial.read();
       mid += letter;       
      }
      //if barcode scanned
      if (mid[0]=='9') {
        myServo.write(15); //Set position as lock (15=open)
        opened=1;
      }
  } 
  }
}

void weightMeasure(){//stage 3
  
  stage = 4;
  delay(500);
}
void speak(){//stage 4
  //speak pills[0,1], powder
  for(int i=0;i<5;i++){
    voice.say(sp3_GREEN);
    switch (pills[0]) {
    case 0:
      voice.say(sp2_ZERO);
      break;
    case 1:
      voice.say(sp2_ONE);
      break;
    case 2:
      voice.say(sp2_TWO);
      break;
    case 3:
      voice.say(sp2_THREE);
      break;
    case 4:
      voice.say(sp2_FOUR);
      break;
    case 5:
      voice.say(sp2_FIVE);
      break;
    case 6:
      voice.say(sp2_SIX);
      break;
    case 7:
      voice.say(sp2_SEVEN);
      break;
    case 8:
      voice.say(sp2_EIGHT);
      break;
    case 9:
      voice.say(sp2_NINE);
      break;
    default:
      // Handle an invalid value of pills[0] here
      break;
  }
  voice.say(sp2_LEFT);
  delay(1000);
  voice.say(sp3_WHITE);
    switch (pills[0]) {
    case 0:
      voice.say(sp2_ZERO);
      break;
    case 1:
      voice.say(sp2_ONE);
      break;
    case 2:
      voice.say(sp2_TWO);
      break;
    case 3:
      voice.say(sp2_THREE);
      break;
    case 4:
      voice.say(sp2_FOUR);
      break;
    case 5:
      voice.say(sp2_FIVE);
      break;
    case 6:
      voice.say(sp2_SIX);
      break;
    case 7:
      voice.say(sp2_SEVEN);
      break;
    case 8:
      voice.say(sp2_EIGHT);
      break;
    case 9:
      voice.say(sp2_NINE);
      break;
    default:
      // Handle an invalid value of pills[0] here
      break;
  }
  voice.say(sp2_LEFT);
  }
  
  stage=5;
}

//stage5
void lineTrack(){
  //track line
  int leftSensorValue = analogRead(leftSensor);
  int rightSensorValue = analogRead(rightSensor);
// Serial.print("Left: ");
// Serial.print(leftSensorValue);
//  Serial.print(" Right: ");
// Serial.println(rightSensorValue);
if (leftSensorValue > threshold && rightSensorValue > threshold) {
    // Both sensors on the black line - move forward
    Motor_PWM = 1830;
    ADVANCE();
    delay(100);
    STOP();
    delay(50);
  } else if (leftSensorValue > threshold && rightSensorValue < threshold) {
    // Left sensor off the line - turn left
    Motor_PWM = 1850;
    rotate_1();
    delay(100);
    STOP();
    delay(50);
  } else if (leftSensorValue < threshold && rightSensorValue > threshold) {
    // Right sensor off the line - turn right
    Motor_PWM = 1850;
    rotate_2();
    delay(100);
    STOP();
    delay(50);
  } else {
    // Both sensors off the line - stop
    STOP();
    delay(50);
    stage=6;
    powder = scale.get_units(10);
  }
}
void delivered(){
  if (opened == 0){
    //password
    Serial.println(2);
    if(Serial.available()>0){
    //Get Data from Jetson Nano (Update the number of pills)
      String mid="";
     char letter;
      while(Serial.available()){
       letter=Serial.read();
       mid += letter;       
      }
      //if barcode scanned
      if (mid[0]=='9') {
        myServo.write(15); //Set position as lock (15=open)
        opened=1;
      }
    } 
  }
  else{
    while(true){
      //ask to take some weight
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Take powder by 0.5 to 0.7 g");
      display.println("and wait 5 seconds.");
      display.display();
      voice.say(spt_TAKE);
      delay(500);
      voice.say(spt_ZERO);
      voice.say(spt_POINT);
      voice.say(spt_FIVE);
      delay(500);
      voice.say(spt_TO);
      voice.say(spt_ZERO);
      voice.say(spt_POINT);
      voice.say(spt_SEVEN);
      voice.say(sp2_G);
      delay(500);
      delay(5000); // wait for 5 seconds
    float currentWeight = scale.get_units(10);
    delay(200);
    float weightReduction = powder - currentWeight;
    if (weightReduction > 0.7) {
      delay(1000);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Current weight: ");
      display.print(currentWeight);
      display.println(" g");
      display.println("Weight reduction too much!");
      display.display();
      voice.say(spt_TAKE);
      voice.say(spt_LESS);


      delay(2000);
    } else if (weightReduction < 0.5) {
      voice.say(spt_TAKE);
      voice.say(spt_MORE);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Current weight: ");
      display.print(currentWeight);
      display.println(" g");
      display.println("Weight reduction too little.");
      display.display();
      voice.say(spt_TAKE);
      voice.say(spt_MORE);
      delay(2000);
    } else {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Current weight: ");
      display.print(currentWeight);
      display.println(" g");
      display.println("Weight within range. Please take the powder");
      display.display();
      voice.say(spt_DONE);
      voice.say(spt_PLEASE);
      voice.say(spt_TAKE);
      break;
    }
    }
  }
}
void loop(){
  // run the code in every 20ms
  if (millis() > (time + 15)) {
    voltCount++;
    time = millis();
   
//    //constrain the servo movement
//    pan = constrain(pan, servo_min, servo_max);
//    tilt = constrain(tilt, servo_min, servo_max);
//    
//    //send signal to servo
//    servo_pan.write(pan);
//    servo_tilt.write(tilt);

    //***************************************************//
    //Serial.print("stage=");
    //Serial.println(stage);
    //*******************************//
    //button
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW && lastButtonState == HIGH) {
      if(stage==0)stage=1;
      else{
        //close and do image detection
            stage=2;
            opened=0;         
      }
      //Serial.println("success");
    }
    lastButtonState = buttonState;
    //open - 15, locked - 100
    if(opened==0){
      myServo.write(100);
    }else{
      myServo.write(15);
    }
    //////////////////////////////////////////////////////////////////
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println(opened?"OPEN":"LOCKED");
    display.display();
    switch(stage){
      case 0: //do nothing
        break;
      case 1: //do image detection
        check();
        break;
      case 2:
        imageDetection();
        break;
      case 3:
        weightMeasure();
        break;
      case 4:
        speak();
        break;
      case 5: //delivery
        lineTrack();
        break;
      case 6: //delivered
        delivered();
        break;
      default:
        break;

        
       
        
    }
    
    //***************************************************//
  }if (voltCount>=5){
    voltCount=0;
    sendVolt();
    
  }
  
}
