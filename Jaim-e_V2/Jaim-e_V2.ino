#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>

//L293D Connections
const int leftMotorA = 2;
const int leftMotorB = 3;
const int rightMotorA = 4;
const int rightMotorB = 5;
//HC-SR04 Connections & var
const int hcEcho = 7;
const int hcTrig = 8;
int duration = 0;
float distance = 0;
//TowerPro MicroServo
Servo servoHead;
const int servoHeadPin = 9;
int servoAngle = 0;
//RGB LED Connections
const int redPin = 10;
const int greenPin = 11;
const int bluePin = 12;
//LCD Screen connections

void setup() {
  // L293D
  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  //HCSR04
  pinMode(hcEcho, INPUT);
  pinMode(hcTrig, OUTPUT);
  //RGB LED
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  //MiniServoHead
  servoHead.attach(servoHeadPin);
  Serial.begin(9600);
}

void loop() {
   //Sensor->Send pulse
   digitalWrite(hcTrig , HIGH);
   delayMicroseconds(50);
   digitalWrite(hcTrig , LOW);
   //Sensor->measure distance
   duration = pulseIn(hcEcho , HIGH);
   distance = (duration/2) / 28.5 ;
   Serial.println(distance);

    if ( distance < 25 && distance > 0)
  {
    goBackTurn();
  }
  else if ( distance > 25 || distance < 0)
  {
    goForward();
  }
  //Look north
   servoHead.write(80);
   delay(500);

   digitalWrite(hcTrig , HIGH);
   delayMicroseconds(50);
   digitalWrite(hcTrig , LOW);
   duration = pulseIn(hcEcho , HIGH);
   //distance = (duration/2) / 29.7 ;
   distance = duration / 58.0;
   Serial.println(distance);

   if ( distance < 25 && distance > 0)
  {
    goBackTurn();
  }
  else if ( distance > 25 || distance < 0)
  {
    goForward();
  }

  delay(500);

   digitalWrite(hcTrig , HIGH);
   delayMicroseconds(50);
   digitalWrite(hcTrig , LOW);
   duration = pulseIn(hcEcho , HIGH);
   //distance = (duration/2) / 29.7 ;
   distance = duration / 58.0;
   Serial.println(distance);

    if ( distance < 25 && distance > 0)
  {
    goBackTurn();
  }
  else if ( distance > 25 || distance < 0)
  {
    goForward();
  }
  //look east
   servoHead.write(135);
   delay(500);

   digitalWrite(hcTrig , HIGH);
   delayMicroseconds(50);
   digitalWrite(hcTrig , LOW);
   duration = pulseIn(hcEcho , HIGH);
   distance = (duration/2) / 28.5 ;
   Serial.println(distance);

   if ( distance < 25 && distance > 0)
  {
    goBackTurn();
  }
  else if ( distance > 25 || distance < 0)
  {
    goForward();
  }

  delay(500);

  digitalWrite(hcTrig , HIGH);
   delayMicroseconds(50);
   digitalWrite(hcTrig , LOW);
   duration = pulseIn(hcEcho , HIGH);
   distance = (duration/2) / 28.5;
   Serial.println(distance);

    if ( distance < 25 && distance > 0)
  {
    goBackTurn();
  }
  else if ( distance > 25 || distance < 0)
  {
    goForward();
  }
  //look west
   servoHead.write(30);
   delay(500);

   digitalWrite(hcTrig , HIGH);
   delayMicroseconds(50);
   digitalWrite(hcTrig , LOW);
   duration = pulseIn(hcEcho , HIGH);
   distance = (duration/2) / 28.5 ;
   Serial.println(distance);

   if ( distance < 25 && distance > 0)
  {
    goBackTurn();
  }
  else if ( distance > 25 || distance < 0)
  {
    goForward();
  }

  delay(500);
}

//antena code
void rgbColor(int redColor, int greenColor, int blueColor) {
  analogWrite(redPin, redColor);
  analogWrite(greenPin, greenColor);
  analogWrite(bluePin, blueColor);
}

void goBackTurn() {
  //Go back
    digitalWrite(leftMotorA , 0);
    analogWrite(leftMotorB , 220);
    analogWrite(rightMotorA , 0);
    digitalWrite(rightMotorB , 220);
    rgbColor(0, 0, 255);
    delay(1000);
    //Stop
    digitalWrite(leftMotorA , 0);
    analogWrite(leftMotorB , 0);
    analogWrite(rightMotorA , 0);
    digitalWrite(rightMotorB , 0);
    delay(500);
    //Turn Around
    digitalWrite(leftMotorA , 0);
    analogWrite(leftMotorB , 220);
    analogWrite(rightMotorA , 220);
    digitalWrite(rightMotorB , 0);
    rgbColor(0, 0, 255);
    delay(500);
}

void goForward() {
    analogWrite(leftMotorA , 255);
    digitalWrite(leftMotorB , 0);
    analogWrite(rightMotorA , 255);
    digitalWrite(rightMotorB , 0);
    rgbColor(255, 0, 0);
}
