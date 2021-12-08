#include <Arduino.h>
#include <analogWrite.h>
#include <AccelStepper.h>
#include <string.h>
#include "DRV8825.h"

// Stepper motor pin definitions
#define dirPin 12
#define stepPin 13
#define MS0 27
#define MS1 33
#define MS2 15
#define motorInterfaceType 1
#define MAX_MESSAGE_LENGTH 12
#define CMD_LENGTH 3

// DC motor pin definitions
#define ENCA 32 // YELLOW
#define ENCB 14 // WHITE
#define IN2 4
#define IN1 21

//PID global variables
volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
long target = 0;
long dist = 0;

//Stepper rotation angle 
long angle = 0;
int angle_prev = angle;

//command token buffers
char *cmd; 
char *val;


//Function headers
void setMotor(int dir, int pwmVal, int in1, int in2);
void readEncoder();

DRV8825 step2(200, dirPin, stepPin, MS0, MS1, MS2);

void setup() {
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  step2.begin(43, 32);
  step2.setSpeedProfile(DRV8825::LINEAR_SPEED, 2000, 2000);
  
  Serial.println("target pos");
}

void loop() {

  // set target positions
  
  static char message[MAX_MESSAGE_LENGTH];
  
  //char *cmdbuf;
  //char *valbuf;
  
  
  //char cmd[CMD_LENGTH];
  //char val[MAX_MESSAGE_LENGTH - CMD_LENGTH];
  static unsigned int message_pos = 0;
  
  while (Serial.available() > 0){
    char inByte = Serial.read();
    if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)){
      message[message_pos] = inByte;
      message_pos++;  
    } else {
      message[message_pos] = '\0';
      message_pos = 0;
      Serial.println(message);
      cmd = strtok(message, " ");
      Serial.printf("%s\n", cmd);  
      val = strtok(NULL, " ");
      Serial.printf("%s\n", val);
      if (strcmp(cmd, "DC") == 0) {
        Serial.println("feeding DC motor dist");
        dist = atol(val);
        target = dist;
      } else if (strcmp(cmd, "STP") == 0) {
        Serial.println("rotating base to angle");
        angle_prev = angle;
        angle = atol(val);
      }
    }
  }
  
  
  //strncpy(cmd, message, CMD_LENGTH);
  //cmd[CMD_LENGTH] = '\0';
  //strncpy(val, message+CMD_LENGTH, sizeof(val));

  // DC [dist]
  // STP [angle]
  
  //int target = 250*sin(prevT/1e6);
  //int target = -1200;

  // PID constants
  float kp = 1;
  float kd = 0.0;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // interrupt for encoder pos reading
  int pos = 0; 
  noInterrupts();
  pos = posi;
  interrupts();
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, IN1, IN2);
  step2.rotate(angle);

  // store previous error
  eprev = e;

  
  Serial.print(target);
  Serial.print(" ");
  Serial.print(angle);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  
}

void setMotor(int dir, int pwmVal, int in1, int in2){
  if(dir == 1){
    analogWrite(in1,pwmVal, 255);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    analogWrite(in2,pwmVal, 255);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}