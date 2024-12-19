/*
 * ============================================================================
 * Final Project for ECE3 - Line Following Car with Turnaround Detection
 * ============================================================================
 * Project Name: Autonomous Line-Following Car with TI-RSLK MAX
 * 
 * Description:
 * This program controls a TI-RSLK MAX robot to follow a line using IR sensors 
 * and a PID-based control system. It detects intersections to perform 
 * turnarounds and uses timing constraints for navigation stability. 
 * 
 * Hardware Used:
 * - TI-RSLK MAX Robotics Kit
 * - MSP432 Microcontroller
 * - 8 Infrared (IR) Sensors for line detection
 * 
 * Features:
 * - PD control for smooth line-following
 * - Turnaround detection at endpoint and stop detection at starting point
 * - Adjustable speed and precision for stable navigation
 * 
 * Authors: Andrew Cardona, Oscar Rivera
 * Class: Introduction to Electrical Engineering (ECE 3) - Fall 2024
 * 
 * Notes:
 * - Ensure all connections to the TI-RSLK MAX are properly configured.
 * - Modify PID constants (Kp, Kd) to optimize performance on specific tracks.
 * - Modify built in timer function to ensure turnaround at desired point.
 */


/*------------------Start of Project------------------*/

#include <ECE3.h>

#define left_nslp_pin 31 // nslp ==> awake & ready for PWM
#define left_dir_pin 29
#define left_pwm_pin 40

#define right_nslp_pin 11 // nslp ==> awake & ready for PWM
#define right_dir_pin 30
#define right_pwm_pin 39

/*------------------Global Variables------------------*/
uint16_t sensorValues[8];

int base = 30;

float leftSpd = base;
float rightSpd = base;
int turnDelay = 200;
int delayTime = 50;

const float innerWheelCorrection = 0.6; 
const float outerWheelCorrection = 1.3; 

float Kp = -0.0275;
float Kd = -0.025;

float previousError = 0;

bool lastTurnAround = false;
bool hasTurned = false;

int timerStart = 0;

/*------------------Setup------------------*/

void setup()
{
  ECE3_Init();

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);

  //Serial.begin(9600); 
  delay(1000);
}

/*------------------Main Loop------------------*/

void loop()
{
  ECE3_read_IR(sensorValues);

  if (timerStart == 0)
  {
    timerStart = millis();
  }
  
    if (sensorTurnAround() && lastTurnAround && !hasTurned && millis() > 15000)
    {
      performTurnAround(turnDelay);

      lastTurnAround = false;
      hasTurned = true;

      increaseTo(base);
    }
    if (sensorTurnAround() && lastTurnAround && hasTurned && millis() > 52000)
    {
      stopCarForever();
    }

  lastTurnAround = sensorTurnAround();

  
  bool leftMotorDirect = true;
  bool rightMotorDirect = true;
  
  float measuredValue = combinedValues();

  float error = measuredValue;
  float derivative = (error - previousError);
 
  float output = Kp * error + Kd * derivative;

  float leftMotorSpeed = leftSpd + output;
  float rightMotorSpeed = rightSpd - output;

    if(leftMotorSpeed < 0) 
    {
      leftMotorDirect = 0;
      leftMotorSpeed *= innerWheelCorrection;
      rightMotorSpeed *= outerWheelCorrection; 
    }
    if(rightMotorSpeed < 0) 
    {
      rightMotorDirect = 0;   
      rightMotorSpeed *= innerWheelCorrection; 
      leftMotorSpeed *= outerWheelCorrection;
    }
  setSpeed(abs( leftMotorSpeed), abs( rightMotorSpeed), leftMotorDirect, rightMotorDirect);
  previousError = error;
}

/*------------------Functions------------------*/

//Note: The outer sensors were turned off for better navigation

int combinedValues()
{
  float newVal = 0;
  float arr[8]  = {0, -14, -12, -8, 8, 12, 14, 0};
    
    for (int i = 1; i < 7; i++)
  {
    newVal += arr[i] * sensorValues[i];
  }
  return newVal/8;
}

bool sensorTurnAround()
{
  int val = 0;
 
   for (int i = 1; i < 7; i++) 
    {
      val = val + sensorValues[i];
    }

   if (val > 12000)
    { 
      return true;
    }
  
  return false;
}

void setSpeed(int leftMotorSpeed, int rightMotorSpeed, bool leftMotorDirect, bool rightMotorDirect)
{
  if(leftMotorDirect) digitalWrite(left_dir_pin,LOW);
  else digitalWrite(left_dir_pin,HIGH);

  if(rightMotorDirect) digitalWrite(right_dir_pin,LOW);
  else digitalWrite(right_dir_pin,HIGH);

  analogWrite(left_pwm_pin, leftMotorSpeed);
  analogWrite(right_pwm_pin, rightMotorSpeed);
}

void performTurnAround(int delayTime)
{
  decreaseToStop();  
  
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, HIGH);

  analogWrite(left_pwm_pin, 255);
  analogWrite(right_pwm_pin, 255);

  delay(turnDelay);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);
}

void increaseTo(int base)
{
  for (int speed = 0 ; speed < base; speed += 20)
  {
    analogWrite(left_pwm_pin, speed);
    analogWrite(right_pwm_pin, speed);
    delay(5);
  }
}


void stopCarForever()
{
  decreaseToStop();
  
  while(true){}
}

void decreaseToStop()
{
  digitalWrite(right_dir_pin, HIGH);
  digitalWrite(left_dir_pin, HIGH);
  
  analogWrite(left_pwm_pin, 10);
  analogWrite(right_pwm_pin, 10);

  delay(10);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);

  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}

/*------------------End of Project------------------*/
