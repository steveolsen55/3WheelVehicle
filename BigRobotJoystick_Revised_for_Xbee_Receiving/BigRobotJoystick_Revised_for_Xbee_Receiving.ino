/*
 * test
 * 
 */

#include <Servo.h> 

Servo leftMotor;
Servo rightMotor;


const int RED_LED = 11;
const int GREEN_LED = 12;

const int BUTTON = A5;             


const int MOTOR_PIN_LEFT = 8;
const int MOTOR_PIN_RIGHT = 11;


const int MOTOR_VALUE_MIN = 50;
const int MOTOR_VALUE_MAX = 130;
const int MOTOR_VALUE_STOP = 90;

const int NUMBER_OF_BYTES_IN_A_COMMAND = 4;
const int SERIAL_COMMAND_SET_LEFT_MOTOR = 254;
const int SERIAL_COMMAND_SET_RIGHT_MOTOR = 255;

const long SERIAL_DATA_SPEED_38400_BPS = 38400;

int leftMotorVal;
int rightMotorVal;

 boolean turnOnOff = 0;
 int valButton;
 
 void setup()
{
    Serial.begin(SERIAL_DATA_SPEED_38400_BPS);
    
    
    pinMode(RED_LED,OUTPUT);            
    pinMode(GREEN_LED,OUTPUT);
    
    
    leftMotor.attach (MOTOR_PIN_LEFT);
    leftMotor.write(MOTOR_VALUE_STOP);
    
   
    rightMotor.attach(MOTOR_PIN_RIGHT);
    rightMotor.write(MOTOR_VALUE_STOP);
    
    
}

void loop()
{
  static char rightMotor = 0;
  static char leftMotor = 0;
  //Serial.println( Serial.available());
  if (Serial.available() > NUMBER_OF_BYTES_IN_A_COMMAND)
  {
    int incomingByte = Serial.read(); 
    Serial.println( incomingByte);
    if(SERIAL_COMMAND_SET_LEFT_MOTOR == incomingByte)
    {
      leftMotor = Serial.read();
    }
    if(SERIAL_COMMAND_SET_RIGHT_MOTOR == incomingByte)
    {
      rightMotor = Serial.read();
    }
   }
   motor_setValues(leftMotor,rightMotor);
}
//************************ Subroutines ****************************


void motor_setValues (int normalizedLeft, int normalizedRight)
{
  if (normalizedLeft == 0)
  {
    leftMotorVal = MOTOR_VALUE_STOP;
  }
  else
  {
     leftMotorVal = map(normalizedLeft,-100,100,MOTOR_VALUE_MIN,MOTOR_VALUE_MAX);
  }
  
   if (normalizedRight == 0)
  {
     rightMotorVal = MOTOR_VALUE_STOP;
  }
  else
  {
     rightMotorVal = map(normalizedRight,-100,100,MOTOR_VALUE_MIN,MOTOR_VALUE_MAX);
  }
  
  leftMotor.write(leftMotorVal);
  rightMotor.write(rightMotorVal);
}
  
