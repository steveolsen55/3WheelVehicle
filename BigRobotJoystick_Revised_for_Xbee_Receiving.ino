/*  
 *   
 *   Robot control program.  
 *    Hand controller joystick position communication via XBee 802.15 radio protocol.    
 *    A 8 bit state variable is used to track status of LEDs, button presses, and other controls.
 *    If the Up/Down joystick is pushed forward --> throttle control
 *                         ... pulled back    --> throttle to zero and apply brake
 *    Left/Right joystick input is currently being limited to 40% of full range in the Hand Controller program.  
 *    
 *  Hardware setup:
 *    Arduino Pro Mini 5v 16MHz microcontroller, 
 *    XBee series 1 1mw U.FL Connection, on a SparkFun 'XBee Explorer Regulated' board
 *    two continuous rotation servos for testing.    
 *    LEDs for status.
 *    LiPo 22.6v battery, regulated to 5v for electronics.
 *    
 *    Robot will use an 80mm Electric Ducted Fan (EDF) with an Electronic Speed Control (ESC) for propulsion.
 *    Braking will be provided by servo controlled mechanical brake on the rear wheel.
 *    Steering is provided by servo controlled rack and pinnion steering mechanism on the front wheels.
 *   
 *    TBD - Turbo boost control, to provide full speed boost on straight segments
 *        - EDF / ECS throttle limit, proportionally to turn rate, max un-boosted limit TBD
 *        - Loss of communication must set throttle to zero and apply brakes.
 *   
 *   
 */

#include <Servo.h> 

Servo leftMotor;
Servo rightMotor;

const int RED_LED = 3;
const int GREEN_LED = 4;

const int MOTOR_PIN_LEFT = 8;
const int MOTOR_PIN_RIGHT = 7;

const int MOTOR_VALUE_MIN = 50;
const int MOTOR_VALUE_MAX = 130;
const int MOTOR_VALUE_STOP = 90;

const int NUMBER_OF_BYTES_IN_A_COMMAND = 6;
const int SERIAL_COMMAND_SET_CMD = 253;
const int SERIAL_COMMAND_SET_LEFT_MOTOR = 254;
const int SERIAL_COMMAND_SET_RIGHT_MOTOR = 255;

const long SERIAL_DATA_SPEED_BPS = 57600;   // Baud rate = 57600 for Capstone Xbee's

byte debug = 0;      //  set to 1 to send debug output to Serial Monitor

int leftMotorVal;
int rightMotorVal;

byte state_machine;  // binary 00 = both LEDs off, Red LED is bit 1, Green LED is bit 0.  
                     //  Set each bit to 1 when on.  This var tracks the state machine status.

void setup()
{
    pinMode(RED_LED,OUTPUT);            
    pinMode(GREEN_LED,OUTPUT);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);    
    leftMotor.attach (MOTOR_PIN_LEFT);
    rightMotor.attach(MOTOR_PIN_RIGHT);
    stopRobot();
    Serial.begin(SERIAL_DATA_SPEED_BPS);
}

void loop()
{
  static char rightMotor = 0;
  static char leftMotor = 0;

//  if(debug == 1)
//      Serial.println( Serial.available());

  if (Serial.available() > NUMBER_OF_BYTES_IN_A_COMMAND)
  {
    int incomingByte = Serial.read(); 
    
//  if(debug == 1)
//    Serial.println( incomingByte);

    if(SERIAL_COMMAND_SET_CMD == incomingByte)
    {
       state_machine = Serial.read();
    }
    if(SERIAL_COMMAND_SET_LEFT_MOTOR == incomingByte)
    { 
      leftMotor = Serial.read();
    }
    if(SERIAL_COMMAND_SET_RIGHT_MOTOR == incomingByte)
    {
      rightMotor = Serial.read();
    }
   }
   if( (state_machine & 0x03) == B01)   // Green status: pulse servos
   {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, LOW);  
      motor_setValues(leftMotor,rightMotor);   
   }
   if( (state_machine & 0x03) == B10)    //  All Stop!!
   {
       digitalWrite(GREEN_LED, LOW);
       digitalWrite(RED_LED, HIGH);
       stopRobot();
   }
}
//************************ Subroutines ****************************

//   This subroutine will need to be replaced, as we're not using powered wheels.
//
void motor_setValues (int normalizedLeft, int normalizedRight)
{
  if (normalizedLeft == 0)
  {
    leftMotorVal = MOTOR_VALUE_STOP;
  }
  else
  {
     leftMotorVal = map(normalizedLeft,100,-100,MOTOR_VALUE_MAX,MOTOR_VALUE_MIN);
  }
  
  if (normalizedRight == 0)
  {
     rightMotorVal = MOTOR_VALUE_STOP;
  }
  else
  {
     rightMotorVal = map(normalizedRight,100,-100,MOTOR_VALUE_MIN,MOTOR_VALUE_MAX);
  }

  if(debug == 1)
  {
     Serial.print("leftMotorVal = "); Serial.print(leftMotorVal);
     Serial.print("\t");
     Serial.print("rightMotorVal = "); Serial.println(rightMotorVal);
  }
  else
  {
    leftMotor.write(leftMotorVal);
    rightMotor.write(rightMotorVal);
  }
}

void stopRobot ()
{
   leftMotor.write(MOTOR_VALUE_STOP);
   rightMotor.write(MOTOR_VALUE_STOP);
}
  
  
