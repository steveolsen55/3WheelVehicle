/*
     James Beaver, Anthony G., Stephen A.
     May 2, 2017
     ETEC 290 Capstone project

    Robot control program.
      Hand controller joystick position communication via XBee 802.15 radio protocol.
      A 8 bit state variable is used to track status of LEDs, button presses, and other controls.
      If the Up/Down joystick is pushed forward --> throttle control
                           ... pulled back    --> throttle to zero and apply brake
      Left/Right joystick input is currently being limited to 40% of full range in the Hand Controller program.

    Hardware setup:
      Arduino Pro Mini 5v 16MHz microcontroller,
      XBee series 1 1mw U.FL Connection, on a SparkFun 'XBee Explorer Regulated' board
      two continuous rotation servos for testing.
      LEDs for status.
      LiPo 22.6v battery, regulated to 5v for electronics.

      Robot will use an 80mm Electric Ducted Fan (EDF) with an Electronic Speed Control (ESC) for propulsion.
      Braking will be provided by servo controlled mechanical brake on the rear wheel.
      Steering is provided by servo controlled rack and pinnion steering mechanism on the front wheels.

      TBD - EDF / ECS throttle limit, proportionally to turn rate, max un-boosted limit TBD

*/

#include <Servo.h>

Servo throttleMotor;
Servo steeringMotor;
Servo brakeMotor;

const byte RED_LED = 3;      // robot OFF
const byte GREEN_LED = 4;    // robot ON
const byte BLUE_LED = 5;     // turbo ON

const int MOTOR_PIN_THROTTLE = 10;
const int MOTOR_PIN_STEERING = 11;
const int MOTOR_PIN_BRAKE = 12;

const int MOTOR_VALUE_MIN = 0;
const int MOTOR_VALUE_CENTER = 90;         // servo position for center position
const int MOTOR_VALUE_MAX = 180;
const int MOTOR_VALUE_FULL_BRAKE = 45;       // servo position for full braking
const int MOTOR_VALUE_NO_BRAKE = 90;         // servo position for no brake applied
const int MOTOR_VALUE_THROTTLE_ZERO = 0;     // electronic speed control input value for throttle off
const int MOTOR_VALUE_THROTTLE_MIN = 0;      // electronic speed control input value for minimum thrust
const int MOTOR_VALUE_THROTTLE_MAX = 70;     // maximum non-turbo throttle limit
const int MOTOR_VALUE_THROTTLE_TURBO = 100;  // maximum turbo boosted throttle limit

const int NUMBER_OF_BYTES_IN_A_COMMAND = 8;
const int SERIAL_COMMAND_SET_CMD = 252;          // serial data code - next byte is a command byte
const int SERIAL_COMMAND_SET_THROTTLE = 253;     // serial data code - next byte is throttle setting
const int SERIAL_COMMAND_SET_STEERING_POS = 254; // serial data code - next byte is steering position
const int SERIAL_COMMAND_SET_BRAKE_POS = 255;    // serial data code - next byte is brake setting

const int COMM_LOSS_LIMIT = 200;     //  If no data acquired in 200 ms ==>  loss of communication

const long SERIAL_DATA_SPEED_BPS = 57600;   // Baud rate = 57600 for Capstone Xbee's

byte debug = 1;      //  set to 1 to send debug output to Serial Monitor

int throttleMotorVal;       //
int steeringMotorVal;
int brakeMotorVal;

byte state_machine = 0x00;  //  Bit 0 = Robot enabled, set for ON, clear for OFF
//  Bit 1 = turbo boost, set for ON, clear for OFF
//  Set each bit to 1 when on.  This var tracks the state machine status.

long communication_loss_timer;    // How long has it been with Serial.available = 0?

void setup()
{
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  for (int i = 0; i<4; i++)
  {
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    delay(100);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    delay(100);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
    delay(100);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
    delay(100);
  }

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(BLUE_LED, LOW);
    
  throttleMotor.attach(MOTOR_PIN_THROTTLE);
  steeringMotor.attach (MOTOR_PIN_STEERING);
  brakeMotor.attach(MOTOR_PIN_BRAKE);
  stopRobot();

  //  -->>  initialize the Electronic Speed Controller (ESC) here <<--

  throttleMotor.write(0);
  delay (20);
  throttleMotor.write(180);
  delay (10);
  throttleMotor.write(0);

  communication_loss_timer = millis();
  
  Serial.begin(SERIAL_DATA_SPEED_BPS);
}

void loop()
{
  static char throttleMotor = 0;
  static char steeringMotor = 0;
  static char brakeMotor = 0;
  
  //  if(debug == 1)
  //      Serial.println( Serial.available());

  if (Serial.available() >= NUMBER_OF_BYTES_IN_A_COMMAND)
  {
    int incomingByte = Serial.read();

    //  if(debug == 1)
    //    Serial.println( incomingByte);

    switch (incomingByte)
    {
      case SERIAL_COMMAND_SET_CMD:
      {
          state_machine = Serial.read();
          break;
      }
      case  SERIAL_COMMAND_SET_THROTTLE:
      {
         throttleMotor = Serial.read();
         break;
      }
      case SERIAL_COMMAND_SET_STEERING_POS:
      {
        steeringMotor = Serial.read();
        break;
      }
      case SERIAL_COMMAND_SET_BRAKE_POS:
      {
         brakeMotor = Serial.read();
         break;
      }
      communication_loss_timer = millis();
    }
    if ( communication_loss_timer > COMM_LOSS_LIMIT )
    {
       stopRobot();                                    // stop the robot immediately
       bitClear(state_machine, 0);                     // clear the robot enabled bit
       throttleMotorVal = MOTOR_VALUE_THROTTLE_ZERO;   // set the throttle var to off 
       steeringMotorVal = MOTOR_VALUE_CENTER;          // center the steering servo var
       brakeMotorVal = MOTOR_VALUE_FULL_BRAKE;         // set brake var to full brake
       digitalWrite(GREEN_LED, LOW);                   // turn off the robot enabled light
       digitalWrite(RED_LED, HIGH);                    // turn on the robot disabled light
    }
/*  
    if (debug == 1)
    {
      Serial.print("state_machine ="); Serial.print(state_machine, BIN);
      Serial.print("\t");
      Serial.print("throttle = "); Serial.print(throttleMotor);
      Serial.print("\t");
      Serial.print("steering = "); Serial.print(steeringMotor);
      Serial.print("\t");
      Serial.print("brake = "); Serial.println(brakeMotor);
    }
*/
  }
  if ( bitRead(state_machine, 0) == true ) // Green status: pulse servos
  {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    motor_setValues(throttleMotor, steeringMotor, brakeMotor);
  }
  if ( bitRead(state_machine, 0) == false )  //  All Stop!!
  {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
    stopRobot();
  }
}
//************************ Subroutines ****************************

void motor_setValues (int throttle, int steering, int brake)
{
  if (throttle == 0)
  {
    throttleMotorVal = MOTOR_VALUE_THROTTLE_ZERO;
  }
  else
  {
    throttleMotorVal = map(throttle,0,100,MOTOR_VALUE_THROTTLE_MIN,MOTOR_VALUE_THROTTLE_MAX );

    if ( bitRead(state_machine, 1) == true )  // turbo boost on
    {
      throttleMotorVal = MOTOR_VALUE_THROTTLE_TURBO;
      digitalWrite(BLUE_LED, HIGH);
    }
    else
    {
      digitalWrite(BLUE_LED, LOW);
    }
  }

  if (steering == 0)
  {
    steeringMotorVal = MOTOR_VALUE_CENTER;
  }
  else
  {
    steeringMotorVal = map(steering, -100, 100, MOTOR_VALUE_MIN, MOTOR_VALUE_MAX);
  }
  if (brake == 0)
  {
    brakeMotorVal = MOTOR_VALUE_CENTER;
  }
  else
  {
    brakeMotorVal = map(brake, -100, 0, MOTOR_VALUE_NO_BRAKE, MOTOR_VALUE_FULL_BRAKE);
  }

  if (debug == 1)
  { 
     Serial.print("throttle = "); Serial.print(throttleMotorVal);
     Serial.print("\t");
     Serial.print("steering = "); Serial.print(steeringMotorVal);
     Serial.print("\t");
     Serial.print("brake = "); Serial.println(brakeMotorVal);
  }
  else
  {
    throttleMotor.write(throttleMotorVal);
    steeringMotor.write(steeringMotorVal);
    brakeMotor.write(brakeMotorVal);
  }
}

void stopRobot ()
{
  throttleMotor.write(MOTOR_VALUE_THROTTLE_ZERO);
  steeringMotor.write(MOTOR_VALUE_CENTER);
  brakeMotor.write(MOTOR_VALUE_FULL_BRAKE);
}


