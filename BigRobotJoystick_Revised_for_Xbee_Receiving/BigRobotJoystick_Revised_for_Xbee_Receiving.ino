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
      Communication from the hand controller is done with an 8 byte data packet defined in the 
      'BigRobotJoystick_Revised_for_Xbee_Sending' program.    This program responds with an Ackownledge code to 
      ensure data communication is working properly.

    Hardware setup:
      Arduino Pro Mini 5v 16MHz microcontroller,
      XBee series 1 1mw U.FL Connection, mounted on a SparkFun 'XBee Explorer Regulated' board
      3 servo connections: Electronic Speed Control (ESC) - propulsion, steering, brake.
      LEDs for status.  Power: LiPo 22.6v battery, regulated to 5v for electronics.

      Robot will use an 80mm Electric Ducted Fan (EDF) with an Electronic Speed Control (ESC) for propulsion.
      Braking will be provided by servo controlled mechanical brake on the rear wheel.
      Steering is provided by servo controlled rack and pinnion steering mechanism on the front wheels.

      TBD - throttle limit, proportionally to turn rate, max un-boosted limit TBD

*/

#include <Servo.h>

Servo throttleMotor;
Servo steeringMotor;
Servo brakeMotor;

const byte RED_LED = 3;      // error condition
const byte GREEN_LED = 4;    // robot ON
const byte BLUE_LED = 5;     // turbo ON

const int MOTOR_PIN_THROTTLE = 12;
const int MOTOR_PIN_STEERING = 11;
const int MOTOR_PIN_BRAKE = 10;

const int MOTOR_VALUE_MIN = 0;
const int MOTOR_VALUE_CENTER = 90;         // servo position for center position
const int MOTOR_VALUE_MAX = 180;
const int MOTOR_VALUE_FULL_BRAKE = 45;       // servo position for full braking
const int MOTOR_VALUE_NO_BRAKE = 90;         // servo position for no brake applied
const int MOTOR_VALUE_THROTTLE_ZERO = 0;     // electronic speed control input value for throttle off
const int MOTOR_VALUE_THROTTLE_MIN = 0;      // electronic speed control input value for minimum thrust
const int MOTOR_VALUE_THROTTLE_MAX = 70;     // maximum non-turbo throttle limit
const int MOTOR_VALUE_THROTTLE_TURBO = 100;  // maximum turbo boosted throttle limit

const int NUMBER_OF_BYTES_IN_A_COMMAND = 8;      // serial data packet is 8 bytes
const int SERIAL_COMMAND_SET_ACK = 251;          // serial data code - next byte is response data
const int SERIAL_COMMAND_SET_CMD = 252;          // serial data code - next byte is a command byte
const int SERIAL_COMMAND_SET_THROTTLE = 253;     // serial data code - next byte is throttle setting
const int SERIAL_COMMAND_SET_STEERING_POS = 254; // serial data code - next byte is steering position
const int SERIAL_COMMAND_SET_BRAKE_POS = 255;    // serial data code - next byte is brake setting
const int NUMBER_OF_BYTES_TO_SEND = 2;           // serial data packet sent back size

const int COMM_LOSS_LIMIT = 200;     //  If no data acquired for specified ms ==>  loss of communication

const long SERIAL_DATA_SPEED_BPS = 38400;   // Baud rate for Capstone Xbee's

byte debug = 0;      //  set to 1 to send debug output to Serial Monitor

int throttleMotorVal;       //
int steeringMotorVal;
int brakeMotorVal;

int throttleVal;            //  temp variables
int steeringVal;
int brakeVal;

byte state_machine = 0x00;  //  Bit 0 = Robot enabled, set for ON, clear for OFF
                            //  Bit 1 = turbo boost, set for ON, clear for OFF
                            //  Set each bit to 1 when on.  This var tracks the state machine status.

char incomingBytes[NUMBER_OF_BYTES_IN_A_COMMAND];    // char type holds signed values -128 to 127
char outgoingBytes[NUMBER_OF_BYTES_TO_SEND];

boolean dataAcquired = false;     //  did read_Serial_Data receive data?

long communication_loss_timer;    // How long has it been with Serial.available = 0?

void setup()
{
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  flashLEDs();     //  cycle through the LEDs so we know they all work

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);      //  set Red LED on to show robot is disabled
  digitalWrite(BLUE_LED, LOW);

  outgoingBytes[0] = SERIAL_COMMAND_SET_ACK;   // 
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
   boolean robotEnabled;
   
   dataAcquired = read_Serial_Data();
   
/*
   if (debug == 1)
   { 
      Serial.print("dataAcquired ="); Serial.print(dataAcquired);
      Serial.print("\t");
      Serial.print("  sm = "); Serial.print(state_machine,BIN);
      Serial.print("\t");
      Serial.print("th = "); Serial.print(throttleMotorVal,DEC);
      Serial.print("\t");
      Serial.print("st = "); Serial.print(steeringMotorVal,DEC);
      Serial.print("\t");
      Serial.print("br = "); Serial.println(brakeMotorVal,DEC);
      delay(50);
   }
*/
   if ( (millis() - communication_loss_timer) > COMM_LOSS_LIMIT )
   {
      if( debug == 1 )
         Serial.println(" Communication lost ");
      
      stopRobot();                                    // stop the robot immediately
      throttleMotorVal = MOTOR_VALUE_THROTTLE_ZERO;   // set the throttle var to off 
      steeringMotorVal = MOTOR_VALUE_CENTER;          // center the steering servo var
      brakeMotorVal = MOTOR_VALUE_FULL_BRAKE;         // set brake var to full brake
      digitalWrite(GREEN_LED, LOW);                   // turn off the robot enabled light
      digitalWrite(RED_LED, HIGH);                    // turn on the error condition light
   }

  robotEnabled = bitRead(state_machine, 0);
  
  if ( robotEnabled )     // Green status: pulse servos
  {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    motor_setValues(throttleVal, steeringVal, brakeVal);
  }
  else                    //  All Stop!!
  {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
    stopRobot();
  }

  if (dataAcquired == false)
  {
          // Do nothing.  communication_loss_timer will keep lagging behind
  }   
  else    // data WAS acquired   
  {
     communication_loss_timer = millis();     // reset timer for next loop
  }   
}

//************************ Subroutines ****************************

boolean read_Serial_Data()
{  
   char garbage;
   int i,out;  

//   if(debug == 1)
//     Serial.println( Serial.available());
   
   if (Serial.available() >= NUMBER_OF_BYTES_IN_A_COMMAND)
   {
      while (Serial.peek() != SERIAL_COMMAND_SET_CMD)
          garbage = Serial.read(); 
   
      for ( i=0; i<NUMBER_OF_BYTES_IN_A_COMMAND; i++)   // capture incoming data
      {
         incomingBytes[i] = Serial.read();
      }
      outgoingBytes[1] = true;                              // capture comm status in response array
      for ( int out=0; out<NUMBER_OF_BYTES_TO_SEND; out++)  // send back response
      {
         Serial.write( outgoingBytes[out] );
      }
      
      if ( (incomingBytes[1] <= 0x03) && (incomingBytes[1] >= 0x00) )
      {
         state_machine = incomingBytes[1];

         if ( (incomingBytes[3] <= 100) && (incomingBytes[3] >= -100) )
            throttleVal = incomingBytes[3];
         if ( (incomingBytes[5] <= 100) && (incomingBytes[5] >= -100) )
            steeringVal = incomingBytes[5];
         if ( (incomingBytes[7] <= 100) && (incomingBytes[7] >= -100) )
            brakeVal = incomingBytes[7];
         return (true);
      }
      else
      {
         Serial.print(" state_machine = "); Serial.println(incomingBytes[1], BIN);
      }
   }
   else        //  Serial.available sees insufficient incoming data
   {  
      outgoingBytes[1] = false;                         // capture comm status in response array
      for ( out=0; out<NUMBER_OF_BYTES_TO_SEND; out++)  // send back response
      {
         Serial.write( outgoingBytes[out] );
      }
      return (false);
   }   
}

void motor_setValues (int throttle, int steering, int brake)
{
  boolean turboBoostOn;

  turboBoostOn = bitRead(state_machine, 1);
    
  if ( turboBoostOn )     // turbo boost on
  { 
     throttleMotorVal = MOTOR_VALUE_THROTTLE_TURBO;
     digitalWrite(BLUE_LED, HIGH);
  }
  else                    // turbo boost off
  {
    throttleMotorVal = map(throttle, 0, 100, MOTOR_VALUE_THROTTLE_MIN, MOTOR_VALUE_THROTTLE_MAX );
    digitalWrite(BLUE_LED, LOW);
  }

  steeringMotorVal = map(steering, -100, 100, MOTOR_VALUE_MIN, MOTOR_VALUE_MAX );
  brakeMotorVal    = map(brake, -100, 0, MOTOR_VALUE_FULL_BRAKE, MOTOR_VALUE_NO_BRAKE );

  if (debug == 1)
  {  /*
     Serial.print("throttle = "); Serial.print(throttleMotorVal);
     Serial.print("\t");
     Serial.print("steering = "); Serial.print(steeringMotorVal);
     Serial.print("\t");
     Serial.print("brake = "); Serial.println(brakeMotorVal);    */
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

void flashLEDs ()
{
  for (int i = 0; i<3; i++)
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
}

