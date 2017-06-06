/*
    James Beaver, Anthony Gilliland, Stephan Archambeault
    May 16, 2017
    ETEC 290 Capstone project

    Hand Controller device program for communication to robot via XBee 802.15 radio protocol.
    A 8 bit state variable is used to track status of LEDs, button presses, and other controls that are communicated
    to the receiving robot.  If the Up/Down joystick is pushed forward --> throttle control.

    Added a LED pushbutton input for throttle 'turbo' boost.  Throttle is limited without turbo boost.   Turbo boost 
    substantially increases the throttle input when no steering angle is being applied.
    
    Added a LED pushbutton input for brakes.   Pressing the brake button applies braking force through servo rotation.
    The Up/Down joystick is currently only used for throttle.  Left/Right joystick input is being limited to 40% of full range.

    Added commuication response from the robot to the hand controller to validate proper communication status.  

    Hardware setup:
      Arduino Pro Mini 5v 16MHz microcontroller
      Two Paralax 2-axis joysticks:  one for Up/Down, one for Left/Right
      XBee series 1 1mw U.FL Connection, mounted on a SparkFun 'XBee Explorer Regulated' board
      LED push buttons for brake and turbo boost to throttle
      LED for communication status

      TBD - Throttle limit, proportionally to turn rate, max un-boosted limit TBD
          -
*/

const int MOTOR_VALUE_STOP = 0;
const int X_JOYSTICK_MIN = 3;
const int X_JOYSTICK_MAX = 1022;
const int Y_JOYSTICK_MIN = 0;
const int Y_JOYSTICK_MAX = 1021;

const int NUMBER_OF_BYTES_OUTGOING = 8;          // serial data packet is 8 bytes
const int SERIAL_COMMAND_SET_ACK = 251;          // serial data code - next byte is response data
const int SERIAL_COMMAND_SET_CMD = 252;          // serial data code - next byte is a command byte
const int SERIAL_COMMAND_SET_THROTTLE = 253;     // serial data code - next byte is throttle setting
const int SERIAL_COMMAND_SET_STEERING_POS = 254; // serial data code - next byte is steering position
const int SERIAL_COMMAND_SET_BRAKE_POS = 255;    // serial data code - next byte is brake setting
const int NUMBER_OF_BYTES_INCOMING = 2;          // serial data response data packet size

const byte RED_LED = 3;     // robot disabled / communication loss
const byte GREEN_LED = 4;   // robot enabled / communication working
const byte BUTTON = 5;      // robot enable input button
const byte TURBO = 6;       // turbo boost input button
const byte BRAKE = 7;       // brake input button
const byte TURBOLED = 8;    // turbo ON indicator
const byte BRAKELED = 9;    // brake LED lights when brake applied 

const int DEAD_ZONE = 5;    //  narrow deadzone near joystick centered position

const unsigned long TIME_BETWEEN_GET_DATA = 50;    // input sample rate and data send period in ms
const unsigned long COMM_LOSS_LIMIT = 600;         // how long can comm response be false before error thrown 

const long SERIAL_DATA_SPEED_BPS = 38400;          //  baud rate for Capstone Xbee's

int debug = 0;           //  set to 1 for debug output on Serial Monitor

int JOYSTICK_X = A3;     //  left/right joystick input
int JOYSTICK_Y = A2;     //  up/down joystick input

int UDvalue;         //  var to hold run time Up / Down joystick (throttle/brake)
int LRvalue;         //  var to hold run time Left / Right steering joystick input
int UDcenter;        //  what value is generated when joystick is centered?
int LRcenter;        //  what value is generated when joystick is centered?

int throttleServoVal;    // value to send for throttle servo speed control
int steeringServoVal;    // value to send for steering servo position control
int brakeServoVal;       // value to send for brake servo position control

byte state_machine = 0x00;   // bit 0 = enable Robot.
                             // bit 1 = enable turbo.
                             // TBD - add bits for headlights, ??
                             // This variable tracks the status of the overall state machine

boolean turnOnOff = false;   //  false == 0, true != 0
boolean responseAck;         //  what is the robot's data response  

int brakesOn;                // brakes ON / OFF
int valButton;               // variable for reading the enable button pin status

char incomingBytes[NUMBER_OF_BYTES_INCOMING];  
char outgoingBytes[NUMBER_OF_BYTES_OUTGOING];  // char type holds signed values -128 to 127

unsigned long previousTime, responseTimer;

void setup()
{
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BRAKELED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(TURBO, INPUT);
  pinMode(BRAKE, INPUT);

  bitClear(state_machine, 0);     // Robot OFF mode
  responseAck = true;             // assume communication is working until we know it's not

  flashLEDs();                    //  cycle through the hand controller LEDs to ensure they work
  
  UDcenter = analogRead(JOYSTICK_Y);     //  initialize the centered joystick value
  LRcenter = analogRead(JOYSTICK_X);     //  initialize the centered joystick value

  throttleServoVal = UDcenter;    // initially: throttle position is off
  brakesOn = HIGH;                // initially: brakes OFF
  steeringServoVal = LRcenter;    // initially: steering centered

  Serial.begin(SERIAL_DATA_SPEED_BPS);   // enable serial communication
  previousTime = millis();               // initialize the time count
  responseTimer = millis();
}

void loop()
{
   turnOnOff = true;              // turn robot on immediately if it's power is on
     
//  turnOnOff = readButton();    // read button input value - subroutine near bottom of this code
//  Serial.print("turnOnOff ="); Serial.println(turnOnOff);
  
  bitClear(state_machine, 1);  //  make sure turbo boost bit is clear
   
  if (turnOnOff == true)       //  Robot ON state
  {
    bitSet(state_machine, 0);       // state_machine bits: Robot ON
  }
  if (turnOnOff == false)       //  Robot OFF state
  {
    bitClear(state_machine, 0);     // state_machine bits: Robot OFF
  }

  if( responseAck == true )          // communication working?
     digitalWrite(GREEN_LED, HIGH);  // turn on the Green LED
  else
     digitalWrite(GREEN_LED, LOW);  // turn off the Green LED

  if ( bitRead(state_machine,0) == true )     //  state machine == 'ON', so process the joystick inputs
  {
    int UDinput, LRinput;          // local variables to store joystick inputs

    boolean turboval = false;

    if (millis() - previousTime >= TIME_BETWEEN_GET_DATA)
    {
       UDinput = analogRead(JOYSTICK_Y);   //  read the current joystick input positions
       LRinput = analogRead(JOYSTICK_X);   //
       turboval = digitalRead(TURBO);      //  read postion of button for turbo - normally HIGH unless pressed
       brakesOn = digitalRead(BRAKE);      // is the brake applied?

 //         Serial.print("turboval = "); Serial.println(turboval);

       if (turboval)
       {
         bitClear(state_machine, 1);
         digitalWrite(TURBOLED,LOW);
       }
       else
       {
         bitSet(state_machine, 1);
       }
      /*
        if(debug ==1)
        {
          Serial.print("LRcenter = "); Serial.print(LRcenter);
          Serial.print("\t");
          Serial.print("UDcenter = "); Serial.print(UDcenter);
          Serial.print("\t");
          Serial.print("LRvalue = "); Serial.print(LRvalue);
          Serial.print("\t");
          Serial.print("UDvalue = "); Serial.println(UDvalue);
        }   */

      if ( UDinput > UDcenter )     // throttle is being applied
      {  
         throttleServoVal = map(UDinput, UDcenter, Y_JOYSTICK_MAX, 0, 100);
         if ( (bitRead(state_machine,1) == 0) )    // turbo is not applied
         {
            //  do nothing.   leave throttle mapping alone      
         }
         else           //  turbo is being applied
         {
            if ( steeringServoVal != 0)       //  disable turbo boost when not going straight
            {                                 //  robot could flip if too much throttle during turns
               bitClear(state_machine, 1);
               digitalWrite(TURBOLED,LOW);
            }
            else        //  full throttle!!
            {
               throttleServoVal = 100;    
               digitalWrite(TURBOLED,HIGH);
            }  
          
         }
      }
      else                          // throttle is off, so check for brakes
      {       
         if (brakesOn == LOW)                // brake is being applied
         {
           brakeServoVal = -100;
           digitalWrite(BRAKELED, HIGH);
           throttleServoVal = 0;
        }
        else                               // brake is not being applied
        {
           brakeServoVal = 0;
           digitalWrite(BRAKELED, LOW);
        }
        
//    we're using a brake button instead of a joystick reading, so skipping this line:
//        brakeServoVal = map(UDinput, Y_JOYSTICK_MIN, UDcenter, -104, 0);  

      }
      if ( LRinput > LRcenter )
      {
        steeringServoVal = map(LRinput, LRcenter, X_JOYSTICK_MAX, 0, 100);
      }
      else
      {
        steeringServoVal = map(LRinput, X_JOYSTICK_MIN, LRcenter, -100, 0);
      }
          
      if(debug ==1)
      {  /*
          Serial.print("throttleServoVal = "); Serial.print(throttleServoVal);
          Serial.print("\t");
          Serial.print("steeringServoVal = "); Serial.print(steeringServoVal);
          Serial.print("\t");
          Serial.print("brakeServoVal = "); Serial.println(brakeServoVal);     */
      }

      if (throttleServoVal <= DEAD_ZONE && steeringServoVal <=  DEAD_ZONE && steeringServoVal >= -DEAD_ZONE)
      {
//        if (debug == 1)    Serial.println( "Deadzone" );

        SendNewMotorValues(MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, brakeServoVal, state_machine);
//        read_Serial_Data_ack();
        bitClear(state_machine, 1);    //  clear the turbo bit, but allow brakes
        previousTime = millis();
      }
      else
      {
        SendNewMotorValues(throttleServoVal, steeringServoVal, brakeServoVal, state_machine);
//        read_Serial_Data_ack();
        bitClear(state_machine, 1);    //  clear the turbo bit for next loop
        previousTime = millis();
      }
      throttleServoVal = 0;     //  clear vars for next loop
      steeringServoVal = 0;
      brakeServoVal = 0;
    }
  }
  else      //  state machine == 'OFF', so tell the robot
  {
    //     Serial.print(" state_machine = "); Serial.println(state_machine,BIN);
    SendNewMotorValues(MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, state_machine);
 //   read_Serial_Data_ack();
    digitalWrite(GREEN_LED, LOW);        // turn off the Green LED
    previousTime = millis();
  }
}

//****************************** SUBROUTINES ************************************
void SendNewMotorValues(char throttle, char steering, char brake, byte statemachine)
{  
   byte count;

   outgoingBytes[0] = SERIAL_COMMAND_SET_CMD;
   outgoingBytes[1] = statemachine;
   outgoingBytes[2] = SERIAL_COMMAND_SET_THROTTLE;
   outgoingBytes[3] = throttle;
   outgoingBytes[4] = SERIAL_COMMAND_SET_STEERING_POS;
   outgoingBytes[5] = steering;
   outgoingBytes[6] = SERIAL_COMMAND_SET_BRAKE_POS;
   outgoingBytes[7] = brake;
    
   if (debug == 1)
   {
      Serial.print("state_machine = "); Serial.print(statemachine, BIN);
      Serial.print("\t");
      Serial.print("throttle = "); Serial.print(throttle, DEC);
      Serial.print("\t");
      Serial.print("steering = "); Serial.print(steering, DEC);
      Serial.print("\t");
      Serial.print("brake = "); Serial.println(brake, DEC); 
   }
   else
   {
      for (count = 0; count < NUMBER_OF_BYTES_OUTGOING; count++)
      {
         Serial.write( outgoingBytes[count] );  // send data via XBee to robot    
      }
   }
}

void read_Serial_Data_ack()
{
   byte i;
   char garbage;

   if (Serial.available() >= NUMBER_OF_BYTES_INCOMING)    // check for incoming response data
   {
      while ( Serial.peek() != SERIAL_COMMAND_SET_ACK )   // data is available so assuming it's working
      {
         garbage = Serial.read();                         // this could get stuck if no ACK detected
      }
      for ( i=0; i<NUMBER_OF_BYTES_INCOMING; i++ )        // ack detected 
      {
        incomingBytes[i] = Serial.read();
      }
      responseAck = incomingBytes[1];    // what was the response?
   }
   else                       // no data available, so comm must be down
   {
      responseAck = false;   // we didn't get a response
   }
 
   if (responseAck == false)
   {
      if( (millis() - responseTimer) <= COMM_LOSS_LIMIT)
      {
        responseAck = true;    //  set ACK = true for now, but let responseTimer continue lagging 
      }
      else
      {
         //  leave responseAck = false   comm has been down for long time
      }
   }
   else       //  responseAck is true, so comm is working
   {
      responseTimer = millis();   //  reset timer for another loop
   }  
}

int readButton()
{
  unsigned long contactTime;          // local variable; contactTime declared
  valButton = digitalRead(BUTTON);    // Read the pushbutton on an digital pin

  if (valButton == HIGH)              // Since 5V will produce a HIGH --> means button not pushed
    return turnOnOff;                 // return the value of turnOnOff without changing it

  contactTime = millis();             // set contactTime = to the millis() clock value
  while (valButton == LOW)            // button pressed produces a LOW while the pushbutton is pushed
  {
    valButton = digitalRead(BUTTON);  // read the button value again and keep reading until valButton is HIGH again
  }

  if (millis() - contactTime < 20)    // If the button is held for less than 20 ms
    return turnOnOff;                 // return the turnOnOff value unchanged

  return (1 - turnOnOff);             // if the button is held longer than 20 ms then change the turnOnOff
                                      // to the opposite value to what it was
}

void flashLEDs ()
{
  for( int j=0; j<3; j++)
  {
     digitalWrite(GREEN_LED, HIGH);    // turn on the Red LED
     digitalWrite(TURBOLED, LOW);      // turn off the Green LED
     digitalWrite(BRAKELED,LOW);       // turn off the Red LED
     delay(100);
     digitalWrite(GREEN_LED, LOW);     // turn off the Green LED
     digitalWrite(TURBOLED, HIGH);     // turn on the Blue LED
     digitalWrite(BRAKELED,LOW);       // turn off the Red LED
     delay(100);
     digitalWrite(GREEN_LED, LOW);     // turn off the Green LED
     digitalWrite(TURBOLED, LOW);      // turn off the Blue LED
     digitalWrite(BRAKELED,HIGH);      // turn on the Red LED
     delay(100);
     digitalWrite(GREEN_LED, LOW);     // turn off the Green LED
     digitalWrite(TURBOLED, LOW);      // turn off the Blue LED
     digitalWrite(BRAKELED,LOW);       // turn off the Red LED
     delay(100);
  }
}

