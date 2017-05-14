/*
    James Beaver, Anthony G., Stephen A.
    May 14, 2017
    ETEC 290 Capstone project

    Hand Controller device program for communication to robot via XBee 802.15 radio protocol.
    A 8 bit state variable is used to track status of LEDs, button presses, and other controls that are communicated
    to the receiving robot.  If the Up/Down joystick is pushed forward --> throttle control.
                           
    Added a pushbutton input for brakes.   The Up/Down joystick is currently only used for throttle.
                               
    Left/Right joystick input is being limited to 40% of full range.

    Hardware setup:
      Arduino Pro Mini 5v 16MHz microcontroller
      Two Paralax 2-axis joysticks:  one for Up/Down, one for Left/Right
      XBee series 1 1mw U.FL Connection, mounted on a SparkFun 'XBee Explorer Regulated' board
      Push button for start / stop control
      LEDs for status indicators

      TBD - Throttle limit, proportionally to turn rate, max un-boosted limit TBD
          -
*/

const int MOTOR_VALUE_STOP = 0;
const int X_JOYSTICK_MIN = 3;
const int X_JOYSTICK_MAX = 1022;
const int Y_JOYSTICK_MIN = 0;
const int Y_JOYSTICK_MAX = 1021;

const int NUMBER_OF_BYTES_OUTGOING = 8;
const int SERIAL_COMMAND_SET_CMD = 252;
const int SERIAL_COMMAND_SET_THROTTLE = 253;
const int SERIAL_COMMAND_SET_STEERING_POS = 254;
const int SERIAL_COMMAND_SET_BRAKE_POS = 255;

const byte RED_LED = 3;     // robot disabled / communication loss
const byte GREEN_LED = 4;   // robot enabled / communication working
const byte BUTTON = 5;      // robot enable input button
const byte TURBO = 6;       // turbo boost input button
const byte BRAKE = 7;       // brake input button
const byte BRAKELED = 8;    // brake LED lights when brake applied 

const int DEAD_ZONE = 5;    //  narrow deadzone near joystick centered position

const unsigned long TIME_BETWEEN_GET_DATA = 50;    // input sample rate and data send period in ms

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

boolean turnOnOff = false;   //  false == 0, true != 0    initially: robot OFF

int brakesOn;                // brakes ON / OFF
int valButton;               // variable for reading the enable button pin status

char outgoingbytes[NUMBER_OF_BYTES_OUTGOING];  // char type holds signed values -128 to 127

unsigned long previousTime;

void setup()
{
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BRAKELED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(TURBO, INPUT);
  pinMode(BRAKE, INPUT);

  digitalWrite(RED_LED, HIGH);    // turn on the Red LED
  digitalWrite(GREEN_LED, LOW);   // turn off the Green LED
  digitalWrite(BRAKELED,HIGH);    // flash brake light to ensure it works
  delay(200);
  digitalWrite(BRAKELED,LOW);     // turn off the brake light
  bitClear(state_machine, 0);     // Robot OFF mode

  UDcenter = analogRead(JOYSTICK_Y);     //  initialize the centered joystick value
  LRcenter = analogRead(JOYSTICK_X);     //  initialize the centered joystick value

  throttleServoVal = UDcenter;    // initially: throttle position is off
  brakesOn = HIGH;                // initially: brakes OFF
  steeringServoVal = LRcenter;    // initially: steering centered

  Serial.begin(SERIAL_DATA_SPEED_BPS);   // enable serial communication
  previousTime = millis();               // initialize the time count
}

void loop()
{
  turnOnOff = readButton();    // read button input value - subroutine near bottom of this code
//  Serial.print("turnOnOff ="); Serial.println(turnOnOff);
  
  bitClear(state_machine, 1);  //  make sure turbo boost bit is clear
   
  if (turnOnOff == true)       //  Robot ON state
  {
    digitalWrite(GREEN_LED, HIGH);  // turn on the Green LED
    digitalWrite(RED_LED, LOW);     // turn off the Red LED
    bitSet(state_machine, 0);       // state_machine bits: Robot ON
  }
  if (turnOnOff == false)       //  Robot OFF state
  {
    digitalWrite(GREEN_LED, LOW);   // turn off the Green LED
    digitalWrite(RED_LED, HIGH);    // turn on the Red LED
    bitClear(state_machine, 0);     // state_machine bits: Robot OFF
  }

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

       //   Serial.print("turboval = "); Serial.println(turboval);

       if (turboval)
       {
         bitClear(state_machine, 1);
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

      if ( UDinput > UDcenter )           // throttle is being applied, turn off brakes
      {
        throttleServoVal = map(UDinput, UDcenter, Y_JOYSTICK_MAX, 0, 100);
        brakeServoVal = 0;
        brakesOn = HIGH;
        digitalWrite(BRAKELED, LOW);
      }
      else              //  throttle is off, so check for brakes
      {
        brakesOn = digitalRead(BRAKE);      // is the brake applied?
        
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
      /*       if(debug ==1)
             {
                Serial.print("throttleServoVal = "); Serial.print(throttleServoVal);
                Serial.print("\t");
                Serial.print("steeringServoVal = "); Serial.print(steeringServoVal);
                Serial.print("\t");
                Serial.print("brakeServoVal = "); Serial.println(brakeServoVal);
             }
      */
      if (throttleServoVal <= DEAD_ZONE && steeringServoVal <=  DEAD_ZONE && steeringServoVal >= -DEAD_ZONE)
      {
        if (debug == 1)    Serial.println( "Deadzone" );

        SendNewMotorValues(MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, state_machine);
        bitClear(state_machine, 1);    //  clear the turbo bit
        previousTime = millis();
      }
      else
      {
        SendNewMotorValues(throttleServoVal, steeringServoVal, brakeServoVal, state_machine);
        bitClear(state_machine, 1);    //  clear the turbo bit
        previousTime = millis();
      }
    }
  }
  else      //  state machine == 'OFF', so tell the robot
  {
    //     Serial.print(" state_machine = "); Serial.println(state_machine,HEX);
    SendNewMotorValues(MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, state_machine);
    previousTime = millis();
  }
}

//****************************** SUBROUTINES ************************************
void SendNewMotorValues(char throttle, char steering, char brake, byte statemachine)
{  
   byte count;

   outgoingbytes[0] = SERIAL_COMMAND_SET_CMD;
   outgoingbytes[1] = statemachine;
   outgoingbytes[2] = SERIAL_COMMAND_SET_THROTTLE;
   outgoingbytes[3] = throttle;
   outgoingbytes[4] = SERIAL_COMMAND_SET_STEERING_POS;
   outgoingbytes[5] = steering;
   outgoingbytes[6] = SERIAL_COMMAND_SET_BRAKE_POS;
   outgoingbytes[7] = brake;
    
   if (debug == 1)
   {
      Serial.print("state_machine = "); Serial.print(statemachine, HEX);
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
         Serial.write( outgoingbytes[count] );  // send data via XBee to robot    
      }
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

