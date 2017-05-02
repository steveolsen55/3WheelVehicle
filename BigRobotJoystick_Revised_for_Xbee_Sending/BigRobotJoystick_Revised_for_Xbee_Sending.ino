/*   
 *  James Beaver, Anthony G., Stephen A.
 *  April 29, 2017
 *   ETEC 290 Capstone project
 *    
 *  Hand Controller device program for communication to robot via XBee 802.15 radio protocol. 
 *  A 8 bit state variable is used to track status of LEDs, button presses, and other controls.
 *  If the Up/Down joystick is pushed forward --> throttle control
 *                         ... pulled back    --> throttle to zero and apply brake
 *  Left/Right joystick input is being limited to 40% of full range.
 * 
 *  Hardware setup:
 *    Arduino Pro Mini 5v 16MHz microcontroller
 *    Two Paralax 2-axis joysticks:  one for Up/Down, one for Left/Right
 *    XBee series 1 1mw U.FL Connection, on a SparkFun 'XBee Explorer Regulated' board
 *    Push button for start / stop control
 *    LEDs for status indicators
 *    
 *    TBD - Turbo boost button control, to provide full speed boost on straight segments
 *        - Throttle limit, proportionally to turn rate, max un-boosted limit TBD
 *        - 
 */
 
const int MOTOR_VALUE_STOP = 0;
const int X_JOYSTICK_MIN = 3;
const int X_JOYSTICK_MAX = 1023;
const int Y_JOYSTICK_MIN = 0;
const int Y_JOYSTICK_MAX = 1023;

const int SERIAL_COMMAND_SET_CMD = 252;
const int SERIAL_COMMAND_SET_THROTTLE = 253;
const int SERIAL_COMMAND_SET_STEERING_POS = 254;
const int SERIAL_COMMAND_SET_BRAKE_POS = 255;

const byte RED_LED = 3;
const byte GREEN_LED = 4;
const byte SWITCH_IN = 5;

const int DEAD_ZONE = 5;

const unsigned long TIME_BETWEEN_GET_DATA = 100;    // 100ms.  perhaps reduce to 50?
const long SERIAL_DATA_SPEED_BPS = 57600;     //  baud rate = 57600 for Capstone Xbee's

int debug = 0;        //  set to 1 for debug output on Serial Monitor
    
int JOYSTICK_X = A3;     //  left/right joystick input 
int JOYSTICK_Y = A2;     //  up/down joystick input
  
int UDvalue;         //  var to hold run time Up / Down joystick (throttle/brake)
int LRvalue;         //  var to hold run time Left / Right steering joystick input
int UDcenter;        //  what value is generated when joystick is centered?
int LRcenter;        //  what value is generated when joystick is centered?
int throttle;        //  limiting throttl values to 0 <--> +100
int steering;        //  limiting L/R values to -100 <--> +100
int brake;           //  limiting brake values to -100 <--> 0
   
int throttleServoVal;    // value to send for throttle servo speed control            
int steeringServoVal;    // value to send for steering servo position control
int brakeServoVal;       // value to send for brake servo position control

byte state_machine = B00000000;  // binary 00 = both LEDs off, Red LED is bit 1, Green LED is bit 0.  Set to 1 when on.
                                 // TBD - add bits for turbo boost, headlights, ??
                                 // This variable tracks the status of the overall state machine

boolean turnOnOff = false;

int val;                    // variable for reading the button pin status
int buttonState;            // variable to hold the latest button state
int buttonPresses = 0;      // how many times the button has been pressed

unsigned long previousTime;

void setup()
{
   pinMode(RED_LED, OUTPUT);  
   pinMode(GREEN_LED, OUTPUT);
   pinMode(SWITCH_IN, INPUT);

   digitalWrite(RED_LED,HIGH);     // turn on the Red LED
   digitalWrite(GREEN_LED,LOW);    // turn off the Green LED

   bitClear(state_machine, 0);     // Green LED is off
   bitSet(state_machine, 1);       // Red LED is on
   
   buttonState = digitalRead(SWITCH_IN);   // read the initial button state

   UDcenter = analogRead(JOYSTICK_Y);     //  initialize the centered value      
   LRcenter = analogRead(JOYSTICK_X);     //  initialize the centered value
   
   Serial.begin(SERIAL_DATA_SPEED_BPS);   // enable serial communication
   previousTime = millis();               // initialize the time count
}

void loop()
{ 
   val = digitalRead(SWITCH_IN);     // read input value and store it in val   
   if (val != buttonState)           // the button state has changed!
   { 
     if (val == LOW)                 // check if the button is pressed
     {    
        buttonPresses++;             // increment the buttonPresses variable
     }
     buttonState = val;              // save the new button state in our variable
   }

   if (buttonPresses == 1)
   {
      digitalWrite(GREEN_LED, HIGH);  // turn on the Green LED
      digitalWrite(RED_LED, LOW);     // turn off the Red LED
      bitSet(state_machine, 0);       // state_machine bits: Green LED is on
      bitClear(state_machine, 1);     //                     Red LED is off
   }
   if (buttonPresses == 2)
   {
      digitalWrite(GREEN_LED, LOW);   // turn off the Green LED
      digitalWrite(RED_LED, HIGH);    // turn on the Red LED
      bitClear(state_machine, 0);     // state_machine bits: Green LED is off
      bitSet(state_machine, 1);       //                     Red LED is on
      buttonPresses = 0;
   }
    
   if ( (state_machine & 0x03) == B01 )     //  state machine == 'GO!!', so process the joystick inputs
   {  
     int UDvalue = 0;          //  clear the joystick input vars
     int LRvalue = 0;
      
     if (millis()- previousTime >= TIME_BETWEEN_GET_DATA)
     {
       UDvalue = analogRead(JOYSTICK_Y);   //  read the current joystick input positions
       LRvalue = analogRead(JOYSTICK_X);   //
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

//      the map function doesn't allow for irregular centered joystick values
//      using float calculations gives more precise control and adjusts the center
//          UDvalue = map(UDvalue,X_JOYSTICK_MIN,Y_JOYSTICK_MAX,-100,100);  
//          LRvalue = map(LRvalue,X_JOYSTICK_MIN,X_JOYSTICK_MAX,-100,100);
    
//  using 105 for values < XXcenter, due to center bias of the joysticks being used: not exactly 1023/2.

       if ( UDvalue > UDcenter )
       {
//          throttleServoVal = constrain( 100*(float(UDvalue-UDcenter)/float(Y_JOYSTICK_MAX-UDcenter)), 0, 100);
          throttleServoVal = map(UDvalue,UDcenter,Y_JOYSTICK_MAX,0,100);
          brakeServoVal = 0;
       }    
       else 
       {   
//          brakeServoVal = constrain( 105*(float(UDcenter-UDvalue)/float(UDcenter)), -100, 0); 
          brakeServoVal = map(UDvalue,Y_JOYSTICK_MIN,UDcenter,-100,0); 
          throttleServoVal = 0;
       }       
       if ( LRvalue > LRcenter )
       {
//          steeringServoVal = constrain( 100*(float(LRvalue-LRcenter)/float(X_JOYSTICK_MAX-LRcenter)), -100, 100);
          steeringServoVal = map(LRvalue,LRcenter,X_JOYSTICK_MAX,0,100);
       }
       else 
       {
//         steeringServoVal = constrain( 105*(float(LRcenter-LRvalue)/float(LRcenter)), -100, 100);
          steeringServoVal = map(LRvalue,X_JOYSTICK_MIN,LRcenter,-100,0);
       }
        if(debug ==1)
        {
           Serial.print("throttleServoVal = "); Serial.print(throttleServoVal);
           Serial.print("\t");
           Serial.print("steeringServoVal = "); Serial.print(steeringServoVal);
           Serial.print("\t");
           Serial.print("brakeServoVal = "); Serial.println(brakeServoVal);
        } 

        if(brakeServoVal >= -DEAD_ZONE && throttleServoVal <= DEAD_ZONE && steeringServoVal <=  DEAD_ZONE && steeringServoVal >= -DEAD_ZONE) 
        {  
           if (debug ==1)    Serial.println( "Deadzone" );
           
           SendNewMotorValues(MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, state_machine);
           previousTime = millis();
        }
        else                                   
        {                                         
           SendNewMotorValues(throttleServoVal, steeringServoVal, brakeServoVal, state_machine);
           previousTime = millis();
        }
      
 
      if ( (state_machine & 0x03) == B10)    //  state machine == 'do nothing', so tell the robot
      {
         SendNewMotorValues(MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, state_machine);
         previousTime = millis();
      }   
    }  
  }
}  

//****************************** SUBROUTINES ************************************
void SendNewMotorValues(char throttle, char steering, char brake, byte statemachine)
{
   if (debug == 1)
   {
 
      Serial.print("state_machine = ");
      Serial.print(statemachine,BIN);
      Serial.print("\t");
      Serial.print("throttle = ");
      Serial.print(throttle,DEC);
      Serial.print("\t");
      Serial.print("steering = ");
      Serial.print(steering,DEC);
      Serial.print("\t");
      Serial.print("brake = ");
      Serial.println(brake,DEC);
 
   }
   else
   {
      Serial.write (SERIAL_COMMAND_SET_CMD);
      Serial.write (statemachine);
      Serial.write (SERIAL_COMMAND_SET_THROTTLE);
      Serial.write (throttle);              
      Serial.write (SERIAL_COMMAND_SET_STEERING_POS);
      Serial.write (steering);              
      Serial.write (SERIAL_COMMAND_SET_BRAKE_POS);
      Serial.write (brake);    
   }
}
 
