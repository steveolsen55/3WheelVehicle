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
 *    XBee series 1 1mw U.FL Connection, mounted on a SparkFun 'XBee Explorer Regulated' board
 *    Push button for start / stop control
 *    LEDs for status indicators
 *    
 *    TBD - Turbo boost button control, to provide full speed boost on straight segments
 *        - Throttle limit, proportionally to turn rate, max un-boosted limit TBD
 *        - 
 */
 
const int MOTOR_VALUE_STOP = 0;
const int X_JOYSTICK_MIN = 3;
const int X_JOYSTICK_MAX = 1022;
const int Y_JOYSTICK_MIN = 0;
const int Y_JOYSTICK_MAX = 1021;

const int SERIAL_COMMAND_SET_CMD = 252;
const int SERIAL_COMMAND_SET_THROTTLE = 253;
const int SERIAL_COMMAND_SET_STEERING_POS = 254;
const int SERIAL_COMMAND_SET_BRAKE_POS = 255;

const byte RED_LED = 3;
const byte GREEN_LED = 4;
const byte BUTTON = 5;

const int DEAD_ZONE = 5;    //  narrow deadzone near joystick centered position

const unsigned long TIME_BETWEEN_GET_DATA = 100;    // 100ms.  perhaps reduce to 50?

const long SERIAL_DATA_SPEED_BPS = 57600;     //  baud rate = 57600 for Capstone Xbee's

int debug = 0;        //  set to 1 for debug output on Serial Monitor
    
int JOYSTICK_X = A3;     //  left/right joystick input 
int JOYSTICK_Y = A2;     //  up/down joystick input
  
int UDvalue;         //  var to hold run time Up / Down joystick (throttle/brake)
int LRvalue;         //  var to hold run time Left / Right steering joystick input
int UDcenter;        //  what value is generated when joystick is centered?
int LRcenter;        //  what value is generated when joystick is centered?
   
int throttleServoVal;    // value to send for throttle servo speed control            
int steeringServoVal;    // value to send for steering servo position control
int brakeServoVal;       // value to send for brake servo position control

byte state_machine = 0x00;   // binary 00 = both LEDs off, Red LED is bit 1, Green LED is bit 0.  Set to 1 when on.
                             // TBD - add bits for turbo boost, headlights, ??
                             // This variable tracks the status of the overall state machine

boolean turnOnOff = false;   //  false == 0, true != 0    initially: robot OFF
int valButton;               // variable for reading the button pin status

unsigned long previousTime;    

void setup()
{
   pinMode(RED_LED, OUTPUT);  
   pinMode(GREEN_LED, OUTPUT);
   pinMode(BUTTON, INPUT);

   digitalWrite(RED_LED,HIGH);     // turn on the Red LED
   digitalWrite(GREEN_LED,LOW);    // turn off the Green LED
   bitClear(state_machine, 0);     // Green LED is off
   bitSet(state_machine, 1);       // Red LED is on

   UDcenter = analogRead(JOYSTICK_Y);     //  initialize the centered value      
   LRcenter = analogRead(JOYSTICK_X);     //  initialize the centered value
   
   Serial.begin(SERIAL_DATA_SPEED_BPS);   // enable serial communication
   previousTime = millis();               // initialize the time count
}

void loop()
{ 
   turnOnOff = readButton();    // read button input value - subroutine near bottom of this code
   
   if (turnOnOff == true)       //  Robot ON state
   {
      digitalWrite(GREEN_LED, HIGH);  // turn on the Green LED
      digitalWrite(RED_LED, LOW);     // turn off the Red LED
      bitSet(state_machine, 0);       // state_machine bits: Green LED is on
      bitClear(state_machine, 1);     //                     Red LED is off
   }
   if (turnOnOff == false)       //  Robot OFF state
   {
      digitalWrite(GREEN_LED, LOW);   // turn off the Green LED
      digitalWrite(RED_LED, HIGH);    // turn on the Red LED
      bitClear(state_machine, 0);     // state_machine bits: Green LED is off
      bitSet(state_machine, 1);       //                     Red LED is on
   }

   if ( (state_machine & 0x03) == B01 )     //  state machine == 'ON', so process the joystick inputs
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

       if ( UDvalue > UDcenter )
       {
          throttleServoVal = map(UDvalue,UDcenter,Y_JOYSTICK_MAX,0,100);
          brakeServoVal = 0;
       }    
       else 
       {   
          brakeServoVal = map(UDvalue,Y_JOYSTICK_MIN,UDcenter,-100,0); 
          throttleServoVal = 0;
       }       
       if ( LRvalue > LRcenter )
       {
          steeringServoVal = map(LRvalue,LRcenter,X_JOYSTICK_MAX,0,100);
       }
       else 
       {
          steeringServoVal = map(LRvalue,X_JOYSTICK_MIN,LRcenter,-100,0);
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
    }  
  }
  else      //  ( (state_machine & 0x03) == B10)  state machine == 'OFF', so tell the robot
  {
//     Serial.print(" state_machine = "); Serial.println(state_machine,HEX);
     SendNewMotorValues(MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, MOTOR_VALUE_STOP, state_machine);
     previousTime = millis();
  } 
}  

//****************************** SUBROUTINES ************************************
void SendNewMotorValues(char throttle, char steering, char brake, byte statemachine)
{
   if (debug == 1)
   {

      Serial.print("throttle = ");
      Serial.print(throttle,DEC);
      Serial.print("\t");
      Serial.print("steering = ");
      Serial.print(steering,DEC);
      Serial.print("\t");
      Serial.print("brake = ");
      Serial.print(brake,DEC);
      Serial.print("\t");      
      Serial.print("state_machine = ");
      Serial.println(statemachine,HEX);  
 
   }
   else
   {
      Serial.write(SERIAL_COMMAND_SET_CMD);
      Serial.write(statemachine);
      Serial.write(SERIAL_COMMAND_SET_THROTTLE);
      Serial.write(throttle);              
      Serial.write(SERIAL_COMMAND_SET_STEERING_POS);
      Serial.write(steering);              
      Serial.write(SERIAL_COMMAND_SET_BRAKE_POS);
      Serial.write(brake);    
   }
}

int readButton()
{
  unsigned long contactTime;           // local variable; contactTime declared
  valButton=digitalRead(BUTTON);       // Read the pushbutton on an digital pin

  if(valButton == HIGH)                // Since 5V will produce a HIGH --> means button not pushed
    return turnOnOff;                  // return the value of turnOnOff without changing it

  contactTime=millis();                // set contactTime = to the millis() clock value
  while(valButton == LOW)              // button pressed produces a LOW while the pushbutton is pushed
  {
    valButton=digitalRead(BUTTON);     // read the button value again and keep reading until valButton is HIGH again
  }

  if(millis()-contactTime<20)          // If the button is held for less than 20 ms
    return turnOnOff;                  // return the turnOnOff value unchanged

  return(1-turnOnOff);                 // if the button is held longer than 20 ms then change the turnOnOff
                                       // to the opposite value to what it was
}




