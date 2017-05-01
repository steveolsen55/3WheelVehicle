

  
 
const int MOTOR_VALUE_STOP = 0;

const int X_JOYSTICK_MIN = 0;
const int X_JOYSTICK_MAX = 1023;

const int Y_JOYSTICK_MIN = 50;
const int Y_JOYSTICK_MAX = 1023;

const int SERIAL_COMMAND_SET_LEFT_MOTOR = 254;
const int SERIAL_COMMAND_SET_RIGHT_MOTOR = 255;

const int DEAD_ZONE = 15;

const unsigned long TIME_BETWEEN_GET_DATA = 100;
const long SERIAL_DATA_SPEED_38400_BPS = 38400;
     
 int JOYSTICK_X = A1;     
 int JOYSTICK_Y = A0; 
 
 
 int UDvalue;      
 int LRvalue;      
 int LeftServoVal;                 
 int RightServoVal;   
 
 
 unsigned long previousTime;

void setup()
{
    Serial.begin(SERIAL_DATA_SPEED_38400_BPS);
         
    previousTime = millis();
}

void loop()
{   
      int UDvalue = 0;
      int LRvalue = 0;
      
      if (millis()- previousTime >= TIME_BETWEEN_GET_DATA)
      {
        UDvalue = analogRead(JOYSTICK_Y);           
        LRvalue = analogRead(JOYSTICK_X);   
    
        UDvalue = map(UDvalue,Y_JOYSTICK_MIN,Y_JOYSTICK_MAX,-100,100);  
        LRvalue = map(LRvalue,X_JOYSTICK_MIN,X_JOYSTICK_MAX,-100,100);    
        
       if(UDvalue >= -DEAD_ZONE && UDvalue <= DEAD_ZONE && LRvalue <=  DEAD_ZONE && LRvalue >= -DEAD_ZONE) 
       {           
        
          SendNewMotorValues(MOTOR_VALUE_STOP,MOTOR_VALUE_STOP);
          previousTime = millis();
        }
        else                                   
        {                                         
          moveRobot(LRvalue,UDvalue);
          previousTime = millis();
        }
      }
}
   


//****************************** SUBROUTINES ************************************
  void SendNewMotorValues(char left, char right)
      {
        Serial.write (SERIAL_COMMAND_SET_LEFT_MOTOR);
        Serial.write (left);
                  
        Serial.write (SERIAL_COMMAND_SET_RIGHT_MOTOR);
        Serial.write (right);
        Serial.print(left,DEC);
        Serial.print("   ");
        Serial.println(right,DEC);
      }
      
//*************** Calculate motor values **************************************************      
  void moveRobot (int valueX, int valueY)//subroutine to send pulses to the correct servo.
  {
    //Use the X direction to set the forward and backward motion
    int leftMotorVal = valueY;
    int rightMotorVal = valueY;
    
    //Apply the Y as a "twist" effect. (about 40% of the calculated value)
    if (valueY >=0)
    {
      leftMotorVal = leftMotorVal + (valueX * 4)/10;
      rightMotorVal = rightMotorVal - (valueX * 4)/10;
    }
    else
    {
      leftMotorVal = leftMotorVal - (valueX * 4)/10;
      rightMotorVal = rightMotorVal + (valueX * 4)/10;
    }
      leftMotorVal = constrain(leftMotorVal,-100,100);
      rightMotorVal =constrain(rightMotorVal,-100,100);
     
      SendNewMotorValues(leftMotorVal,rightMotorVal);
  }



