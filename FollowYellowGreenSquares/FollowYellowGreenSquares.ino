#include <SPI.h>  
#include <Pixy.h>
#include <Wire.h>
#include <NXTI2CDevice.h>
#include <NXTMMX.h>

// Initializing motors.  
NXTMMX driveControl;
float leftSpeed = 0;
float rightSpeed = 0;
int error = 0;
long delay_period = 1000;
long timestamp = 0;
int searchingCounter = 0;
float kp = 0.3;
float baseSpeed = 10.0;

// This is the main Pixy object 
Pixy pixy;

// Provides structure for state 
enum State { SEARCHING, SINGLE_LOCK_BLUE, SINGLE_LOCK_GREEN, DOUBLE_LOCK, ERROR_TILT, ERROR_SPURIOUS };
State state = SEARCHING;

void setup()
{
  Serial.begin(9600);
  Serial.write(17);  //backlight for LCD
  delay(50);
  Serial.write(12);  //reset display for LCD
  delay(50);
  Serial.print("Starting...\n");

  driveControl.reset();   //reset NXT motor encoders
  driveControl.stop(MMX_Motor_Both, MMX_Next_Action_Float);

  pinMode(2, INPUT);
  digitalWrite(2, HIGH);  //enable internal pullup resistor

  pixy.init();
  uint16_t w = 0;
  while(w != 0xaa55)
  {
    w = pixy.testConnection();
  }
  Serial.write(12);  //reset display for LCD
  Serial.print("Found Pixy!...\n");
}

void loop()
{ 
  // NORMAL MODE
  if(digitalRead(2) == 0)
  {
    static int i = 0;
    uint16_t blocks;
  
      // grab blocks!
    blocks = pixy.getBlocks();
  
    // Detect and parse blocks
    if (blocks)
    {
        searchingCounter = 0;
  
       // found one block
        if(blocks == 1)
        {
          if(pixy.blocks[0].signature == 1)
          {
            state = SINGLE_LOCK_BLUE;
          }
          else if(pixy.blocks[0].signature == 2)
          {
            state = SINGLE_LOCK_GREEN;
          }
          else
          {
            state = ERROR_SPURIOUS;
          }
                  
          //Get distance between x-coordinate of whichever block we see and the center of screen
          error = pixy.blocks[0].x - PIXY_MAX_X / 2;
        }
        
        // found two blocks
        else if(blocks == 2)
        {
          state = DOUBLE_LOCK;
          int heightDelta = pixy.blocks[1].y - pixy.blocks[0].y;
          //widthDelta not currently used
          int widthDelta = pixy.blocks[1].x - pixy.blocks[0].x;
          if(abs(heightDelta) > 5)
          {
            state = ERROR_TILT;
          }
          else
          {
            // normalizedSeparation not currently used
            float normalizedSeparation = 2*float(widthDelta)/float(pixy.blocks[0].width + pixy.blocks[1].width);
            //get distance between the center of the two blocks, and the center of the screen
            error = (pixy.blocks[1].x + pixy.blocks[0].x - PIXY_MAX_X) / 2;
          }
        }
  
        // found three or more blocks
        else
        {
          state = ERROR_SPURIOUS;   
        }
      }
    
      else
      {
        // didn't find any blocks
        ++searchingCounter;  
      }
    
  
    if(searchingCounter > 2000)
    {
      state = SEARCHING;
    }
    
    //Serial feedback happens periodically
    if(millis() - timestamp > delay_period )
    {
      Serial.write(12); //clear LCD display
      switch(state)
        {
          case ERROR_TILT:
            Serial.print("Tilted");
            break;
          case ERROR_SPURIOUS:
            Serial.print("Bad Reading");
            break;
          case SEARCHING:
            Serial.print("Searching");
            break;
          case SINGLE_LOCK_GREEN:
            Serial.print("Green Lock");
            break;
          case SINGLE_LOCK_BLUE:
            Serial.print("Blue Lock");
            break;
          case DOUBLE_LOCK:
            Serial.print("Double Lock");
            break;
        }
        Serial.write(13);  // move down a line on LCD
        Serial.print("E:");
        Serial.print(error);
        Serial.print(" L:");
        Serial.print(int(leftSpeed));
        Serial.print(" R:");
        Serial.print(int(rightSpeed));
        timestamp = millis();
    }
  
    switch(state)
    {
      case ERROR_TILT:
        driveControl.stop(MMX_Motor_Both, MMX_Next_Action_Float);
        break;
      case ERROR_SPURIOUS:
        driveControl.stop(MMX_Motor_Both, MMX_Next_Action_Float);
        break;
      case SEARCHING:
        driveControl.runUnlimited(MMX_Motor_1, MMX_Direction_Forward, MMX_Speed_Slow);
        driveControl.runUnlimited(MMX_Motor_2, MMX_Direction_Reverse, MMX_Speed_Slow);
        break;
      case SINGLE_LOCK_BLUE:
      case SINGLE_LOCK_GREEN:
      case DOUBLE_LOCK:
        leftSpeed = baseSpeed + kp * error;
        rightSpeed = baseSpeed - kp * error;
        byte motorDirection =  leftSpeed > 0 ? 0x01: 0x00;
        byte motorSpeed = abs(leftSpeed);
        driveControl.runUnlimited(MMX_Motor_1, motorDirection, motorSpeed);
        motorDirection =  rightSpeed > 0 ? 0x01: 0x00;
        motorSpeed = abs(rightSpeed);
        driveControl.runUnlimited(MMX_Motor_2, motorDirection, motorSpeed);
        break;
    }
  }
  
  // CALIBRATION MODE
  else
  {
    driveControl.stop(MMX_Motor_Both, MMX_Next_Action_Float);
    Serial.write(12);  //clear
    Serial.print("speed:");

    int sensorValue = analogRead(A2);
    int offset = sensorValue - 512;
    int range = map(offset, 512, -512, 5, 0)-1;
    if(range != 2)
    {
      switch(range) 
      {
        case 0:
          baseSpeed -= 0.5;
          break;
         case 1:
          baseSpeed -= 0.1;
          break;
         case 3:
          baseSpeed += 0.1;
          break;
         case 4:
          baseSpeed += 0.5;
          break;
      }
      constrain_(baseSpeed, 0.0, 100.0);
    }
    Serial.print(baseSpeed);
    Serial.write(13);  //move down
    Serial.print("kp:");
    
    sensorValue = analogRead(A3);
    offset = sensorValue - 512;
    range = map(offset, -512, 512, 5, 0)-1;
    if(range != 2)
    {
      switch(range) 
      {
        case 0:
          kp -= 0.1;
          break;
         case 1:
          kp -= 0.01;
          break;
         case 3:
          kp += 0.01;
          break;
         case 4:
          kp += 0.1;
          break;
      }
      constrain_(kp, 0.0, 5.0);
    }
    Serial.print(kp);
    delay(250); 
  }
}

float constrain_(float value, float minValue, float maxValue)
{
  if(value > minValue)  value = minValue;
  else if(value > maxValue) value = maxValue;
  return value;
}



