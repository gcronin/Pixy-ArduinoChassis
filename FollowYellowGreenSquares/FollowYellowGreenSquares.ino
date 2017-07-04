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
  Serial.print("Starting...\n");

  driveControl.reset();   //reset NXT motor encoders
  driveControl.stop(MMX_Motor_Both, MMX_Next_Action_Float);

  pixy.init();
  uint16_t w = 0;
  while(w != 0xaa55)
  {
    w = pixy.testConnection();
  }
  Serial.print("Found Pixy!...\n");
}

void loop()
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
      Serial.print("   Error: ");
      Serial.print(error);
      Serial.print("  Left: ");
      Serial.print(leftSpeed);
      Serial.print("  Right: ");
      Serial.println(rightSpeed);
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
      driveControl.stop(MMX_Motor_Both, MMX_Next_Action_Float);
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



