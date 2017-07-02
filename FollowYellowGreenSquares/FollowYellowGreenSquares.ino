#include <Adafruit_TB6612.h>   
#include <SPI.h>  
#include <Pixy.h>

// motor pins
#define AIN1 5
#define BIN1 9
#define AIN2 4
#define BIN2 10
#define PWMA 3
#define PWMB 11

// Initializing motors.  
Motor leftWheel = Motor(AIN1, AIN2, PWMA, 1);
Motor rightWheel = Motor(BIN1, BIN2, PWMB, 1);
float leftSpeed = 0;
float rightSpeed = 0;
int error = 0;
long delay_period = 1000;
long timestamp = 0;
int searchingCounter = 0;
float kp = 0.3;
float baseSpeed = 0.0;

// This is the main Pixy object 
Pixy pixy;

// Provides structure for state 
enum State { SEARCHING, SINGLE_LOCK_BLUE, SINGLE_LOCK_GREEN, DOUBLE_LOCK, ERROR_TILT, ERROR_SPURIOUS };
State state = SEARCHING;

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

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
      brake(leftWheel, rightWheel);
      break;
    case ERROR_SPURIOUS:
      brake(leftWheel, rightWheel);
      break;
    case SEARCHING:
      brake(leftWheel, rightWheel);
      break;
    case SINGLE_LOCK_BLUE:
    case SINGLE_LOCK_GREEN:
    case DOUBLE_LOCK:
      leftSpeed = baseSpeed + kp * error;
      rightSpeed = baseSpeed - kp * error;
      leftWheel.drive(leftSpeed);
      rightWheel.drive(rightSpeed);
      break;
  }

}

