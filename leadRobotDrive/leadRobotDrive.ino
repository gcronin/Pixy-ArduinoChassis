#include <Adafruit_TB6612.h>   

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
int driveSpeed = 40;
int turnSpeed = 80;

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
}

void loop()
{ 
      forward(leftWheel, rightWheel, driveSpeed);
      delay(random(2000, 8000));
      if(random(2))
      {
        left(leftWheel, rightWheel, turnSpeed);
      }
      else
      {
        right(leftWheel, rightWheel, turnSpeed);
      }
      delay(random(500, 2000));

}

