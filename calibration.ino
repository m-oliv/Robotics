#include <OrangutanMotors.h>

OrangutanMotors motor;

float mleft=0;
float mright=0;

void setup()
{
  mright = 20;
  mleft = 20;
  delay(4000);
  motor.setSpeeds(0, mright);
  delay(10000);
  motor.setSpeeds(0,0);
}

void loop()
{
  
}
