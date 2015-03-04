#include <OrangutanMotors.h>


OrangutanMotors motor;

int mleft=0;
float mright=0;

void anda(float cm){
  float t= (float)cm/16.61;
  int ms= t * 1000;
  mleft=40;
  mright=42.5;
  motor.setSpeeds(mleft, mright);
  delay(ms);
  mleft=0;
  mright=0;
  motor.setSpeeds(mleft, mright);
}

void gira(float grau){
  float arc = (2*3.14*grau*9.34)/360;
  float t= (float)arc/16.61;
  int ms= t * 1000;
  mleft=-40;
  mright=42.5;
  motor.setSpeeds(mleft, mright);
  delay(ms/2);
  mleft=0;
  mright=0;
  motor.setSpeeds(mleft, mright);
}

void setup()
{
  delay(10000);
}

void loop()
{
  anda(90.0);
  delay(2000);
  gira(90); 
}
