#include <OrangutanMotors.h>

OrangutanMotors motor;

// velocidade angular (va) - entre 0 e 120
void anda(float va,float vc){
  va = va / 10;
  float mleft = va + vc;
  float mright = -va + vc + 5;
  motor.setSpeeds(mleft, mright);
}

void setup()
{
  delay(10000);
}

void loop()
{
  anda(0,80);
  delay(4000);
  motor.setSpeeds(0,0);
}
