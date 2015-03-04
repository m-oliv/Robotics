/*
 * PID3piLineFollower - demo code for the Pololu 3pi Robot
 * 
 * This code will follow a black line on a white background, using a
 * PID-based algorithm.
 *
 * http://www.pololu.com/docs/0J21
 * http://www.pololu.com
 * http://forum.pololu.com
 *
 */

// The following libraries will be needed by this demo
#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

Pololu3pi robot;
unsigned int sensors[5]; // an array to hold sensor values
unsigned int last_proportional = 0;
long integral = 0;
/*cHECKPOINTS conta o número de tiras pretas na pista.*/
int CHECKPOINTS=0;
/*Nº de tiras na pista*/
int TRACK_N_CHECKPOINTS=2;
/*nº de voltas até parar*/
int TRACK_N_LAPS=12;
/* 
TRACK_POSITION - posicao
TRACK_VARIANCE - variancia
TRACK_UNCERTAINTY - incerteza
TRACK_FULL_LAP - tamanho de uma volta completa na medida do 3pi


*/
float CONVERSION_RATE = 0.401; //unidades para centimetros
float CONVERSION_RATE_VELOCITY = 0.2363; //unidades para centimetros

float TRACK_POSITION = 0.0;
float TRACK_FULL_LAP = 477.0 * CONVERSION_RATE;
//1 UNIDADE DE POSIÇÃO=0.401CM

float TRACK_VARIANCE = 5.0; // Variancia inicial
float TRACK_ERROR = 1.0;    //Variancia dos sensores
float TRACK_UNCERTAINTY = 10.0; //Variancia
/*
State defines the curent bot state.
0 = Line; the robot is currently walking on a line.
1 = Full; The robot is either on a black hole or facing a T-Shaped path.
*/
int ROBOT_STATE=0;
/*
The base robot speed; it will be tweaked according to calibration.
*/
int ROBOT_SPEED=60;

/* 
TIME_READ - o tempo actual
TIME_PREVIOUS - o tempo lido na ultima vez
TIME_CONVERT - tempo do movimento (em s) */
long TIME_READ, TIME_PREVIOUS= 0;
float TIME_CONVERT = 0.0; 
int TIME_DELTA = 0;

/*Estas variáveis são usadas quando apanhamos preto pela primeira vez.*/

/*TIME_ASWHITE guarda quantos ms de branco o robot obteve.*/
long TIME_ASWHITE=0;
/*TIME_ASBLACK guarda quantos ms de branco o robot obteve.*/
long TIME_ASBLACK=0;
/*TIME_VERIFY guarda o momento em que verificamos se lemos
suficiente preto para o branco que lemos.*/
////prteo->branco verify
long TIME_VERIFY=50 ;
/*Em milissegundos, quanto tempo é o mínimo de preto que temos
de ler para o branco?*/
//braco->preto minimo
long TIME_TOLERANCE=13;

int MARGEM_ERRO_SUPER=1800;
int MARGEM_ERRO_INFER=200;
// This include file allows data to be stored in program space.  The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

// Introductory messages.  The "PROGMEM" identifier causes the data to
// go into program space.
const char welcome_line1[] PROGMEM = " Pololu";
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "PID Line";
const char demo_name_line2[] PROGMEM = "follower";

// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";

const char go[] PROGMEM = "L16 cdegreg4";
const char par[] PROGMEM = "<C6R<C6";
const char impar[] PROGMEM = "C8";
// Data for generating the characters used in load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char levels[] PROGMEM = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 7 levels of a bar graph.
void load_custom_characters()
{
  OrangutanLCD::loadCustomCharacter(levels + 0, 0); // no offset, e.g. one bar
  OrangutanLCD::loadCustomCharacter(levels + 1, 1); // two bars
  OrangutanLCD::loadCustomCharacter(levels + 2, 2); // etc...
  OrangutanLCD::loadCustomCharacter(levels + 3, 3);
  OrangutanLCD::loadCustomCharacter(levels + 4, 4);
  OrangutanLCD::loadCustomCharacter(levels + 5, 5);
  OrangutanLCD::loadCustomCharacter(levels + 6, 6);
  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
}

// This function displays the sensor readings using a bar graph.
void display_readings(const unsigned int *calibrated_values)
{
  unsigned char i;

  for (i=0;i<5;i++) {
    // Initialize the array of characters that we will use for the
    // graph.  Using the space, an extra copy of the one-bar
    // character, and character 255 (a full black box), we get 10
    // characters in the array.
    const char display_characters[10] = { ' ', 0, 0, 1, 2, 3, 4, 5, 6, 255 };

    // The variable c will have values from 0 to 9, since
    // calibrated values are in the range of 0 to 1000, and
    // 1000/101 is 9 with integer math.
    char c = display_characters[calibrated_values[i] / 101];

    // Display the bar graph character.
    OrangutanLCD::print(c);
  }
}

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.  This function is automatically called
// by the Arduino framework at the start of program execution.
void setup()
{
  unsigned int counter; // used as a simple timer

  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  robot.init(2000);

  load_custom_characters(); // load the custom characters

  // Play welcome music and display a message
  OrangutanLCD::printFromProgramSpace(welcome_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(welcome_line2);
  OrangutanBuzzer::playFromProgramSpace(welcome);
  delay(1000);

  OrangutanLCD::clear();
  OrangutanLCD::printFromProgramSpace(demo_name_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(demo_name_line2);
  delay(1000);

  // Display battery voltage and wait for button press
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    int bat = OrangutanAnalog::readBatteryMillivolts();

    OrangutanLCD::clear();
    OrangutanLCD::print(bat);
    OrangutanLCD::print("mV");
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print("Press B");

    delay(100);
  }

  // Always wait for the button to be released so that 3pi doesn't
  // start moving until your hand is away from it.
  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(1000);

  // Auto-calibration: turn right and left while calibrating the
  // sensors.
  for (counter=0; counter<80; counter++)
  {
    if (counter < 20 || counter >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);

    // This function records a set of sensor readings and keeps
    // track of the minimum and maximum values encountered.  The
    // IR_EMITTERS_ON argument means that the IR LEDs will be
    // turned on during the reading, which is usually what you
    // want.
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);

  // Display calibrated values as a bar graph.
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    // Read the sensor values and get the position measurement.
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

    // Display the position measurement, which will go from 0
    // (when the leftmost sensor is over the line) to 4000 (when
    // the rightmost sensor is over the line) on the 3pi, along
    // with a bar graph of the sensor readings.  This allows you
    // to make sure the robot is ready to go.
    OrangutanLCD::clear();
    OrangutanLCD::print(position);
    OrangutanLCD::gotoXY(0, 1);
    display_readings(sensors);

    delay(100);
  }
  OrangutanPushbuttons::waitForRelease(BUTTON_B);

  OrangutanLCD::clear();
  OrangutanLCD::print("Gooooooooo!");		

  // Play music and wait for it to finish before we start driving.
  OrangutanBuzzer::playFromProgramSpace(go);
  while(OrangutanBuzzer::isPlaying());
}

// The main function.  This function is repeatedly called by
// the Arduino framework.
void loop()
{
  // Get the position of the line.  Note that we *must* provide
  // the "sensors" argument to read_line() here, even though we
  // are not interested in the individual sensor readings.
  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

  float erro = (3.1416/4.0)/2000.0*((float)position - 2000.0);
  
  float k=14; //a calibrar + tarde
  float r=4.5;
  float vc =60.0; //velocidade base
  
  // quanto tem de girar
  float w= k*erro;
  // ajuste as velocidades
  float vl= (w*r)+(vc);
  float vr= (-w*r)+vc;
   
  // media das velocidades
  int vx = (vl + vr) / 2;
  int sensor_preto = 0;
  int n=0;
  unsigned int sensor_values[5];
  // ler os valores dos sensores para um array
  read_line_sensors(sensor_values,IR_EMITTERS_ON);
  // ver quando todos os sensores estão a preto
  
  //ESTADOS
  /*Quanto tempo decorreu?*/
  TIME_READ=millis();
  TIME_DELTA=TIME_READ-TIME_PREVIOUS;
  
  TIME_CONVERT = TIME_DELTA/1000.0; 
  

  // Calcular posição
  TRACK_POSITION = TRACK_POSITION + ((vl+vr)/2 * CONVERSION_RATE_VELOCITY) * TIME_CONVERT;                  
  // Calcular variância
  TRACK_VARIANCE = TRACK_VARIANCE + (TIME_CONVERT*TIME_CONVERT)*(TRACK_UNCERTAINTY);
      
  
  //Verificar se todos os sensores estão a ver preto
    for(n;n<5;n++)
      {
        if(sensor_values[n]>MARGEM_ERRO_SUPER)
        {
          sensor_preto++;
        }//se um nao for preto ja não ve mais nenhum sensor
        else
        {
          break;
        }
      }
	/*
    Ao VER tudo preto, queremos:
    	- Contar quanto tempo lemos tudo preto do sensor (TIME_ASBLACK);
        - Continuar no estado 1: estamos a ver tudo preto
        
    */
	if(CHECKPOINTS!= (TRACK_N_CHECKPOINTS*TRACK_N_LAPS+1) )
    {
		if(sensor_preto==5) 
    	  {
        	 ROBOT_STATE=1; //tudo a preto
             TIME_ASBLACK += TIME_DELTA;
          }  
         /* Ao VER branco:
            - Contar quanto tempo lemos branco do sensor (TIME_ASWHITE);
                - SE for tão grande como TIME_VERIFY, vamos verificar se estamos num checkpoint.
            - 
         */ 
         else
         {
            TIME_ASWHITE+= TIME_DELTA;
            if(TIME_ASWHITE>= TIME_VERIFY)//ja nao esta numa linha com tudo a preto há tempo suficiente
            {
              if(TIME_ASBLACK>=TIME_TOLERANCE) //Estamos num checkpoint porque lemos preto tempo suficiente!
              {
                  // beep quando o num de checkpoints e impar, duplo beep quando par
                  if((CHECKPOINTS%TRACK_N_CHECKPOINTS)==0) //rever para N>2
                  {
                    OrangutanBuzzer::playFromProgramSpace(impar);
                    // Estimativa da localização 2

                    TRACK_POSITION = TRACK_ERROR*(TRACK_POSITION) / (TRACK_ERROR + TRACK_VARIANCE);
                    TRACK_VARIANCE = TRACK_ERROR*TRACK_VARIANCE / (TRACK_ERROR + TRACK_VARIANCE);
                    if(CHECKPOINTS!=0)
                    {
                      TRACK_POSITION -= (TRACK_FULL_LAP*CONVERSION_RATE);
                    }
                  }
                  else
                  {
                    OrangutanBuzzer::playFromProgramSpace(par);
                    TRACK_POSITION = TRACK_ERROR*(TRACK_POSITION) / (TRACK_ERROR + TRACK_VARIANCE);
                    TRACK_VARIANCE = TRACK_ERROR*TRACK_VARIANCE / (TRACK_ERROR + TRACK_VARIANCE);
                  }
                //}
                CHECKPOINTS++;
                TIME_ASBLACK=0;
                TIME_ASWHITE=0;
                ROBOT_STATE=0;
                
                
              }
              else // nao estamos num checkpoint!
              {
                TIME_ASBLACK=0;
                TIME_ASWHITE=0;
                ROBOT_STATE=0;//estado --> tudo o que nao seja preto TODO: Linha
              }
            }
         }
      //Coloca o motor em movimento

      
      //print pos e var no LCD
        OrangutanLCD::clear();
        OrangutanLCD::print((long)TRACK_POSITION);
        OrangutanLCD::gotoXY(0, 1);
        OrangutanLCD::print((long)((TRACK_VARIANCE)*100.0));
    }
  	else
    {
      vl=0;
      vr=0;
    }
  
	OrangutanMotors::setSpeeds(vl,vr);
    TIME_PREVIOUS=TIME_READ;
}


