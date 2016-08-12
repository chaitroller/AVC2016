/* Autonomous Car Code
 *
 *  This code was written for the Sparkfun AVC competition
  Chaitanya.Sanghadia@gmail.com "Chai"
  matt.chandlerAZ@gmail.com "Matt"

 */
#include <Servo.h>
#include "constants.h"

#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();


#define DEBUG = true
#define SIMULATIONMODE = false
//Steering
#define LEFT 70
#define STRAIGHT 95
#define RIGHT 110
//Speeds for motor
#define NEUTRAL 1500
#define FORWARD_MAX 2000
#define REVERSE_MAX 1000

Servo steering;
Servo drive;



int current_speed = 0;

// timeDelay account using milliSec for future use
//==================================================
unsigned long previousMillis = 0;

//Setup course mapping arrays (index = Leg)
int MapDistance[COURSELEGS] = {LEG1, LEG2, LEG3, LEG4, LEG5, LEG6, LEG7, LEG8, LEG9};

int MapSpeed[COURSELEGS] = {FASTSPEED, CRUISESPEED, CRUISESPEED, CRUISESPEED, CRUISESPEED, SLOWSPEED, SLOWSPEED, SLOWSPEED, FASTSPEED};

char MapTurnsAtEndOfLeg[COURSELEGS] = {RIGHT, RIGHT, LEFT, RIGHT, RIGHT, LEFT, LEFT, RIGHT, RIGHT };

char MapOBSTACLES[COURSELEGS] = {OBSTACLENONE, OBSTACLENONE, OBSTACLEBARREL, OBSTACLEHOOP, OBSTACLERAMP, OBSTACLENONE, OBSTACLENONE, OBSTACLENONE, OBSTACLENONE};

bool offCourse = false; // Set separately via serial input, triggers to remap


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}


int minAngle = 0;
int maxAngle = 0;
int driftAngle = 5;
int currentAngle = 0;

void setup()
{

#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("LSM raw read demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");

  //Start ESC
  drive.writeMicroseconds(0);
  delay(1000);
  drive.writeMicroseconds(NEUTRAL);
  delay(500);
  //Setup Serial Sync
  //  link.setup();

  //Setup Steering
  steering.attach(13);

  //Setup Drive
  drive.attach(12);
  current_speed = NEUTRAL;


int myAngle = -99;

int firsttime = 1;

if (firsttime == 1) {
  firsttime = 0;
  lsm.read();
  delay(600);
  
}
}


//============================================================================================
//==============================  MAIN WORKING LOOP ==========================================
//============================================================================================
void loop()
{
  StartupAndAlign();

  StartupThrottleEngine(STARTUP_DELAY_MS,STARTUP_DELAY_MS);



  /* DESIGN:  For each leg of course:
   *    - Go the direction and speed indicated
   *    - Take a turn and continue to next iteration
   *    ---> IF LEG HAS OBSTACLES, RAMP, HOOP, ETC ...
   *    [
   *       -- SLOW TO STOP, SCAN, REMAP and CONTINUE
   *
   *    ]
   *
   */

  current_speed = 1650;


  StartupThrottleEngine(STARTUP_SPEED, STARTUP_DELAY_MS);

  //Read magnetometer


  for (int i = 0; i < 3; i++)
  {

    currentAngle = GetHeading( lsm );
    CourseCorrection(  minAngle,  maxAngle,  currentAngle);
    GoStraight(MapDistance[i], MapSpeed[i]);





    switch (MapTurnsAtEndOfLeg[i])
    {
      case LEFT:
        TurnLeft();
      case RIGHT:
        TurnRight();
    }

    NavigationRecheck(i);
  }

  TurnEngineOff(); // THE END !!
  exit(1);

}


