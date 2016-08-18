/* Autonomous Car Code

    This code was written for the Sparkfun AVC competition
  Chaitanya.Sanghadia@gmail.com "Chai"
  matt.chandlerAZ@gmail.com "Matt"

*/

#include <Servo.h>
#include "constants.h"
#include "utils.h"

#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <RobotGeekLCD.h>


#define DEBUG_MODE 1
#define SIMULATION_MODE 0
//Steering
#define LEFT 70
#define STRAIGHT 95
#define RIGHT 110
//Speeds for motor
#define NEUTRAL 1500
#define FORWARD_MAX 2000
#define REVERSE_MAX 1000

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
unsigned long time;
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

const byte interruptPin = 2;
RobotGeekLCD lcd;

// Create Two objects, Drive and Steering
ESC_Throttle servoSteering;    // 15 mSec delay  between throttle pulses
ESC_Throttle servoDrive;       // 10 miSec delay between throttle pulses


void setup()
{
  Serial.begin(9600);
  // Interrupt at Falling Edge to read the RPM Sensor
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rpm, FALLING);
  /*
    #ifndef ESP8266
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
    #endif
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
  */
  // Attach PWM Pins to steering and drive
  ////////////////////////////////////////

  //steering.attach(13);    //Setup Steering
  //drive.attach(12);       //Setup Drive

  servoDrive.Attach(12);
  servoSteering.Attach(13);

  // Initialize the Throttle
  //////////////////////////
  current_speed = NEUTRAL;
  //Start Electronic Speed Controller (ESC): The delay is required for Throttle
  drive.writeMicroseconds(0);                  //  ??? Why zero
  delay(1000);
  drive.writeMicroseconds(NEUTRAL);            //
  delay(500);                                  // Delay
  /*
    int myAngle = -99;
    lsm.read();
    delay(600);
  */
  // initlaize the lcd object - this sets up all the variables and IIC setup for the LCD object to work
  lcd.init();
  // Print a message to the LCD.
  lcd.print("Hello, World!");
}


// QUESTION#1: Do I have to write to servo every 15 mSec if there is no change in the speed/duty-cycle?
//             following test writes to servo only once, with the start speed of 1700 the motor should
//             not stop, it should run as long as the power is ON.
//== == == == == == ==
bool flag1 = true;
void loop() {

  if (flag1) {
    servoDrive.GoForward(40);
    flag1 = false;
    Serial.println("Inside");
  }
}

// Try writing to the motor 1000 times; after that do nothing. I want to test if the motor continuous to run

int loopcount = 0;
void loop_test2() {

  if (loopcount++ < 1000) {
    servoDrive.GoForward(40);
  }
}



// Note: Following loop is temporary code used to implement and debug the distance calculation using RPM Sensor
// the parameter passed to GoForward() should be replaced with the LEG
void loop2() {
  //Serial.println(LEG_NO);
  //servoDrive.takeTurn(150);

  switch (LEG_NO) {

    case 0:
      servoDrive.GoForward(20);      // TBD: Add parameters 1) Distance (LEG), 2) Speed
      //servoSteering.takeTurn(130);
      Serial.print("LEG # : "); Serial.print(" ");
      Serial.println(LEG_NO);
      break;

    case 1:
      //Serial.println("Case 1 : ");
      //servoDrive.updateSpeed(1600);
      servoDrive.GoForward(40);      // TBD: Add parameters 1) Distance (LEG), 2) Speed
      servoSteering.takeTurn(70);
      servoSteering.takeTurn(95);
      Serial.print("LEG # : "); Serial.print(" ");
      Serial.println(LEG_NO);
      break;

    case 2:
      //servoDrive.updateSpeed(1600);
      servoDrive.GoForward(60);      // TBD: Add parameters 1) Distance (LEG), 2) Speed
      servoSteering.takeTurn(130);
      Serial.print("LEG # : "); Serial.print(" ");
      Serial.println(LEG_NO);
      break;

    case 3:
      //servoDrive.updateSpeed(1600);
      servoDrive.GoForward(80);      // TBD: Add parameters 1) Distance (LEG), 2) Speed
      servoSteering.takeTurn(70);
      servoSteering.takeTurn(95);
      Serial.print("LEG # : "); Serial.print(" ");
      Serial.println(LEG_NO);

      break;

    default:
      servoDrive.updateSpeed(1500);
      servoDrive.GoForward(1);      // TBD: Add parameters 1) Distance (LEG), 2) Speed

      Serial.println("======================= DONE ++++++++++++++++++++");
  }

  // LCD displays the distance or RPM for debugging purpose: TBD : Add LCD Class
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  lcd.print(rotation);
  //timeDelay(5);
}

// ISR : Intrrupt Service Routine, Sensor output is connected to interrupt pin 2
void rpm() {
  rotation++;
  distance_travelled = rotation * ROTATION_MULTIPLIER;

}

//============================================================================================
//==============================  MAIN WORKING LOOP ==========================================
//============================================================================================
void loop1()
{
  SetSteeringGoStraight();

  StartupThrottleEngine(STARTUP_SPEED, STARTUP_DELAY_MS);

  /* DESIGN:  For each leg of course:
        - Go the direction and speed indicated
        - Take a turn and continue to next iteration
        ---> IF LEG HAS OBSTACLES, RAMP, HOOP, ETC ...
        [
           -- SLOW TO STOP, SCAN, REMAP and CONTINUE

        ]

  */

  for (int i = 0; i < 3; i++)
  {
    currentAngle = GetDirection( lsm );      //Read magnetometer
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


