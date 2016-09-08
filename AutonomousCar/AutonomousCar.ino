/* Autonomous Car Code

    This code was written for the Sparkfun AVC competition
  Chaitanya.Sanghadia@gmail.com "Chai"
  matt.chandlerAZ@gmail.com "Matt"

*/

#include <Servo.h>
#include "constants.h"
#include "utils.h"

#include <Wire.h>
#include <SPI.h>
//#include <SD.h>


//#include <RobotGeekLCD.h>
//#include <HMC5883L.h>

#define HMC5883
////////////// Compass Code ///////////////
#ifdef HMC5883
#include "compass.h"
#define Task_t 10          // Task Time in milli seconds
int dt = 0;
unsigned long t;
#else
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
#endif


#define DEBUG_MODE 1
#define SIMULATION_MODE 0
//Steering
#define LEFT_TURN_ANGLE 70
//#define STRAIGHT 95
#define RIGHT_TURN_ANGLE 110
//Speeds for motor
#define NEUTRAL 1500
#define FORWARD_MAX 2000
#define REVERSE_MAX 1000

const byte interruptPin = 2;      // For RPM Sensor to measure the distance travelled

// Create Instances:
//HMC5883L compass;
// i2c

//RobotGeekLCD lcd;

// Create Two objects, Drive and Steering
ESC_Throttle servoSteering;    //
ESC_Throttle servoDrive;       //

unsigned long time1;

//int g_minAngle = 0;
//int g_maxAngle = 0;
//int g_driftAngle = 5;

int g_current_speed = 0;
bool flag1 = true;

// timeDelay account using milliSec for future use
//==================================================
unsigned long previousMillis = 0;

//Setup course mapping arrays (index = Leg)
int MapDistance[COURSELEGS] = {LEG1, LEG2, LEG3, LEG4, LEG5, LEG6, LEG7, LEG8, LEG9};

//int MapSpeed[COURSELEGS] = {FASTSPEED, CRUISESPEED, CRUISESPEED, CRUISESPEED, CRUISESPEED, SLOWSPEED, SLOWSPEED, SLOWSPEED, FASTSPEED};

//char MapTurnsAtEndOfLeg[COURSELEGS] = {RIGHT, RIGHT, LEFT, RIGHT, RIGHT, LEFT, LEFT, RIGHT, RIGHT };

int MapTurnAngleAtEndOfLeg[COURSELEGS] = {90, 90, 90, 90, 45, 90, 45, 90, 45};

//char MapOBSTACLES[COURSELEGS] = {OBSTACLENONE, OBSTACLENONE, OBSTACLEBARREL, OBSTACLEHOOP, OBSTACLERAMP, OBSTACLENONE, OBSTACLENONE, OBSTACLENONE, OBSTACLENONE};

bool offCourse = false; // Set separately via serial input, triggers to remap

unsigned long fileNameTime;
//File dataFile;
String fileName;

void setup()
{

#ifdef HMC5883
  Serial.begin(9600);
  Serial.print("L: Setting up I2C ........\n");
  Wire.begin();
  compass_x_offset = 122.17;
  compass_y_offset = 230.08;
  compass_z_offset = 389.85;
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;

  compass_init(2);
  compass_debug = 1;
  compass_offset_calibration(3);
#else                             // LSM9DS0

#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  Serial.println("=== LSM9DS0 9DOF Initialized");

  setupCompass_LSM9DS0();   // Setup LMSDS0
#endif

  // Interrupt at Falling Edge to read the RPM Sensor
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rpm, FALLING);


  // Attach PWM Pins to steering and drive
  ////////////////////////////////////////
  Serial.println(" 2. Initialize Servo Motors");
  servoDrive.Attach(PINDRIVE);
  servoSteering.Attach(PINSTEER);

  // Initialize the Throttle
  //////////////////////////
  Serial.println(" 2. Initialize Throttle");
  g_current_speed = NEUTRAL;
  //Start Electronic Speed Controller (ESC): The delay is required for Throttle
  servoDrive.driveMotor(0);
  //drive.writeMicroseconds(0);                  //  ??? Why zero
  delay(1000);
  servoDrive.driveMotor(NEUTRAL);
  //drive.writeMicroseconds(NEUTRAL);            //
  delay(500);

  Serial.println(" 3. Set Speed and Steering Angle");
  servoSteering.setAngle(STEER_STRAIGHT); //(STEER_STRAIGHT);
  servoDrive.driveMotor(1650);
}


/////////////////////////////////////////////////////////
//  Main Loop:
//
//
////////////////////////////////////////////////////////
int myCounter = 0;
void loop() {

  g_currentAngle = readDirection();      //Read magnetometer
  Serial.print("CurrentAngle = "); Serial.println(g_currentAngle);
  //Serial.print("DistanceTravelled = "); Serial.println(g_distance_travelled);

  if (flag1) {
    servoDrive.driveMotor(1650);        // Set the speed once, the motor will keep running at this speed
    g_startAngle = g_currentAngle;
    flag1 = false;
  }

   // Distance is updagetd by an ISR which uses Pin # 2 interrupt
  switch (LEG_NO) {

    case 0:                                            // LEG 1
      if (g_distance_travelled > MapDistance[LEG_NO]) {
        g_startAngle = getNewAngle(g_startAngle, MapTurnAngleAtEndOfLeg[LEG_NO], TURN_RIGHT);    // 1 = RIGHT TURN
        LEG_NO++;
        MODE++;
        //debugPrint_0_ToSerial(g_startAngle);
      } else {
        setAngle(g_startAngle);    // Course correct when the vehicle is going straight
      }
      break;

    case 1:                                                  // Take Turn
      if (setAngle(g_startAngle) < 5)     // Returns error between Current Angle and the Goal Angle
        LEG_NO++;
      MODE++;
      break;

    case 2:
      if (++myCounter < 1) {
        Serial.print("case 2 : myCounter =  "); Serial.println(myCounter);
        LEG_NO++;
        MODE = 0;        // Go back to Case 0 to restart the same loop
        g_distance_travelled = 0;
        g_rotation = 0;
      } else {
        servoDrive.driveMotor(1500);
        Serial.println("-------END-------");
      }

      break;

    default:
      servoDrive.driveMotor(1500);    // Stop
      break;
  }

}

// ISR : Intrrupt Service Routine, Sensor output is connected to interrupt pin 2
void rpm() {
  g_distance_travelled = 6 * g_rotation++;          // Based on the experiments on the friction with tile (indoor)
}


