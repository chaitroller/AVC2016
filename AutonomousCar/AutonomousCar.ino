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
#include <Adafruit_Sensor.h>
//#include <RobotGeekLCD.h>
#include <HMC5883L.h>

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

const int chipSelect = 4;
const byte interruptPin = 2;      // For RPM Sensor to measure the distance travelled

// Create Instances:
HMC5883L compass;

//RobotGeekLCD lcd;

// Create Two objects, Drive and Steering
ESC_Throttle servoSteering;    // 15 mSec delay  between throttle pulses
ESC_Throttle servoDrive;       // 10 miSec delay between throttle pulses

unsigned long time1;

int g_minAngle = 0;
int g_maxAngle = 0;
//int g_driftAngle = 5;

int g_current_speed = 0;
int g_Angle_UT = 0; //startAngle * 0.11; // 10% upper threshold
int g_Angle_LT = 0; //startAngle * 0.9; // 10% lower threshold
bool flag1 = true;

// timeDelay account using milliSec for future use
//==================================================
unsigned long previousMillis = 0;

//Setup course mapping arrays (index = Leg)
int MapDistance[COURSELEGS] = {LEG1, LEG2, LEG3, LEG4, LEG5, LEG6, LEG7, LEG8, LEG9};

int MapSpeed[COURSELEGS] = {FASTSPEED, CRUISESPEED, CRUISESPEED, CRUISESPEED, CRUISESPEED, SLOWSPEED, SLOWSPEED, SLOWSPEED, FASTSPEED};

//char MapTurnsAtEndOfLeg[COURSELEGS] = {RIGHT, RIGHT, LEFT, RIGHT, RIGHT, LEFT, LEFT, RIGHT, RIGHT };

int MapTurnAngleAtEndOfLeg[COURSELEGS] = {90, 90, 90, 90, 45, 90, 45, 90, 45};

char MapOBSTACLES[COURSELEGS] = {OBSTACLENONE, OBSTACLENONE, OBSTACLEBARREL, OBSTACLEHOOP, OBSTACLERAMP, OBSTACLENONE, OBSTACLENONE, OBSTACLENONE, OBSTACLENONE};

bool offCourse = false; // Set separately via serial input, triggers to remap

unsigned long fileNameTime;
//File dataFile;
String fileName;

void setup()
{
  Serial.begin(9600);

  //  Serial.print("Initializing SD card...");
  //
  //  // see if the card is present and can be initialized:
  //  if (!SD.begin(chipSelect)) {
  //    Serial.println("Card failed, or not present");
  //    // don't do anything more:
  //    return;
  //  }
  //  Serial.println("card initialized.");
  //
  //  fileName = String(millis());
  //  fileName += ".";
  //  fileName += "txt";
  //  dataFile = SD.open(String(fileName), FILE_WRITE);
  //  dataFile.println("===== Autonomoous Car Debug Messages =====");
  //  dataFile.close();


  // Interrupt at Falling Edge to read the RPM Sensor
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rpm, FALLING);

  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  setupCompass();

  // Attach PWM Pins to steering and drive
  ////////////////////////////////////////
  servoDrive.Attach(PINDRIVE);
  servoSteering.Attach(PINSTEER);

  // Initialize the Throttle
  //////////////////////////
  g_current_speed = NEUTRAL;
  //Start Electronic Speed Controller (ESC): The delay is required for Throttle
  servoDrive.driveMotor(0);
  //drive.writeMicroseconds(0);                  //  ??? Why zero
  delay(1000);
  servoDrive.driveMotor(NEUTRAL);
  //drive.writeMicroseconds(NEUTRAL);            //
  delay(500);

  servoSteering.setAngle(95); //(STEER_STRAIGHT);

  // initlaize lcd object - this sets up all the variables and IIC setup for the LCD object to work
  //lcd.init();
  // Print a message to the LCD.
  //lcd.print("Autonomous Car");

  servoDrive.driveMotor(1650);
}


/////////////////////////////////////////////////////////
//  Main Loop:
//
//
////////////////////////////////////////////////////////
int myCounter = 0;
void loop() {

  g_currentAngle = readDirection( compass );      //Read magnetometer
  Serial.print(": CurrentANgle = "); Serial.println(g_currentAngle);
  Serial.print("DistanceTravelled = "); Serial.println(g_distance_travelled);
 
  if (flag1) {
    servoDrive.driveMotor(1650);        // Set the speed once, the motor will keep running at this speed
    g_startAngle = g_currentAngle;
    flag1 = false;
  }

  // Each LEG is defined in a case, taking turn is also a case so that there is no course correction while turning
  // LIDAR: We can add lidar correction in case statement, hopefully not while taking turn but in case 0
  // Distance is updagetd by an ISR which uses Pin # 2 interrupt
  switch (LEG_NO) {

    case 0:                                            // LEG 1
      if (g_distance_travelled > MapDistance[LEG_NO]) {
        g_startAngle = getNewAngle(g_startAngle, MapTurnAngleAtEndOfLeg[LEG_NO], TURN_RIGHT);    // 1 = RIGHT TURN
        LEG_NO++;
        debugPrint_0_ToSerial(g_startAngle);
      } else {
        courseCorrection(STEER_STRAIGHT);    // Course correct when the vehicle is going straight
      }
      break;

    case 1:                                                  // Take Turn
      takeTurn(g_startAngle);
      break;

    case 2:
      if (++myCounter < 2) {
        Serial.print("case 2 : myCounter =  "); Serial.println(myCounter);
        LEG_NO = 0;        // Go back to Case 0 to restart the same loop
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


