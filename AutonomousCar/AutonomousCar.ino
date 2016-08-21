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
#include <Adafruit_Sensor.h>
#include <RobotGeekLCD.h>
#include <HMC5883L.h>


#define DEBUG_MODE 1
#define SIMULATION_MODE 0
//Steering
#define LEFT 70
//#define STRAIGHT 95
#define RIGHT 110
//Speeds for motor
#define NEUTRAL 1500
#define FORWARD_MAX 2000
#define REVERSE_MAX 1000

// Create Instances:
HMC5883L compass;

const byte interruptPin = 2;
RobotGeekLCD lcd;

// Create Two objects, Drive and Steering
ESC_Throttle servoSteering;    // 15 mSec delay  between throttle pulses
ESC_Throttle servoDrive;       // 10 miSec delay between throttle pulses

unsigned long time;

int g_minAngle = 0;
int g_maxAngle = 0;
//int g_driftAngle = 5;

int g_current_speed = 0;
int g_startAngleUpperThreshold = 0; //startAngle * 0.11; // 10% upper threshold
int g_startAngleLowerThreshold = 0; //startAngle * 0.9; // 10% lower threshold
bool flag1 = true;

// timeDelay account using milliSec for future use
//==================================================
unsigned long previousMillis = 0;

//Setup course mapping arrays (index = Leg)
int MapDistance[COURSELEGS] = {LEG1, LEG2, LEG3, LEG4, LEG5, LEG6, LEG7, LEG8, LEG9};

int MapSpeed[COURSELEGS] = {FASTSPEED, CRUISESPEED, CRUISESPEED, CRUISESPEED, CRUISESPEED, SLOWSPEED, SLOWSPEED, SLOWSPEED, FASTSPEED};

char MapTurnsAtEndOfLeg[COURSELEGS] = {RIGHT, RIGHT, LEFT, RIGHT, RIGHT, LEFT, LEFT, RIGHT, RIGHT };

int MapTurnAngleAtEndOfLeg[COURSELEGS] = {45, 90, 45, 90, 45, 90, 45, 90, 45};

char MapOBSTACLES[COURSELEGS] = {OBSTACLENONE, OBSTACLENONE, OBSTACLEBARREL, OBSTACLEHOOP, OBSTACLERAMP, OBSTACLENONE, OBSTACLENONE, OBSTACLENONE, OBSTACLENONE};

bool offCourse = false; // Set separately via serial input, triggers to remap




void setup()
{
  Serial.begin(9600);
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

  servoSteering.setAngle(STEER_STRAIGHT);

  // initlaize lcd object - this sets up all the variables and IIC setup for the LCD object to work
  lcd.init();
  // Print a message to the LCD.
  lcd.print("Autonomous Car");
}


void loop() {

  if (flag1) {
    //servoDrive.GoForward(40);
    servoDrive.driveMotor(1700);        // Set the speed once, the motor will keep running at this speed
    flag1 = false;
    //Serial.println("Inside");
  }

  //Serial.println(LEG_NO);
  g_currentAngle = readDirection( compass );      //Read magnetometer
  //Serial.println(currentAngle); Serial.print("  : ");
  if (g_updateStartAngle) {                  // Update only once for a LEG
    g_startAngle = g_currentAngle;

    g_startAngleUpperThreshold = getUpperThresholdAngle(g_startAngle, 10); //g_startAngle * 1.1; // 10% upper threshold
    g_startAngleLowerThreshold = getLowerThresholdAngle(g_startAngle, 10); //g_startAngle * 0.9; // 10% lower threshold

    g_updateStartAngle = false;
  }

  // Each LEG is defined in a case, taking turn is also a case so that there is no course correction while turning
  // LIDAR: We can add lidar correction in case statement, hopefully not while taking turn but in case 0
  // Distance is updagetd by an ISR which uses Pin # 2 interrupt
  switch (LEG_NO) {

    case 0:                                            // LEG 1
      //distance_travelled += 0.01;
      Serial.println(g_distance_travelled);
      if (g_distance_travelled > MapDistance[LEG_NO]) {
        Serial.print("Start Angle = "); Serial.print(g_startAngle); Serial.print(" ");
        g_startAngle = getNewAngle(g_startAngle, MapTurnAngleAtEndOfLeg[LEG_NO], RIGHT);    // 1 = RIGHT TURN
        Serial.print("New Start Angle = "); Serial.println(g_startAngle);
        g_startAngleUpperThreshold = getUpperThresholdAngle(g_startAngle, 10); //g_startAngle * 1.1; // 10% upper threshold
        g_startAngleLowerThreshold = getLowerThresholdAngle(g_startAngle, 10); //g_startAngle * 0.9; // 10% lower threshold
        LEG_NO++;
        //lcd.setCursor(0, 1); lcd.print(g_distance_travelled);
        //g_distance_travelled = 0;
      } else {
        //Serial.println("CourseCorret");
        courseCorrection();                // Course correct when the vehicle is going straight
      }

      break;

    case 1:                                          // Take Turn
      Serial.print("Current Angle = "); Serial.print(g_currentAngle); Serial.print(" ");
      Serial.print("Upper Threshold = "); Serial.print(g_startAngleUpperThreshold); Serial.print(" ");
      Serial.print("Lower Threshold = "); Serial.println(g_startAngleLowerThreshold);

      if ( (g_currentAngle <= g_startAngleUpperThreshold) && (g_currentAngle >= g_startAngleLowerThreshold) ) {
        servoSteering.setAngle(STEER_STRAIGHT);                  // Set the servo angle to go straight
        LEG_NO++;
        lcd.setCursor(11, 1);
        lcd.print(200);

      }
      else {
        slowTurn(RIGHT);
        lcd.setCursor(0, 1);
        lcd.print(100);
        lcd.setCursor(4, 1);
        lcd.print(g_currentAngle);


      }

      break;

    case 2:
      LEG_NO = 10;        // Go back to Case 0 to restart the same loop
      g_distance_travelled = 0;
      Serial.println("-------END-------");
      //return;
      //servoDrive.timeDelay(5000);
      //servoDrive.driveMotor(1700);
      //      if (distance_travelled == MapDistance[LEG_NO]) {
      //        newAngle = currentAngle + 90;
      //        LEG_NO++;
      //        lcd.setCursor(0, 1); lcd.print(distance_travelled);
      //      }
      break;

    case 3:
      // servoDrive.timeDelay(5000);
      //servoDrive.driveMotor(1650);
      if (g_distance_travelled == MapDistance[LEG_NO]) {
        newAngle = g_currentAngle + 90;
        LEG_NO++;
        //lcd.setCursor(0, 1); lcd.print(g_distance_travelled);
      }
      break;

    case 4:
      //servoDrive.driveMotor(1700);
      if (g_distance_travelled == MapDistance[LEG_NO]) {
        newAngle = g_currentAngle + 90;
        LEG_NO++;
        //lcd.setCursor(0, 1); lcd.print(g_distance_travelled);
      }
      break;

    default:
      //LEG_NO = 0;
      servoDrive.driveMotor(1500);
      //servoDrive.updateSpeed(1500);
      //servoDrive.GoForward(1);      // TBD: Add parameters 1) Distance (LEG), 2) Speed
      //Serial.println("======================= DONE ++++++++++++++++++++");
      break;
  }

  // LCD displays the distance or RPM for debugging purpose: TBD : Add LCD Class
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  //  lcd.setCursor(0, 1);
  //  lcd.print(g_currentAngle);
  //Serial.println(diffAngle);
}

// ISR : Intrrupt Service Routine, Sensor output is connected to interrupt pin 2
void rpm() {
  g_distance_travelled = 6 * g_rotation++;          // Based on the experiments on the friction with tile (indoor)
}


