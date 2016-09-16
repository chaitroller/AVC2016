/* Autonomous Car Code

    This code was written for the Sparkfun AVC competition
  Chaitanya.Sanghadia@gmail.com "Chai"
  matt.chandlerAZ@gmail.com "Matt"
  Pranav.Sanghadia@gmail.com "Pranav"

*/

#include <Servo.h>
#include "constants.h"
#include "utils.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

//Speeds for motor
#define NEUTRAL 1500
#define MAX_SPEED 1580
#define TURN_SPEED 1570
#define REVERSE_MAX 1000

// Create an instance of compass
Adafruit_BNO055 bno = Adafruit_BNO055(55);

const byte interruptPin = 2;      // For RPM Sensor to measure the distance travelled

// Create Two objects, Drive and Steering
ESC_Throttle servoSteering;    //
ESC_Throttle servoDrive;       //

bool flag1 = true;
int gDistance = 0;
int gAngle = 0;

// timeDelay account using milliSec for future use
//==================================================
unsigned long previousMillis = 0;

bool offCourse = false; // Set separately via serial input, triggers to remap

unsigned long fileNameTime;
String fileName;

///////////////////////////
// setup()
//////////////////////////
void setup()
{
  Serial.begin(115200);

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

  delay(500);

  setupLidar();
  
  /////////////////////////////////

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
  //Start Electronic Speed Controller (ESC): The delay is required for Throttle
  servoDrive.applyThrottle(0);
  //drive.writeMicroseconds(0);                  //  ??? Why zero
  delay(1000);
  servoDrive.applyThrottle(NEUTRAL);
  //drive.writeMicroseconds(NEUTRAL);            //
  delay(500);

  Serial.println(" 3. Set Speed and Steering Angle");
  servoSteering.setAngle(STEER_STRAIGHT); //(STEER_STRAIGHT);
  servoDrive.applyThrottle(MAX_SPEED);


}

//int MapAngle[] = {90, 90, 90, 90, 90, 45, 90, 45, 90, 45};
//int MapDistance[COURSELEGS] = {148, 55, 68, 63, 77, 118, 10, 30, 10, 15, 10};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Main Loop:
//  SparkFun Competition Track:
//  Distance: 148  55   68   63   77   118  10  30? 10? 15?
//  Turn    : 90R  90R  90L  90R  90R  90R  90L 90L 90R FINISH
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LEG0  12*5
#define LEG1  12*5
#define LEG2  12*5
#define LEG3  12*5
#define LEG4  12*5
#define LEG5  12*10
#define LEG6  12*10
#define LEG7  12*10
#define LEG8  12*10
int MapDistance[] = {LEG0, LEG1, LEG2, LEG3, LEG4, LEG5, LEG6, LEG7, LEG8};

int MapTurnAngle[] = {90, 90, 90, 90, 90, 90, 90, 90};
int MapDir[] = {TURN_RIGHT, TURN_RIGHT, TURN_RIGHT, TURN_RIGHT, TURN_RIGHT, TURN_RIGHT, TURN_LEFT, TURN_RIGHT, TURN_RIGHT};

bool avoidObstacle = false;
////////////////////////////////////////////////////////////////////////
// main program
//
///////////////////////////////////////////////////////////////////////
void loop() {

  readLidar();
  
  //if(gAngle == 180)
    Serial.println(gDistance);
  //return;

  if (LEG_NO == 4) {
    servoDrive.applyThrottle(NEUTRAL);
    return;
  }

  g_currentAngle = readDirection();      //Read magnetometer

  if (flag1) {
    g_targetAngle = g_currentAngle;
    flag1 = false;
  }
  
  /*
    if(distance180 < 10) {
      int saved_g_targetAngle = g_targetAngle;
      avoidObstacle = true;
      if(distance135 >= 15) {

        g_targetAngle += 30;           // Turn Right

      } else if(distance225 >= 15) {

        g_targetAngle -= 30;          // Turn Left

      }

    }
  */
  ////////////////////////////////////
  // Take turn OR do course correction
  ////////////////////////////////////
  if (setAngle(g_currentAngle, g_targetAngle) > 10) {      // Course correct if AV is drifting more than 10 degrees

    servoDrive.applyThrottle(TURN_SPEED);    // Slow down

    while ( setAngle(g_currentAngle, g_targetAngle) > 5) {       // Take turn or course correct
      g_currentAngle = readDirection();      //Read magnetometer
    }    // Set the target angle

    servoDrive.applyThrottle(MAX_SPEED);      // Resume normal/full speed
  }
  /*
    if(avoidObstacle) {

      g_targetAngle = saved_g_targetAngle;
      avoidObstacle = false;

    }
  */
  //////////////////////////////////////////////////////
  // Check the distance travelled, Move to the next LEG
  //////////////////////////////////////////////////////
  if ( g_distance_travelled >= MapDistance[LEG_NO]) {        // Check if current leg is over
    LEG_NO++;                                                // Move to next LEG
    g_targetAngle = getNewAngle(g_targetAngle, MapTurnAngle[LEG_NO], MapDir[LEG_NO]);
    g_rotation = 0;
    Serial.print("LEG_NO = "); Serial.println(LEG_NO); //Serial.print(" ");
  }

}


////////////////////////////////////////////////////////
// Interrupt Service Routine : Intrrupt Service Routine,
// Sensor output is connected to interrupt pin 2
///////////////////////////////////////////////////////
void rpm() {
  g_distance_travelled = 5.3 * g_rotation++;          // Based on the experiments on the friction with tile (indoor)
}


