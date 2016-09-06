/* Autonomous Car Code
 *  
 *  This code was written for the Sparkfun AVC competition
  Chaitanya.Sanghadia@gmail.com "Chai"
  matt.chandlerAZ@gmail.com "Matt"

 */
 //-----------------------------------------------------------------
// -------------------- Set and Modify Constants  ------------------
//------------------------------------------------------------------
/*
   * 
   * MAPPING DETAILS
   * 10' wide paths

Mapping starts with direction to go, then what direction to turn

LEG / Distance / Direction to Turn / Obstacles on this leg

1 - 120' / GO RIGHT
2 - 45' / GO RIGHT
3 - 58' / GO LEFT / Barrells 
4 - 53' / GO RIGHT / Hoop
5 - 66' / GO RIGHT / Jump ramp
6 - 8' / GO LEFT  
7 - 28' / GO LEFT
8 - 8' / GO RIGHT
9 - 5' / GO RIGHT - FINISH

   * 
   */
   
// Rotation to distance Calculations
#define ROTATION_MULTIPLIER 0.22      // Muliplying # of rotation gives distance in Inches
   
//GPIO Assignments

#define PINDRIVE 12
#define PINSTEER 13

// To go X feet, trigger driver for Y time
#define DISTANCE_TO_TIME_RATIO  5

#define STEERING_DELAY_MS       100
#define THROTTLE_DELAY_MS       100

#define LEFT_MIN_ANGLE          30
#define RIGHT_MAX_ANGLE         150
#define STARTUP_DELAY_MS        500
#define STARTUP_SPEED           1700
#define DRIFTANGLE              10
#define NEUTRAL                 1500
#define FORWARD_MAX             2000
#define REVERSE_MAX             1000
#define DELAYAFTERLEFT          640
#define DELAYAFTERRIGHT         640
#define CRUISESPEED             1750
#define FASTSPEED               2000
#define SLOWSPEED               1000

#define TURN_RIGHT  0
#define TURN_LEFT   1

//enums for Directions
#define STRAIGHT    1
#define RIGHT1      5
#define LEFT1       5
#define BACKWARDS   4

//Servo Defines for Steering
#define STEER_LEFT      50
#define STEER_STRAIGHT  95
#define STEER_RIGHT     130


//Number of straightaways/journeys
#define COURSELEGS  9

//Distance to go after each turn

#define LEG1 36 //120
#define LEG2 36 //45
#define LEG3 48 //58
#define LEG4 48 //53
#define LEG5 66
#define LEG6 8
#define LEG7 28
#define LEG8 8
#define LEG9 5

#define OBSTACLENONE    0
#define OBSTACLEHOOP    1
#define OBSTACLEBARREL  2
#define OBSTACLERAMP    3


