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

#define TURN_RIGHT  0
#define TURN_LEFT   1

//Servo Defines for Steering
#define STEER_LEFT      50
#define STEER_STRAIGHT  95
#define STEER_RIGHT     130


//Number of straightaways/journeys
#define COURSELEGS  9

#define OBSTACLENONE    0
#define OBSTACLEHOOP    1
#define OBSTACLEBARREL  2
#define OBSTACLERAMP    3


