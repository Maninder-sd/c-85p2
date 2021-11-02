/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization

 This file provides the implementation of all the functionality required for the
 EV3 robot localization project. Please read through this file carefully, and
 note the sections where you must implement functionality for your bot.

 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are
 expected to develop high wuality, clean code. Test your code extensively with
 valgrind, and make sure its memory management is clean.

 In a nutshell, the starter code provides:

 * Reading a map from an input image (in .ppm format). The map is bordered with
 red, must have black streets with yellow intersections, and buildings must be
 either blue, green, or be left white (no building).

 * Setting up an array with map information which contains, for each
 intersection, the colours of the buildings around it in ** CLOCKWISE ** order
 from the top-left.

 * Initialization of the EV3 robot (opening a socket and setting up the
 communication between your laptop and your bot)

 What you must implement:

 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot
 must not wander outside the map (though of course it's possible parts of the
 robot will leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to
 determine its location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different
 intersections in a sequence that allows it to achieve reliable localization

 * Basic path planning - once the robot has found its location, it must drive
 toward a user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---

  The starter code provides a skeleton for implementing a sensor calibration
 routine, it is called when the code receives -1  -1 as target coordinates. The
 goal of this function should be to gather informatin about what the sensor
 reads for different colours under the particular map/room illumination/battery
 level conditions you are working on - it's entirely up to you how you want to
 do this, but note that careful calibration would make your work much easier, by
 allowing your robot to more robustly (and with fewer mistakes) interpret the
 sensor data into colours.

   --> The code will exit after calibration without running localization (no
 target!) SO - your calibration code must *save* the calibration information
 into a file, and you have to add code to main() to read and use this
            calibration data yourselves.

 What you need to understand thoroughly in order to complete this project:

 * The histogram localization method as discussed in lecture. The general steps
 of probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and
 unreliable, you have to handle this smartly

 * Robot control with feedback - your robot does not perform exact motions, you
 can assume there will be error and drift, your code has to handle this.

 * The robot control API you will use to get your robot to move, and to acquire
   sensor data. Please see the API directory and read through the header files
 and attached documentation

 Starter code:
 F. Estrada, 2018 - for CSC C85

*/

#include "EV3_Localization.h"

#include <inttypes.h>
#include <math.h>
#include <time.h>

#define MAX_SPEED 10
#define SENSOR_M_SPEED 8
#define NUM_SAMPLES 7
#define NUM_SMALL_SAMPLES 3
#define DO_COLOUR_READING -1

// Colours Code
#define BLACK 1
#define BLUE 2
#define GREEN 3
#define YELLOW 4
#define RED 5
#define WHITE 6
#define BROWN 7
#define NO_COLOUR 0

// Motor code
#define GYRO_PORT PORT_1
#define ARM_MOTOR MOTOR_C
#define LEFT_WHEEL_MOTOR MOTOR_D
#define RIGHT_WHEEL_MOTOR MOTOR_A

void road_find();
int traverse_road(int (*exit_conditon_func)(int));
int turn_till_black(int);
double curr_time();
void led_test();
int read_colour(int);
void get_colours_at_intersection();
void get_colours_at_intersection_2();
void print_belief(double beliefs[400][4]);

int intersection_or_boundary_detected(int colourID) {
  if (colourID == DO_COLOUR_READING) {
    colourID = read_colour(NUM_SAMPLES);
  }
  // printf("int or bound : %d\n", colourID);
  return (colourID == 5 || colourID == 4);  // change later
}
int boundary_detected(int colourID) {
  if (colourID == DO_COLOUR_READING) {
    colourID = read_colour(NUM_SAMPLES);
  }
  // printf("int or bound : %d\n", colourID);
  return (colourID == 5);  // change later
}
int road_detected(int colourID) {
  if (colourID == DO_COLOUR_READING) {
    return (read_colour(NUM_SAMPLES) == 1);
  }
  return (colourID == 1);
}

int blue_tone[50][3];
int white_tone[50][3];
int green_tone[50][3];

int P_BLUE_INDEX = 0;
int P_WHITE_INDEX = 1;
int P_GREEN_INDEX = 2;

// Action Macros
#define GO_STRAIGHT 0
#define DRIFT_LEFT 1
#define DRIFT_RIGHT 2
#define TURN_LEFT 3
#define TURN_RIGHT 4
#define TURN_180 5

int instruction_vectors[6][3] = {{-1, 0, 0}, {-1, -1, 1}, {-1, 1, -1},
                                 {-1, 0, 1}, {-1, 0, -1}, {-1, 0, 2}};

double instruction_probabilities[6] = {.9, .05, .05, .9, .9, .1};

inline int bound_direction(int d) { return (d + 4) % 4; }

// Sensor Probability Macros
#define P_BLUE_G_BLUE .9
#define P_BLUE_G_GREEN .05
#define P_BLUE_G_WHITE .05

#define P_WHITE_G_BLUE .05
#define P_WHITE_G_GREEN .01
#define P_WHITE_G_WHITE .94

#define P_GREEN_G_BLUE .01
#define P_GREEN_G_GREEN .94
#define P_GREEN_G_WHITE .05

#define LEFT 1
#define RIGHT -1

double colourProbMatrix[3][3] = {
    {P_BLUE_G_BLUE, P_BLUE_G_WHITE, P_BLUE_G_GREEN},
    {P_WHITE_G_BLUE, P_WHITE_G_WHITE, P_WHITE_G_GREEN},
    {P_GREEN_G_BLUE, P_GREEN_G_WHITE, P_GREEN_G_GREEN}};

int getProbabilityColourIndex(int colourID) {
  if (colourID == BLUE) return P_BLUE_INDEX;
  if (colourID == WHITE) return P_WHITE_INDEX;
  if (colourID == GREEN) return P_GREEN_INDEX;
  return -1;
}

inline double getColourProbability(int colour_read, int colour_actual) {
  return colourProbMatrix[getProbabilityColourIndex(colour_read)]
                         [getProbabilityColourIndex(colour_actual)];
}

int map[400][4];  // This holds the representation of the map, up to 20x20
                  // intersections, raster ordered, 4 building colours per
                  // intersection.
int sx, sy;       // Size of the map (number of intersections along x and y)
double beliefs[400][4];  // Beliefs for each location and motion direction

inline int check_intersection_in_bounds(int x, int y) {
  return x < sx && y < sy && x >= 0 && y >= 0;
}

double curr_time() {
  // returns time in milliseconds since program started
  double time = 0;

  time = clock();
  time = (double)time / CLOCKS_PER_SEC * 1000;

  return time;
}

int read_colour(int num_samples) {
  /**
  Function does multiple colours reads and returns most frequent COLOR
  **/
  // reading colours
  int col_readings[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < num_samples; i++) {
    int index = BT_read_colour_sensor(PORT_3);
    col_readings[index] += 1;
  }
  // finding maximum
  int max_ind = 0;
  for (int i = 0; i < 8; i++) {
    if (col_readings[i] > col_readings[max_ind]) {
      max_ind = i;
    }
  }
  return max_ind;
}

int get_current_gyro(int setpoint) {
  while (int curr_angle = BT_read_gyro_sensor(GYRO_PORT) != -1) {
    return setpoint - curr_angle;
  }
}

int turn_till_black_gyro(int turnDirection) {
  int pendulum_angle = 15; int curr_angle;
  int colourID; int moving_back;

  int setpoint = BT_read_gyro_sensor(GYRO_PORT);
  for (pendulum_angle; pendulum_angle < 180; pendulum_angle += 15) {
    BT_motor_port_start(MOTOR_D, turnDirection * -MAX_SPEED);
    BT_motor_port_start(MOTOR_A, turnDirection * MAX_SPEED);
    while (curr_angle = abs(get_current_gyro(setpoint)) < pendulum_angle) {
      colourID = read_colour(NUM_SMALL_SAMPLES);
      if (colourID == BLACK || colourID == YELLOW) {
        BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
        return turnDirection;
      }
    }

    turnDirection *= -1; moving_back = false;
    BT_motor_port_start(MOTOR_D, turnDirection * -MAX_SPEED);
    BT_motor_port_start(MOTOR_A, turnDirection * MAX_SPEED);
    
    while (curr_angle = abs(get_current_gyro(setpoint)) < pendulum_angle || !moving_back ) {
      if (curr_angle < pendulum_angle) moving_back = true;
      colourID = read_colour(NUM_SMALL_SAMPLES);
      if (colourID == BLACK || colourID == YELLOW) {
        BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
        return turnDirection;
    }

    turnDirection *= -1;
    BT_motor_port_start(MOTOR_D, turnDirection * -MAX_SPEED);
    BT_motor_port_start(MOTOR_A, turnDirection * MAX_SPEED);
    
    while (curr_angle = abs(get_current_gyro(setpoint)) >= 3) {
      colourID = read_colour(NUM_SMALL_SAMPLES);
      if (colourID == BLACK || colourID == YELLOW) {
        BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
        return turnDirection;
    }
  }
}
  }
}

int turn_till_black(int turnDirection) {
  /***
  Will stay in this function until a road is found
  Implimented a reverse pendulum
  :return: int - last direction checked to find the road
  ***/

  fprintf(stdout, "\ninside turn_till_black()\n");
  float time_stamp, pendulum_duration;
  int returnDirection = turnDirection;
  int colourID;

  time_stamp = curr_time();
  pendulum_duration = 10.0;

  while (1) {
    // motor logic - reverse pendulum
    if ((curr_time() - time_stamp) < (pendulum_duration / 4.0)) {
      BT_motor_port_start(MOTOR_D, turnDirection * -MAX_SPEED);
      BT_motor_port_start(MOTOR_A, turnDirection * MAX_SPEED);
    } else if ((pendulum_duration / 4) <= (curr_time() - time_stamp) &&
               (curr_time() - time_stamp) < (3 * pendulum_duration / 4)) {
      BT_motor_port_start(MOTOR_D, turnDirection * MAX_SPEED);
      BT_motor_port_start(MOTOR_A, turnDirection * -MAX_SPEED);
      returnDirection = turnDirection * -1;
    } else if ((3 * pendulum_duration / 4) <= (curr_time() - time_stamp) &&
               (curr_time() - time_stamp) < pendulum_duration) {
      BT_motor_port_start(MOTOR_D, turnDirection * -MAX_SPEED);
      BT_motor_port_start(MOTOR_A, turnDirection * MAX_SPEED);
    } else {
      time_stamp = curr_time();
      pendulum_duration += 5;
      printf("Pendulum duration: %f", pendulum_duration);
    }
    // break condition - find black or yellow
    colourID = read_colour(NUM_SMALL_SAMPLES);
    if (colourID == BLACK || colourID == YELLOW || colourID == BROWN) {
      printf("\n found black or yellow or brown %d\n", colourID);
      BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
      return returnDirection;
    }
  }
}

void play_tune_for_colour(int colour_id) {
  // return;  // remove later
  if (colour_id == 2) {
    BT_play_tone_sequence(blue_tone);
  } else if (colour_id == 6) {
    BT_play_tone_sequence(white_tone);
  } else {
    BT_play_tone_sequence(green_tone);
  }
}

int main(int argc, char *argv[]) {
  char mapname[1024];
  int dest_x, dest_y, rx, ry;
  unsigned char *map_image;

  memset(&map[0][0], 0, 400 * 4 * sizeof(int));
  sx = 0;
  sy = 0;

  if (argc < 4) {
    fprintf(stderr, "Usage: EV3_Localization Map1.ppm 1 1\n");
    fprintf(stderr,
            "    map_name - should correspond to a properly formatted .ppm map "
            "image\n");
    fprintf(stderr,
            "    dest_x, dest_y - target location for the bot within the map, "
            "-1 -1 calls calibration routine\n");
    exit(1);
  }
  strcpy(&mapname[0], argv[1]);
  dest_x = atoi(argv[2]);
  dest_y = atoi(argv[3]);

  if (dest_x == -1 && dest_y == -1) {
    calibrate_sensor();
    exit(1);
  }

  /******************************************************************************************************************
   * OPTIONAL TO DO: If you added code for sensor calibration, add just below
   * this comment block any code needed to read your calibration data for use in
   * your localization code. Skip this if you are not using calibration
   * ****************************************************************************************************************/

  // Your code for reading any calibration information should not go below this
  // line //

  map_image = readPPMimage(&mapname[0], &rx, &ry);
  if (map_image == NULL) {
    fprintf(stderr, "Unable to open specified map image\n");
    exit(1);
  }

  if (parse_map(map_image, rx, ry) == 0) {
    fprintf(stderr,
            "Unable to parse input image map. Make sure the image is properly "
            "formatted\n");
    free(map_image);
    exit(1);
  }

  if (dest_x < 0 || dest_x >= sx || dest_y < 0 || dest_y >= sy) {
    fprintf(stderr, "Destination location is outside of the map\n");
    free(map_image);
    exit(1);
  }

  // Initialize beliefs - uniform probability for each location and direction
  for (int j = 0; j < sy; j++)
    for (int i = 0; i < sx; i++) {
      beliefs[i + (j * sx)][0] = 1.0 / (double)(sx * sy * 4);
      beliefs[i + (j * sx)][1] = 1.0 / (double)(sx * sy * 4);
      beliefs[i + (j * sx)][2] = 1.0 / (double)(sx * sy * 4);
      beliefs[i + (j * sx)][3] = 1.0 / (double)(sx * sy * 4);
    }

  // Open a socket to the EV3 for remote controlling the bot.
  if (BT_open(HEXKEY) != 0) {
    fprintf(stderr,
            "Unable to open comm socket to the EV3, make sure the EV3 kit is "
            "powered on, and that the\n");
    fprintf(stderr,
            " hex key for the EV3 matches the one in EV3_Localization.h\n");
    free(map_image);
    exit(1);
  }

  fprintf(stderr, "All set, ready to go!\n");

  /*******************************************************************************************************************************
   *
   *  TO DO - Implement the main localization loop, this loop will have the
   *robot explore the map, scanning intersections and updating beliefs in the
   *beliefs array until a single location/direction is determined to be the
   *correct one.
   *
   *          The beliefs array contains one row per intersection (recall that
   *the number of intersections in the map_image is given by sx, sy, and that
   *the map[][] array contains the colour indices of buildings around each
   *intersection. Indexing into the map[][] and beliefs[][] arrays is by raster
   *order, so for an intersection at i,j (with 0<=i<=sx-1 and 0<=j<=sy-1),
   *index=i+(j*sx)
   *
   *          In the beliefs[][] array, you need to keep track of 4 values per
   *intersection, these correspond to the belief the robot is at that specific
   *intersection, moving in one of the 4 possible directions as follows:
   *
   *          beliefs[i][0] <---- belief the robot is at intersection with index
   *i, facing UP beliefs[i][1] <---- belief the robot is at intersection with
   *index i, facing RIGHT beliefs[i][2] <---- belief the robot is at
   *intersection with index i, facing DOWN beliefs[i][3] <---- belief the robot
   *is at intersection with index i, facing LEFT
   *
   *          Initially, all of these beliefs have uniform, beliefs[i][0] <----
   *belief the robot is at intersection with index i, facing UP beliefs[i][1]
   *<---- belief the robot is at intersection with index i, facing RIGHT
   *          beliefs[i][2] <---- belief the robot is at intersection with index
   *i, facing DOWN beliefs[i][3] <---- belief the robot is at intersection with
   *index i, facing LEFT equal probability. Your robot must scan intersections
   *and update belief values based on agreement between what the robot sensed,
   *and the colours in the map.
   *
   *          You have two main tasks these are organized into two major
   *functions:
   *
   *          robot_localization()    <---- Runs the localization loop until the
   *robot's location is found go_to_target()          <---- After localization
   *is achieved, takes the bot to the specified map location
   *
   *          The target location, read from the command line, is left in
   *dest_x, dest_y
   *
   *          Here in main(), you have to call these two functions as
   *appropriate. But keep in mind that it is always possible that even if your
   *bot managed to find its location, it can become lost again while driving to
   *the target location, or it may be the initial localization was wrong and the
   *robot ends up in an unexpected place - a very solid implementation should
   *give your robot the ability to determine it's lost and needs to run
   *localization again.
   *
   *******************************************************************************************************************************/

  // HERE - write code to call robot_localization() and go_to_target() as
  // needed, any additional logic required to get the
  //        robot to complete its task should be here.

  for (int i = 0; i < 50; i++) {
    blue_tone[i][0] = -1;
    blue_tone[i][1] = -1;
    blue_tone[i][2] = -1;
  }
  blue_tone[0][0] = 100;
  blue_tone[0][1] = 1000;
  blue_tone[0][2] = 60;

  // turn_at_intersection(1);
  print_belief(beliefs);
  road_find();
  traverse_road(&intersection_or_boundary_detected);
  BT_motor_port_start(MOTOR_D | MOTOR_A, MAX_SPEED);
  sleep(1);
  BT_motor_port_start(MOTOR_D | MOTOR_A, 0);
  traverse_road(&intersection_or_boundary_detected);
  // traverse_road(&intersection_or_boundary_detected);
  robot_localization(NULL, NULL, NULL);  // TODO: update vars

  road_find();

  // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
  BT_close();
  free(map_image);
  exit(0);
}

// int find_street(void)
// {
//  /*
//   * This function gets your robot onto a street, wherever it is placed on the
//   map. You can do this in many ways, but think
//   * about what is the most effective and reliable way to detect a street and
//   stop your robot once it's on it.
//   *
//   * You can use the return value to indicate success or failure, or to inform
//   the rest of your code of the state of your
//   * bot after calling this function
//   */
//   return(0);
// }

void road_find() {
  // go forward until finds black
  fprintf(stdout, "\ninside road_find()\n");
  int is_lost = 1;
  int colourID;

  while (is_lost) {
    BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED);
    colourID = read_colour(NUM_SAMPLES);
    if (colourID == 1) {  // find black
      BT_motor_port_start(MOTOR_A | MOTOR_D, 0);
      is_lost = 0;
      return;
    }
  }
  fprintf(stdout, "\nLeaving road_find()\n");
}

int traverse_road(int (*exit_conditon_func)(int))
// keeps going forward until no more black
{
  fprintf(stdout, "\n inside traverse_road() \n");
  int colourID;
  int turnDirection = 1;

  while (1) {
    colourID = read_colour(NUM_SAMPLES);
    if (exit_conditon_func(colourID)) {
      // fprintf(stdout, "exit cond true");
      BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);

      return colourID;
    }
    BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED);

    if (colourID != 1 && colourID != 4 &&
        colourID != 7)  // not black and not yellow
    {
      printf("road lost %d\n", colourID);
      // fprintf(stdout, "colour ID: %d\n", colourID);
      BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
      // sleep(1);
      turnDirection = turn_till_black(turnDirection);
    }
  }
}



int scan_corner(int direction, int *colour_pointer) {

  int success = 0;

  //re-calibrate gyro
  int setpoint = BT_read_gyro_sensor(GYRO_PORT);

  //scan for colour
  BT_motor_port_start(ARM_MOTOR, direction * MAX_SPEED);
  while (abs(get_current_gyro(setpoint)) < 91) {
    if (int colourID = read_colour(NUM_SAMPLES) == BLUE || colourID == GREEN || colourID == WHITE) {
      *colour_pointer = colourID;
      success = 1; break;
    }
  }
  BT_motor_port_start(ARM_MOTOR, 0); sleep(1);

  //return to start position
  BT_motor_port_start(ARM_MOTOR, -direction * MAX_SPEED);
  while (int colourID = read_colour(NUM_SMALL_SAMPLES) != BLACK && colourID != YELLOW);
  
  //kill motor and return
  BT_motor_port_start(ARM_MOTOR, 0);
  return success;
}

int scan_corner_drive(int direction, int *colourPointer) {

  int success = 0;

  //re-calibrate gyro
  int setpoint = BT_read_gyro_sensor(GYRO_PORT);

  //scan for colour
  BT_motor_port_start(RIGHT_WHEEL_MOTOR, direction * MAX_SPEED);
  BT_motor_port_start(LEFT_WHEEL_MOTOR, -direction * MAX_SPEED);
  while (1) {
    if (int colourID = read_colour(NUM_SAMPLES) == BLUE || colourID == GREEN || colourID == WHITE) {
      *colour_pointer = colourID;
      success = 1; break;
    }
  }
  BT_motor_port_start(ARM_MOTOR, 0); sleep(1);

  //return to start position
  BT_motor_port_start(RIGHT_WHEEL_MOTOR, -direction * MAX_SPEED);
  BT_motor_port_start(LEFT_WHEEL_MOTOR, direction * MAX_SPEED);
  while (int colourID = read_colour(NUM_SMALL_SAMPLES) != BLACK && colourID != YELLOW);
  
  //kill motor and return
  BT_motor_port_start(ARM_MOTOR, 0);
  return success;
}

}

int scan_intersection_gyro(int *tl, int *tr, int *br, int *bl) {
  /*
   * This function carries out the intersection scan - the bot should
   * (obviously) be placed at an intersection for this, and the specific set of
   * actions will depend on how you designed your bot and its sensor. Whatever
   * the process, you should make sure the intersection scan is reliable - i.e.
   * the positioning of the sensor is reliably over the buildings it needs to
   * read, repeatably, and as the robot moves over the map.
   *
   * Use the APIs sensor reading calls to poll the sensors. You need to remember
   * that sensor readings are noisy and unreliable so * YOU HAVE TO IMPLEMENT
   * SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
   *
   * Recall your lectures on sensor and noise management, and implement a
   * strategy that makes sense. Document your process in the code below so your
   * TA can quickly understand how it works.
   *
   * Once your bot has read the colours at the intersection, it must return them
   * using the provided pointers to 4 integer variables:
   *
   * tl - top left building colour
   * tr - top right building colour
   * br - bottom right building colour
   * bl - bottom left building colour
   *
   * The function's return value can be used to indicate success or failure, or
   * to notify your code of the bot's state after this call.
   */

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

  // Return invalid colour values, and a zero to indicate failure (you will
  // replace this with your code)
  printf("inside scan_intersection  \n");
  *(tl) = -1;
  *(tr) = -1;
  *(br) = -1;
  *(bl) = -1;

  // Move back till black
  BT_motor_port_start(MOTOR_D | MOTOR_A, -MAX_SPEED);
  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);
    // fprintf(stdout, "current_color: %d\n", current_colour);

    // stop when u detect black
    if (current_colour == 1) {
      BT_motor_port_start(MOTOR_D | MOTOR_A,
                          0);  // stopping colour sesnor motor
      break;
    }
  }

  int success = scan_corner(RIGHT, br);
  success = scan_corner(LEFT, bl);

  traverse_road(intersection_or_boundary_detected);
  traverse_road(road_detected);
  BT_motor_port_start(MOTOR_A || MOTOR_D, MAX_SPEED); sleep(1);
  BT_motor_port_stop(MOTOR_A || MOTOR_A, 0);
  traverse_road(road_detected);

  success = scan_corner(LEFT, tl);
  success = scan_corner(RIGHT, tr);
  // while (abs(get_current_gyro(GYRO_PORT)) > 3) {
  //   BT_motor_port_start(GYRO_PORT, )
  // }



  return (1);
}


int scan_intersection(int *tl, int *tr, int *br, int *bl) {
  /*
   * This function carries out the intersection scan - the bot should
   * (obviously) be placed at an intersection for this, and the specific set of
   * actions will depend on how you designed your bot and its sensor. Whatever
   * the process, you should make sure the intersection scan is reliable - i.e.
   * the positioning of the sensor is reliably over the buildings it needs to
   * read, repeatably, and as the robot moves over the map.
   *
   * Use the APIs sensor reading calls to poll the sensors. You need to remember
   * that sensor readings are noisy and unreliable so * YOU HAVE TO IMPLEMENT
   * SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
   *
   * Recall your lectures on sensor and noise management, and implement a
   * strategy that makes sense. Document your process in the code below so your
   * TA can quickly understand how it works.
   *
   * Once your bot has read the colours at the intersection, it must return them
   * using the provided pointers to 4 integer variables:
   *
   * tl - top left building colour
   * tr - top right building colour
   * br - bottom right building colour
   * bl - bottom left building colour
   *
   * The function's return value can be used to indicate success or failure, or
   * to notify your code of the bot's state after this call.
   */

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

  // Return invalid colour values, and a zero to indicate failure (you will
  // replace this with your code)
  printf("inside scan_intersection  \n");
  *(tl) = -1;
  *(tr) = -1;
  *(br) = -1;
  *(bl) = -1;

  /***
   ASSUME robot is at intersection and is not moving
   implicitly assumes that the color sensor is at 90 degrees from the wheel
   axis go back till black
   ***/

  int bottom_left = -1, bottom_right = -1, top_left = -1, top_right = -1;
  int colourCheck[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Move back till black
  BT_motor_port_start(MOTOR_D | MOTOR_A, -MAX_SPEED);
  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);
    // fprintf(stdout, "current_color: %d\n", current_colour);

    // stop when u detect black
    if (current_colour == 1) {
      BT_motor_port_start(MOTOR_D | MOTOR_A,
                          0);  // stopping colour sesnor motor
      break;
    }
  }

  // turn color sesnor motor to the left, until you detect either green, white
  // or blue

  // Move arm left till green, white, blue
  BT_motor_port_start(MOTOR_C, SENSOR_M_SPEED);
  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);
    // fprintf(stdout, "current_color: %d\n", current_colour);

    // stop when u detect one of the colors and put them in some variable
    if (current_colour == 2 || current_colour == 3 || current_colour == 6) {
      BT_motor_port_start(MOTOR_C, 0);  // stopping colour sesnor motor
      // BT_motor_port_start(MOTOR_C, 0);
      bottom_right = current_colour;  // TODO: confrim assumption that
      // play_tune_for_colour(bottom_left);
      break;
    }
  }

  fprintf(stdout, "done first bit\n");

  // keep turning right, until you see black

  BT_motor_port_start(MOTOR_C, -1 * SENSOR_M_SPEED);
  while (1) {
    int current_colour = read_colour(NUM_SMALL_SAMPLES);
    // fprintf(stdout, "current_color: %d\n", current_colour);

    if (current_colour == 1) {
      BT_motor_port_start(MOTOR_C, 0);  // stopping colour sesnor motor
      break;
    }
  }
  fprintf(stdout, "done first.5 bit\n");

  // now keep turning right until you seel green, white or blue
  BT_motor_port_start(MOTOR_C, -1 * SENSOR_M_SPEED);
  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);
    // fprintf(stdout, "current_color: %d\n", current_colour);

    if (current_colour == 2 || current_colour == 3 || current_colour == 6) {
      BT_motor_port_start(MOTOR_C, 0);  // stopping colour sesnor motor
      bottom_left = current_colour;     // TODO: confrim assumption that
      // play_tune_for_colour(bottom_right);
      break;
    }
  }

  // maninder's code

  BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED);  // move forward
  int state = 0;                                      // 0-before_black
                                                      // 1-black
                                                      // 2-after_black
  int colourID = 0;
  while (1) {  // go forward until crosses black
    colourID = read_colour(NUM_SAMPLES);
    // fprintf(stdout, "current_color: %d\n", colourID);

    if (state == 0 && colourID == 1) {  // found road
      state = 1;
    } else if (state == 1 && (colourID == 2 || colourID == 3 ||
                              colourID == 6)) {  // found colour after road
      state = 2;
      top_left = colourID;

      BT_motor_port_start(MOTOR_A | MOTOR_D, 0);
      // play_tune_for_colour(top_right);
      break;
    }
  }

  // turn color sensor left until you see black
  BT_motor_port_start(MOTOR_C, SENSOR_M_SPEED);

  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);

    if (current_colour == 1) {
      BT_motor_port_start(MOTOR_C, 0);  // stopping colour sesnor motor
      break;
    }
  }

  // turn color sensor left until you see green, white or blue
  // assign(colourCheck, 0, 0, 1, 1, 0, 0, 1, 0);  // set colours
  // top_left = move_till_colour(0, 0, SENSOR_M_SPEED, 0, colourCheck);
  // play_tune_for_colour(top_left);

  BT_motor_port_start(MOTOR_C, SENSOR_M_SPEED);

  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);

    if (current_colour == 2 || current_colour == 3 || current_colour == 6) {
      BT_motor_port_start(MOTOR_C, 0);  // stopping colour sesnor motor
      top_right = current_colour;       // TODO: confrim assumption that motor
      // play_tune_for_colour(top_left);
      break;
    }
  }

  BT_motor_port_start(MOTOR_C, -1.0 * SENSOR_M_SPEED);

  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);

    if (current_colour == 1) {
      BT_motor_port_start(MOTOR_C, 0);  // stopping colour sesnor motor
      // TODO: confrim assumption that motor
      // play_tune_for_colour(top_left);
      break;
    }
  }

  // fprintf(stdout, "\n inside get_colours_at_intersection() \n");
  fprintf(stdout, " intersection colours%d %d %d %d \n", top_left, top_right,
          bottom_right, bottom_left);

  // realign color sesnor motor - has to be at 90 degrees again
  // turn color sesnor right until you see black

  *(tl) = top_left;
  *(tr) = top_right;
  *(br) = bottom_right;
  *(bl) = bottom_left;

  return (1);
}

void turn_at_boundary(int direction) {
  printf("inside turnat boundary \n");
  BT_motor_port_start(MOTOR_D | MOTOR_A, -20);
  while (!road_detected(DO_COLOUR_READING)) {
  }
  BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);

  int onCurrentRoad = 1;
  BT_motor_port_start(MOTOR_A, direction * MAX_SPEED);
  BT_motor_port_start(MOTOR_D, direction * -MAX_SPEED);
  while (onCurrentRoad || !(on_road = road_detected(DO_COLOUR_READING))) {
    if (onCurrentRoad && !on_road) {
      onCurrentRoad = 0;
    }
  }
  BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);
}

void turn_at_intersection(int direction) {
  /***
  Turns the robot 90 degrees at a intersection
  ASSUME AT YELLOW AND ROAD FOUND

  direction
            1 turns robot to left
           -1 turns robot to right
  ***/
  printf("turn at intersection \n");
  traverse_road(&road_detected);
  traverse_road(&intersection_or_boundary_detected);

  // reverse till back on road
  BT_motor_port_start(MOTOR_D | MOTOR_A, -20);
  while (!road_detected(DO_COLOUR_READING)) {
  }
  BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);

  int onCurrentRoad = 1;
  BT_motor_port_start(MOTOR_A, direction * MAX_SPEED);
  BT_motor_port_start(MOTOR_D, direction * -MAX_SPEED);
  while (1) {
    int colourID = read_colour(NUM_SMALL_SAMPLES);
    if (!(onCurrentRoad || !road_detected(colourID))) break;
    if (onCurrentRoad && !road_detected(colourID)) {
      onCurrentRoad = 0;
    }
  }
  BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);
}

void normalize_belief(double beliefs[400][4]) {
  double sum = 0;
  for (int intersection_x = 0; intersection_x < sx; intersection_x++) {
    for (int intersection_y = 0; intersection_y < sy; intersection_y++) {
      int index = intersection_x + (intersection_y * sx);
      for (int direction = 0; direction < 4; direction++) {
        sum += beliefs[index][direction];
      }
    }
  }

  printf("sum: %f\n", sum);
  for (int intersection_x = 0; intersection_x < sx; intersection_x++) {
    for (int intersection_y = 0; intersection_y < sy; intersection_y++) {
      int index = intersection_x + (intersection_y * sx);
      for (int direction = 0; direction < 4; direction++) {
        beliefs[index][direction] /= sum;
      }
    }
  }

  sum = 0;
  for (int intersection_x = 0; intersection_x < sx; intersection_x++) {
    for (int intersection_y = 0; intersection_y < sy; intersection_y++) {
      int index = intersection_x + (intersection_y * sx);
      for (int direction = 0; direction < 4; direction++) {
        sum += beliefs[index][direction];
      }
    }
  }
  printf("final sum: %f\n", sum);
}

void print_belief(double beliefs[400][4]) {
  for (int direction = 0; direction < 4; direction++) {
    // printf("\n\ndirection: %d\n", direction);
    for (int intersection_x = 0; intersection_x < sx; intersection_x++) {
      for (int intersection_y = 0; intersection_y < sy; intersection_y++) {
        int index = intersection_x + (intersection_y * sx);
        // printf("belief[int_x=%d][int_y=%d]: %f  \n", intersection_x,
        //        intersection_y, beliefs[index][direction]);
        printf("belief (x,y,dir) %d %d %d  %f  \n", intersection_x,
               intersection_y, direction, beliefs[index][direction]);
      }
      printf("\n");
    }
  }

  fflush(stdout);
}

void find_largest_beliefs() {
  double curr_largest = 0.0;
  // int max_x,max_y,max_dir;
  for (int intersection_y = 0; intersection_y < sy; intersection_y++)
    for (int intersection_x = 0; intersection_x < sx; intersection_x++)
      for (int direction = 0; direction < 4; direction++) {
        int index = intersection_x + (intersection_y * sx);
        if (curr_largest <= beliefs[index][direction]) {
        }
      }
}

void update_sensor_beliefs(int tl, int tr, int br, int bl,
                           double sensor_belief[400][4]) {
  // print_belief(sensor_belief);
  for (int intersection_y = 0; intersection_y < sy; intersection_y++)
    for (int intersection_x = 0; intersection_x < sx; intersection_x++)
      for (int direction = 0; direction < 4; direction++) {
        int index = intersection_x + (intersection_y * sx);

        int m_tl = map[index][(direction + 0) % 4];
        int m_tr = map[index][(direction + 1) % 4];
        int m_br = map[index][(direction + 2) % 4];
        int m_bl = map[index][(direction + 3) % 4];

        // if (intersection_y == 0 && intersection_x == 0 && direction == 0) {
        //   printf("map colours: %d %d %d %d \n", m_tl, m_tr, m_br, m_bl);
        // }

        double p_tl = getColourProbability(tl, m_tl);
        double p_tr = getColourProbability(tr, m_tr);
        double p_bl = getColourProbability(bl, m_bl);
        double p_br = getColourProbability(br, m_br);

        printf("high belief intersection (x,y,direction): (%d, %d, %d)\n",
               intersection_x, intersection_y, direction);
        printf("  p_tl: %f p_tr: %f p_bl: %f p_br: %f\n", p_tl, p_tr, p_bl,
               p_br);

        double p_sensor = p_tl * p_tr * p_bl * p_br;

        // action code

        sensor_belief[index][direction] = p_sensor;
      }

  // normalize_belief(sensor_belief);
  // print_belief(sensor_belief);
}

void modify_absolute_instruction(int absolute_instruction[3], int direction) {
  int temp;
  temp = absolute_instruction[0];

  if (direction == 0) {
    absolute_instruction[0] = absolute_instruction[1];
    absolute_instruction[1] = -temp;
    return;
  }

  if (direction == 1) return;
  if (direction == 2) {
    absolute_instruction[0] = absolute_instruction[1];
    absolute_instruction[1] = temp;
    return;
  }

  if (direction == 3) {
    absolute_instruction[0] = -absolute_instruction[0];
    absolute_instruction[1] = -absolute_instruction[1];
    return;
  }
}

double getActionProbability(int action, int absolute_pos[3]) {
  int absolute_instruction[3] = {instruction_vectors[action][0],
                                 instruction_vectors[action][1],
                                 instruction_vectors[action][2]};

  modify_absolute_instruction(absolute_instruction, absolute_pos[2]);
  int prior_pos[3] = {
      absolute_pos[0] + absolute_instruction[0],
      absolute_pos[1] + absolute_instruction[1],
      bound_direction(absolute_pos[2] + absolute_instruction[2])};

  if (absolute_pos[0] == 0 && absolute_pos[1] == 0) {
    printf("current postiion %d %d %d \n", absolute_pos[0], absolute_pos[1],
           absolute_pos[2]);
    printf("prior postiion %d %d %d \n", prior_pos[0], prior_pos[1],
           prior_pos[2]);
    printf("action %d \n", action);
  }

  if (!check_intersection_in_bounds(prior_pos[0], prior_pos[1])) return 0;

  int index = prior_pos[0] + sx * prior_pos[1];
  return beliefs[index][prior_pos[2]];
}

void update_beliefs(int action, double sensor_belief[400][4]) {
  // copy global beliefs into onld_beliefs

  for (int pos_x = 0; pos_x < sx; pos_x++)
    for (int pos_y = 0; pos_y < sy; pos_y++) {
      int index = pos_y * sx + pos_x;
      for (int direction = 0; direction < 4; direction++) {
        int absolute_pos[3] = {pos_x, pos_y, direction};
        if (action == GO_STRAIGHT) {
          sensor_belief[index][direction] *=
              instruction_probabilities[GO_STRAIGHT] *
                  getActionProbability(GO_STRAIGHT, absolute_pos) +
              instruction_probabilities[DRIFT_LEFT] *
                  getActionProbability(DRIFT_LEFT, absolute_pos) +
              instruction_probabilities[DRIFT_RIGHT] *
                  getActionProbability(DRIFT_RIGHT, absolute_pos);
        } else if (action == TURN_LEFT) {
          sensor_belief[index][direction] *=
              instruction_probabilities[TURN_LEFT] *
                  getActionProbability(TURN_LEFT, absolute_pos) +
              instruction_probabilities[TURN_180] *
                  getActionProbability(TURN_180, absolute_pos);
        } else if (action == TURN_RIGHT) {
          sensor_belief[index][direction] *=
              instruction_probabilities[TURN_RIGHT] *
                  getActionProbability(TURN_RIGHT, absolute_pos) +
              instruction_probabilities[TURN_180] *
                  getActionProbability(TURN_180, absolute_pos);
        }
      }
    }

  // copying new beliefs into old beliefs
  for (int pos_x = 0; pos_x < sx; pos_x++)
    for (int pos_y = 0; pos_y < sy; pos_y++) {
      int index = pos_y * sx + pos_x;
      for (int direction = 0; direction < 4; direction++) {
        beliefs[index][direction] = sensor_belief[index][direction];
      }
    }

  normalize_belief(beliefs);
  print_belief(beliefs);
}

int robot_localization(int *robot_x, int *robot_y, int *direction) {
  /*  This function implements the main robot localization process. You have to
   * write all code that will control the robot and get it to carry out the
   * actions required localcess:
   *
   *  - Find the street, and drive along the street toward an intersection
   *  - Scan the colours of buildings around the intersection
   *  - Update the beliefs in the beliefs[][] array according to the sensor
   * measurements and the map data
   *  - Repeat the process until a single intersection/facing direction is
   * distintly more likely than all the rest
   *
   *  * We have provided headers for the following functions:
   *
   *  find_street()
   *  drive_along_street()
   *  scan_intersection()
   *  turn_at_intersection()
   *scan_intersection
   *  Note that *your bot must explore* the map to achieve reliable
   * localization, this means your intersection scanning strategy should not
   * rely exclusively on moving forward, but should include turning and
   * exploring other streets than the one your bot was initially placed on.
   *
   *  For each of the control functions, however, you will need to use the EV3
   * API, so be sure to become familiar with it.
   *
   *  In terms of sensor management - the API allows you to read colours either
   * as indexed values or RGB, it's up to you which one to use, and how to
   * interpret the noisy, unreliable data you're likely to get from the sensor
   *  in order to update beliefs.
   *
   *  HOWEVER: *** YOU must document clearly both in comments within this
   * function, and in your report, how the sensor is used to read colour data,
   * and how the beliefs are updated based on the sensor readings.
   *
   *  DO NOT FORGET - Beliefs should always remain normalized to be a
   * probability distribution, that means the sum of beliefs over all
   * intersections and facing directions must be 1 at all times.
   *
   *  The function receives as input pointers to three integer values, these
   * will be used to store the estimated robot's location and facing direction.
   * The direction is specified as: 0 - UP 1 - RIGHT 2 - BOTTOM 3 - LEFT
   *
   *  The function's return value is 1 if localization was successful, and 0
   * otherwise.
   */

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

  // Return an invalid location/direction and notify that localization was
  // unsuccessful (you will delete this and replace it with your code).
  // Initialize beliefs - uniform probability for each location and direction
  // for (int j = 0; j < sy; j++)
  //   for (int i = 0; i < sx; i++) {
  //     beliefs[i + (j * sx)][0] = 1.0 / (double)(sx * sy * 4);
  //     beliefs[i + (j * sx)][1] = 1.0 / (double)(sx * sy * 4);
  //     beliefs[i + (j * sx)][2] = 1.0 / (double)(sx * sy * 4);
  //     beliefs[i + (j * sx)][3] = 1.0 / (double)(sx * sy * 4);
  //   }
  // *          The beliefs array contains one row per intersection (recall that
  //  *the number of intersections in the map_image is given by sx, sy, and that
  //  *the map[][] array contains the colour indices of buildings around each
  //  *intersection. Indexing into the map[][] and beliefs[][] arrays is by
  //  raster *order, so for an intersection at i,j (with 0<=i<=sx-1 and
  //  0<=j<=sy-1), *index=i+(j*sx)
  // *(robot_x) = -1;
  // *(robot_y) = -1;
  // *(direction) = -1;

  double sensor_belief[400][4];
  // double action_belief[400][4];

  for (int i = 0; i < 400; i++) {
    for (int j = 0; j < 4; j++) {
      sensor_belief[i][j] = 0;
    }
  }

  // int prior_action = GO_STRAIGHT;

  while (1) {
    int colourID = traverse_road(&intersection_or_boundary_detected);
    // if intercetion then scan

    if (colourID == YELLOW) {
      int tl, tr, bl, br;
      scan_intersection(&tl, &tr, &br, &bl);
      update_sensor_beliefs(tl, tr, br, bl, sensor_belief);
      update_beliefs(GO_STRAIGHT, sensor_belief);
    } else {
      // TODO update action here as well
      // ensure to check for 180 deg turn, if you do two consequtive turns
      // without going fwd
      turn_at_boundary(LEFT);
      // if (prior_action == GO_STRAIGHT) {
      //   prior_action = TURN_RIGHT;
      // }
    }
  }

  return (0);
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x,
                 int target_y) {
  /*
   * This function is called once localization has been successful, it performs
   * the actions required to take the robot from its current location to the
   * specified target location.
   *
   * You have to write the code required to carry out this task - once again,
   * you can use the function headers provided, or write your own code to
   * control the bot, but document your process carefully in the comments below
   * so your TA can easily understand how everything works.
   *
   * Your code should be able to determine if the robot has gotten lost (or if
   * localization was incorrect), and your bot should be able to recover.
   *
   * Inputs - The robot's current location x,y (the intersection coordinates,
   * not image pixel coordinates) The target's intersection location
   *
   * Return values: 1 if successful (the bot reached its target destination), 0
   * otherwise
   */

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  return (0);
}

void calibrate_sensor(void) {
  /*
   * This function is called when the program is started with -1  -1 for the
   * target location.
   *
   * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended
   * as good calibration will make sensor readings more reliable and will make
   * your code more resistent to changes in illumination, map quality, or
   * battery level.
   *
   * The principle is - Your code should allow you to sample the different
   * colours in the map, and store representative values that will help you
   * figure out what colours the sensor is reading given the current conditions.
   *
   * Inputs - None
   * Return values - None - your code has to save the calibration information to
   * a file, for later use (see in main())
   *
   * How to do this part is up to you, but feel free to talk with your TA and
   * instructor about it!
   */

  /************************************************************************************************************************
   *   OIPTIONAL TO DO  -   Complete this function
   ***********************************************************************************************************************/
  fprintf(stderr, "Calibration function called!\n");
}

int parse_map(unsigned char *map_img, int rx, int ry) {
  /*
    This function takes an input image map array, and two integers that specify
    the image size. It attempts to parse this image into a representation of the
    map in the image. The size and resolution of the map image should not affect
    the parsing (i.e. you can make your own maps without worrying about the
    exact position of intersections, roads, buildings, etc.).

    However, this function requires:

    * White background for the image  [255 255 255]
    * Red borders around the map  [255 0 0]
    * Black roads  [0 0 0]
    * Yellow intersections  [255 255 0]
    * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white
    [255 255 255] (any other colour values are ignored - so you can add markings
    if you like, those will not affect parsing)

    The image must be a properly formated .ppm image, see readPPMimage below for
    details of the format. The GIMP image editor saves properly formatted .ppm
    images, as does the imagemagick image processing suite.

    The map representation is read into the map array, with each row in the
    array corrsponding to one intersection, in raster order, that is, for a map
    with k intersections along its width:

     (row index for the intersection)

     0     1     2    3 ......   k-1

     k    k+1   k+2  ........

     Each row will then contain the colour values for buildings around the
    intersection clockwise from top-left, that is


     top-left               top-right

             intersection

     bottom-left           bottom-right

     So, for the first intersection (at row 0 in the map array)
     map[0][0] <---- colour for the top-left building
     map[0][1] <---- colour for the top-right building
     map[0][2] <---- colour for the bottom-right building
     map[0][3] <---- colour for the bottom-left building

     Color values for map locations are defined as follows (this agrees with
    what the EV3 sensor returns in indexed-colour-reading mode):

     1 -  Black
     2 -  Blue
     3 -  Green
     4 -  Yellow
     5 -  Red
     6 -  White

     If you find a 0, that means you're trying to access an intersection that is
    not on the map! Also note that in practice, because of how the map is
    defined, you should find only Green, Blue, or White around a given
    intersection.

     The map size (the number of intersections along the horizontal and vertical
    directions) is updated and left in the global variables sx and sy.

     Feel free to create your own maps for testing (you'll have to print them to
    a reasonable size to use with your bot).

  */

  int last3[3];
  int x, y;
  unsigned char R, G, B;
  int ix, iy;
  int bx, by, dx, dy, wx, wy;  // Intersection geometry parameters
  int tgl;
  int idx;

  ix = iy = 0;  // Index to identify the current intersection

  // Determine the spacing and size of intersections in the map
  tgl = 0;
  for (int i = 0; i < rx; i++) {
    for (int j = 0; j < ry; j++) {
      R = *(map_img + ((i + (j * rx)) * 3));
      G = *(map_img + ((i + (j * rx)) * 3) + 1);
      B = *(map_img + ((i + (j * rx)) * 3) + 2);
      if (R == 255 && G == 255 && B == 0) {
        // First intersection, top-left pixel. Scan right to find width and
        // spacing
        bx = i;  // Anchor for intersection locations
        by = j;
        for (int k = i; k < rx;
             k++)  // Find width and horizontal distance to next intersection
        {
          R = *(map_img + ((k + (by * rx)) * 3));
          G = *(map_img + ((k + (by * rx)) * 3) + 1);
          B = *(map_img + ((k + (by * rx)) * 3) + 2);
          if (tgl == 0 && (R != 255 || G != 255 || B != 0)) {
            tgl = 1;
            wx = k - i;
          }
          if (tgl == 1 && R == 255 && G == 255 && B == 0) {
            tgl = 2;
            dx = k - i;
          }
        }
        for (int k = j; k < ry;
             k++)  // Find height and vertical distance to next intersection
        {
          R = *(map_img + ((bx + (k * rx)) * 3));
          G = *(map_img + ((bx + (k * rx)) * 3) + 1);
          B = *(map_img + ((bx + (k * rx)) * 3) + 2);
          if (tgl == 2 && (R != 255 || G != 255 || B != 0)) {
            tgl = 3;
            wy = k - j;
          }
          if (tgl == 3 && R == 255 && G == 255 && B == 0) {
            tgl = 4;
            dy = k - j;
          }
        }

        if (tgl != 4) {
          fprintf(stderr, "Unable to determine intersection geometry!\n");
          return (0);
        } else
          break;
      }
    }
    if (tgl == 4) break;
  }
  fprintf(stderr,
          "Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, "
          "horiz_distance=%d, vertical_distance=%d\n",
          bx, by, wx, wy, dx, dy);

  sx = 0;
  for (int i = bx + (wx / 2); i < rx; i += dx) {
    R = *(map_img + ((i + (by * rx)) * 3));
    G = *(map_img + ((i + (by * rx)) * 3) + 1);
    B = *(map_img + ((i + (by * rx)) * 3) + 2);
    if (R == 255 && G == 255 && B == 0) sx++;
  }

  sy = 0;
  for (int j = by + (wy / 2); j < ry; j += dy) {
    R = *(map_img + ((bx + (j * rx)) * 3));
    G = *(map_img + ((bx + (j * rx)) * 3) + 1);
    B = *(map_img + ((bx + (j * rx)) * 3) + 2);
    if (R == 255 && G == 255 && B == 0) sy++;
  }

  fprintf(stderr,
          "Map size: Number of horizontal intersections=%d, number of vertical "
          "intersections=%d\n",
          sx, sy);

  // Scan for building colours around each intersection
  idx = 0;
  for (int j = 0; j < sy; j++)
    for (int i = 0; i < sx; i++) {
      x = bx + (i * dx) + (wx / 2);
      y = by + (j * dy) + (wy / 2);

      fprintf(stderr, "Intersection location: %d, %d\n", x, y);
      // Top-left
      x -= wx;
      y -= wy;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][0] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][0] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][0] = 6;
      else
        fprintf(stderr,
                "Colour is not valid for intersection %d,%d, Top-Left "
                "RGB=%d,%d,%d\n",
                i, j, R, G, B);

      // Top-right
      x += 2 * wx;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][1] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][1] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][1] = 6;
      else
        fprintf(stderr,
                "Colour is not valid for intersection %d,%d, Top-Right "
                "RGB=%d,%d,%d\n",
                i, j, R, G, B);

      // Bottom-right
      y += 2 * wy;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][2] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][2] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][2] = 6;
      else
        fprintf(stderr,
                "Colour is not valid for intersection %d,%d, Bottom-Right "
                "RGB=%d,%d,%d\n",
                i, j, R, G, B);

      // Bottom-left
      x -= 2 * wx;
      R = *(map_img + ((x + (y * rx)) * 3));
      G = *(map_img + ((x + (y * rx)) * 3) + 1);
      B = *(map_img + ((x + (y * rx)) * 3) + 2);
      if (R == 0 && G == 255 && B == 0)
        map[idx][3] = 3;
      else if (R == 0 && G == 0 && B == 255)
        map[idx][3] = 2;
      else if (R == 255 && G == 255 && B == 255)
        map[idx][3] = 6;
      else
        fprintf(stderr,
                "Colour is not valid for intersection %d,%d, Bottom-Left "
                "RGB=%d,%d,%d\n",
                i, j, R, G, B);

      fprintf(stderr, "Colours for this intersection: %d, %d, %d, %d\n",
              map[idx][0], map[idx][1], map[idx][2], map[idx][3]);

      idx++;
    }

  return (1);
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry) {
  // Reads an image from a .ppm file. A .ppm file is a very simple image
  // representation format with a text header followed by the binary RGB data at
  // 24bits per pixel. The header has the following form:
  //
  // P6
  // # One or more comment lines preceded by '#'
  // 340 200
  // 255
  //
  // The first line 'P6' is the .ppm format identifier, this is followed by one
  // or more lines with comments, typically used to inidicate which program
  // generated the .ppm file. After the comments, a line with two integer values
  // specifies the image resolution as number of pixels in x and number of
  // pixels in y. The final line of the header stores the maximum value for
  // pixels in the image, usually 255. After this last header line, binary data
  // stores the RGB values for each pixel in row-major order. Each pixel
  // requires 3 bytes ordered R, G, and B.
  //
  // NOTE: Windows file handling is rather crotchetty. You may have to change
  // the
  //       way this file is accessed if the images are being corrupted on read
  //       on Windows.
  //

  FILE *f;
  unsigned char *im;
  char line[1024];
  int i;
  unsigned char *tmp;
  double *fRGB;

  im = NULL;
  f = fopen(filename, "rb+");
  if (f == NULL) {
    fprintf(stderr,
            "Unable to open file %s for reading, please check name and path\n",
            filename);
    return (NULL);
  }
  fgets(&line[0], 1000, f);
  if (strcmp(&line[0], "P6\n") != 0) {
    fprintf(stderr,
            "Wrong file format, not a .ppm file or header end-of-line "
            "characters missing\n");
    fclose(f);
    return (NULL);
  }
  fprintf(stderr, "%s\n", line);
  // Skip over comments
  fgets(&line[0], 511, f);
  while (line[0] == '#') {
    fprintf(stderr, "%s", line);
    fgets(&line[0], 511, f);
  }
  sscanf(&line[0], "%d %d\n", rx, ry);  // Read image size
  fprintf(stderr, "nx=%d, ny=%d\n\n", *rx, *ry);

  fgets(&line[0], 9, f);  // Read the remaining header line
  fprintf(stderr, "%s\n", line);
  im = (unsigned char *)calloc((*rx) * (*ry) * 3, sizeof(unsigned char));
  if (im == NULL) {
    fprintf(stderr, "Out of memory allocating space for image\n");
    fclose(f);
    return (NULL);
  }
  fread(im, (*rx) * (*ry) * 3 * sizeof(unsigned char), 1, f);
  fclose(f);

  return (im);
}