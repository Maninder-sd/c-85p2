// Initial testing of a bare-bones BlueTooth communication
// library for the EV3 - Thank you Lego for changing everything
// from the NXT to the EV3!

#include "btcomm.h"
#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <math.h>
#include <time.h>

#define MAX_SPEED 10
#define SENSOR_M_SPEED 5
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

void road_find();
void traverse_road(int (*exit_conditon_func)(int));
int turn_till_black(int);
double curr_time();
void turn_at_intersection(int);
void led_test();
int read_colour(int);
void get_colours_at_intersection();
void get_colours_at_intersection_2();

int blue_tone[50][3];
int white_tone[50][3];
int green_tone[50][3];

int intersection_or_boundary_detected(int colourID) {
  if (colourID == DO_COLOUR_READING) {
    colourID = read_colour(NUM_SAMPLES);
  }
  printf("int or bound : %d\n", colourID);
  return (colourID == 5);  // change later
}

void traverse_road(int (*exit_conditon_func)(int))
// keeps going forward until no more black
{
  fprintf(stdout, "\n inside traverse_road() \n");
  int colourID;
  int turnDirection = 1;

  while (1) {
    fprintf(stdout, "traverse before read\n");

    fprintf(stdout, "traverse after read\n");
    colourID = read_colour(NUM_SAMPLES);
    if (exit_conditon_func(colourID)) {
      fprintf(stdout, "exit cond true");
      BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);

      return;
    }
    BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED);

    if (colourID != 1 && colourID != 4 &&
        colourID != 7)  // not black and not yellow
    {
      fprintf(stdout, "colour ID: %d\n", colourID);
      BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
      // sleep(1);
      turnDirection = turn_till_black(turnDirection);
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

double curr_time() {
  // returns time in milliseconds since program started
  double time = 0;

  time = clock();
  time = (double)time / CLOCKS_PER_SEC * 1000;

  return time;
}

int main(int argc, char *argv[]) {
  char test_msg[8] = {0x06, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x01};
  char reply[1024];
  int tone_data[50][3];

  for (int i = 0; i < 50; i++) {
    tone_data[i][0] = -1;
    tone_data[i][1] = -1;
    tone_data[i][2] = -1;
  }

  for (int i = 0; i < 50; i++) {
    blue_tone[i][0] = -1;
    blue_tone[i][1] = -1;
    blue_tone[i][2] = -1;
  }

  for (int i = 0; i < 50; i++) {
    green_tone[i][0] = -1;
    green_tone[i][1] = -1;
    green_tone[i][2] = -1;
  }

  for (int i = 0; i < 50; i++) {
    white_tone[i][0] = -1;
    white_tone[i][1] = -1;
    white_tone[i][2] = -1;
  }

  blue_tone[0][0] = 100;
  blue_tone[0][1] = 1000;
  blue_tone[0][2] = 60;

  green_tone[0][0] = 700;
  green_tone[0][1] = 1000;
  green_tone[0][2] = 60;

  white_tone[0][0] = 1500;
  white_tone[0][1] = 1000;
  white_tone[0][2] = 60;

  // Reset tone data information

  tone_data[0][0] = 262;
  tone_data[0][1] = 250;
  tone_data[0][2] = 60;
  tone_data[1][0] = 330;
  tone_data[1][1] = 250;
  tone_data[1][2] = 60;
  tone_data[2][0] = 392;
  tone_data[2][1] = 250;
  tone_data[2][2] = 60;
  tone_data[3][0] = 523;
  tone_data[3][1] = 250;
  tone_data[3][2] = 60;

  memset(&reply[0], 0, 1024);

// just uncomment your bot's hex key to compile for your bot, and comment the
// other ones out.
#ifndef HEXKEY
#define HEXKEY "00:16:53:55:D9:FC"  // <--- SET UP YOUR EV3's HEX ID here
#endif

  BT_open(HEXKEY);

  // name must not contain spaces or special characters
  // max name length is 12 characters
  BT_setEV3name("R2D2");

  // BT_play_tone_sequence(tone_data);
  // BT_play_tone_sequence(blue_tone);
  // BT_play_tone_sequence(green_tone);
  // BT_play_tone_sequence(white_tone);

  int TURN_DIRECTION = -1;

  // get_colours_at_intersection();
  // get_colours_at_intersection_2();
  // turn_at_intersection(TURN_DIRECTION);
  // road_find();
  // traverse_road(&intersection_or_boundary_detected);
  // BT_motor_port_start(MOTOR_D|MOTOR_A, MAX_SPEED);
  led_test();
  printf("done");

  BT_close();
  fprintf(stderr, "Done!\n");
}

void led_test() {
  int RGB[3];
  int colourID = 0;
  while (1) {
    // BT_read_colour_sensor_RGB( PORT_3 ,RGB);
    // RGB[0] *=0.25;
    // RGB[1] *=0.25;
    // RGB[2] *=0.25;
    // printf("\n (R,G,B) is (%d,%d,%d)\n", RGB[0], RGB[1], RGB[2]);
    colourID = read_colour(NUM_SAMPLES);
    printf("\n %d \n", colourID);

    // sleep(1);
  }
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

int road_detected(int colourID) {
  if (colourID == DO_COLOUR_READING) {
    return (read_colour(NUM_SAMPLES) == 1);
  }
  return (colourID == 1);
}

void printColour(int colourID) {
  fprintf(stdout, "Current colour reading: %d\n", colourID);
}

int move_till_colour(int motor_A, int motor_B, int motor_C, int motor_D,
                     int *colourCheck) {
  /***
   * Move motors until sees any colour in array colourID
   * len(colourID) = 8 for each colour
   * colourID[i] = 1 means this colour is part of exit condition
   * returns the colourID of the last reading
   ***/
  int current_colour;
  // start motors
  BT_motor_port_start(MOTOR_A, motor_A);
  BT_motor_port_start(MOTOR_B, motor_B);
  BT_motor_port_start(MOTOR_C, motor_C);
  BT_motor_port_start(MOTOR_D, motor_D);

  // fprintf(stdout, "moving until color: %d\n", colourCheck);
  while (1) {
    current_colour = read_colour(NUM_SAMPLES);
    // stop when detect colourCheck
    for (int i = 0; i < 8; i++) {
      if (colourCheck[i] == 1 && current_colour == colourCheck[i]) {
        BT_motor_port_start(MOTOR_A | MOTOR_B | MOTOR_C | MOTOR_D, 0);
        return current_colour;
      }
    }
  }
}

void assign(int *colourCheck, int v0, int v1, int v2, int v3, int v4, int v5,
            int v6, int v7) {
  // Value Colour
  //  0    No colour read
  //  1    Black
  //  2    Blue
  //  3    Green
  //  4    Yellow
  //  5    Red
  //  6    White
  //  7    Brown
  colourCheck[0] = v0;
  colourCheck[1] = v1;
  colourCheck[2] = v2;
  colourCheck[3] = v3;
  colourCheck[4] = v4;
  colourCheck[5] = v5;
  colourCheck[6] = v6;
  colourCheck[7] = v7;
}

void get_colours_at_intersection() {
  /***
  ASSUME robot is at intersection and is not moving
  implicitly assumes that the color sensor is at 90 degrees from the wheel
  axis go back till black
  ***/

  int bottom_left = -1, bottom_right = -1, top_left = -1, top_right = -1;
  int colourCheck[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Move back till black
  printf("moving to black\n");
  assign(colourCheck, 0, 1, 0, 0, 0, 0, 0, 0);  // set black
  move_till_colour(-MAX_SPEED, 0, 0, -MAX_SPEED, colourCheck);

  // Move arm left till green, white, blue
  printf("moving arm left till green, white, blue\n");
  assign(colourCheck, 0, 0, 1, 1, 0, 0, 1, 0);  // set colours
  bottom_left = move_till_colour(0, 0, SENSOR_M_SPEED, 0, colourCheck);
  play_tune_for_colour(bottom_left);

  // move arm right, until you see black
  printf("move arm right, until you see black");
  assign(colourCheck, 0, 1, 0, 0, 0, 0, 0, 0);  // set black
  move_till_colour(0, 0, -1 * SENSOR_M_SPEED, 0, colourCheck);

  // now keep turning right until you seel green, white or blue
  assign(colourCheck, 0, 0, 1, 1, 0, 0, 1, 0);  // set colours
  bottom_right = move_till_colour(0, 0, -1 * SENSOR_M_SPEED, 0, colourCheck);
  play_tune_for_colour(bottom_right);

  // maninder's code

  // Move forward till black
  assign(colourCheck, 0, 1, 0, 0, 0, 0, 0, 0);  // set black
  move_till_colour(MAX_SPEED, 0, 0, MAX_SPEED, colourCheck);

  // now keep Move forward until you seel green, white or blue
  assign(colourCheck, 0, 1, 0, 0, 0, 0, 0, 0);  // set black
  top_right = move_till_colour(MAX_SPEED, 0, 0, MAX_SPEED, colourCheck);
  play_tune_for_colour(top_right);

  // turn color sensor left until you see black
  assign(colourCheck, 0, 1, 0, 0, 0, 0, 0, 0);  // set black
  move_till_colour(0, 0, SENSOR_M_SPEED, 0, colourCheck);

  // turn color sensor left until you see green, white or blue
  assign(colourCheck, 0, 0, 1, 1, 0, 0, 1, 0);  // set colours
  top_left = move_till_colour(0, 0, SENSOR_M_SPEED, 0, colourCheck);
  play_tune_for_colour(top_left);

  fprintf(stdout, "\n inside get_colours_at_intersection() \n");
  fprintf(stdout, "%d %d %d %d \n", bottom_left, bottom_right, top_right,
          top_left);

  // realign color sesnor motor - has to be at 90 degrees again
  // turn color sesnor right until you see black
}

void get_colours_at_intersection_2() {
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
    fprintf(stdout, "current_color: %d\n", current_colour);

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
    fprintf(stdout, "current_color: %d\n", current_colour);

    // stop when u detect one of the colors and put them in some variable
    if (current_colour == 2 || current_colour == 3 || current_colour == 6) {
      BT_motor_port_start(MOTOR_C, 0);  // stopping colour sesnor motor
      bottom_left = current_colour;     // TODO: confrim assumption that
      play_tune_for_colour(bottom_left);
      break;
    }
  }

  fprintf(stdout, "done first bit\n");

  // keep turning right, until you see black

  BT_motor_port_start(MOTOR_C, -1 * SENSOR_M_SPEED);
  while (1) {
    int current_colour = read_colour(NUM_SMALL_SAMPLES);
    fprintf(stdout, "current_color: %d\n", current_colour);

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
    fprintf(stdout, "current_color: %d\n", current_colour);

    if (current_colour == 2 || current_colour == 3 || current_colour == 6) {
      BT_motor_port_start(MOTOR_C, 0);  // stopping colour sesnor motor
      bottom_right = current_colour;    // TODO: confrim assumption that
      play_tune_for_colour(bottom_right);
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
    fprintf(stdout, "current_color: %d\n", colourID);

    if (state == 0 && colourID == 1) {  // found road
      state = 1;
    } else if (state == 1 && (colourID == 2 || colourID == 3 ||
                              colourID == 6)) {  // found colour after road
      state = 2;
      top_right = colourID;

      BT_motor_port_start(MOTOR_A | MOTOR_D, 0);
      play_tune_for_colour(top_right);
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
      top_left = current_colour;        // TODO: confrim assumption that motor
      play_tune_for_colour(top_left);
      break;
    }
  }

  fprintf(stdout, "\n inside get_colours_at_intersection() \n");
  fprintf(stdout, "%d %d %d %d \n", bottom_left, bottom_right, top_right,
          top_left);

  // realign color sesnor motor - has to be at 90 degrees again
  // turn color sesnor right until you see black
}

void turn_at_intersection(int direction) {
  /***
  Turns the robot 90 degrees at a intersection
  ASSUME AT YELLOW AND ROAD FOUND

  direction
            1 turns robot to left
           -1 turns robot to right
  ***/
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
  while (onCurrentRoad || !road_detected(DO_COLOUR_READING)) {
    if (onCurrentRoad && !road_detected(DO_COLOUR_READING)) {
      onCurrentRoad = 0;
    }
  }
  BT_motor_port_stop(MOTOR_D | MOTOR_A, 0);
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
      BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
      return returnDirection;
    }
  }
}