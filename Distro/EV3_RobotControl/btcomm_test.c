// Initial testing of a bare-bones BlueTooth communication
// library for the EV3 - Thank you Lego for changing everything
// from the NXT to the EV3!

#include "btcomm.h"
#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <time.h>
#include <math.h>

#define MAX_SPEED 3
#define NUM_SAMPLES 10
#define NUM_SMALL_SAMPLES 3
#define DO_COLOUR_READING -1

void road_find();
void traverse_road(int (*exit_conditon_func)(int));
int turn_till_black(int);
long curr_time();
void turn_at_intersection(int);
void led_test();
int read_colour(int);
void get_colours_at_intersection();

long curr_time()
{
  // returns time in milliseconds since program started
  clock_t time = 0;

  time = clock();
  time = (double)time / CLOCKS_PER_SEC * 1000;

  return time;
}

int main(int argc, char *argv[])
{
  char test_msg[8] = {0x06, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x01};
  char reply[1024];
  int tone_data[50][3];

  // Reset tone data information
  for (int i = 0; i < 50; i++)
  {
    tone_data[i][0] = -1;
    tone_data[i][1] = -1;
    tone_data[i][2] = -1;
  }

  tone_data[0][0] = 262;
  tone_data[0][1] = 250;
  tone_data[0][2] = 1;
  tone_data[1][0] = 330;
  tone_data[1][1] = 250;
  tone_data[1][2] = 25;
  tone_data[2][0] = 392;
  tone_data[2][1] = 250;
  tone_data[2][2] = 50;
  tone_data[3][0] = 523;
  tone_data[3][1] = 250;
  tone_data[3][2] = 63;

  memset(&reply[0], 0, 1024);

// just uncomment your bot's hex key to compile for your bot, and comment the other ones out.
#ifndef HEXKEY
#define HEXKEY "00:16:53:55:D9:FC" // <--- SET UP YOUR EV3's HEX ID here
#endif

  BT_open(HEXKEY);

  // name must not contain spaces or special characters
  // max name length is 12 characters
  BT_setEV3name("R2D2");

  //  BT_play_tone_sequence(tone_data);

  // int success = 0;
  // printf("\n before on \n");
  // // BT_timed_motor_port_start(MOTOR_A, 20, 0, 15000, 0);
  // BT_motor_port_start(MOTOR_D, 25);
  // printf("\n after on \n");
  // sleep(2);
  // // BT_timed_motor_port_start(MOTOR_A, 0, 0, 100, 0);
  // BT_motor_port_start(MOTOR_D, 0);
  // printf("\n after off \n");
  // // BT_motor_port_start(MOTOR_D, -25);

  // road_find();
  // sleep(1);
  // traverse_road();

  int TURN_DIRECTION = -1;
    // BT_motor_port_start(MOTOR_C, -1* MAX_SPEED);
// int vel = 0;
//   while(1) {
//     scanf("%d\n", &vel);
//     BT_motor_port_start(MOTOR_C, vel);
//   }
  get_colours_at_intersection();
  // turn_at_intersection(TURN_DIRECTION);
  // BT_motor_port_start(MOTOR_D|MOTOR_A, MAX_SPEED);
  // led_test();
  printf("done");

  BT_close();
  fprintf(stderr, "Done!\n");
}

void led_test()
{
  int RGB[3];
  int col_ind = 0;
  while (1)
  {
    // BT_read_colour_sensor_RGB( PORT_3 ,RGB);
    // RGB[0] *=0.25;
    // RGB[1] *=0.25;
    // RGB[2] *=0.25;
    // printf("\n (R,G,B) is (%d,%d,%d)\n", RGB[0], RGB[1], RGB[2]);
    col_ind = read_colour(NUM_SAMPLES);
    printf("\n %d \n", col_ind);

    // sleep(1);
  }
}

int read_colour(int num_samples)
{
  // function does multiple reads and returns most frequent COLOR
  int col_readings[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < num_samples; i++)
  {
    int index = BT_read_colour_sensor(PORT_3);
    col_readings[index] += 1;
  }

  int max_ind = 0;
  for (int i = 0; i < 8; i++)
  {
    if (col_readings[i] > col_readings[max_ind])
    {
      max_ind = i;
    }
  }

  return max_ind;
}

void road_find()
{
  // go forward until finds black
  // calls traverse_road()
  fprintf(stdout, "\ninside road_find()\n");
  int is_lost = 1;
  int col_ind;

  while (is_lost)
  {
    BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED);
    col_ind = read_colour(NUM_SAMPLES);
    if (col_ind == 1)
    { // find black
      BT_motor_port_start(MOTOR_A | MOTOR_D, 0);
      is_lost = 0;
      return;
    }
  }
}

int road_detected(int colourID)
{
  if (colourID == DO_COLOUR_READING)
  {
    return (read_colour(NUM_SAMPLES) == 1);
  }
  return (colourID == 1);
}

int intersection_or_boundary_detected(int colourID)
{
  if (colourID == DO_COLOUR_READING)
  {
    colourID = read_colour(NUM_SAMPLES);
  }
  printf("int or bound : %d\n", colourID);
  return (colourID == 4 || colourID == 5);
}

void printColour(int colourID)
{
  fprintf(stdout, "Current colour reading: %d\n", colourID);
}

#define INTERSECTION_SAMPLES 100 // play around with this value

void get_colours_at_intersection() {
  // ASSUME robot is at intersection and is not moving
  // implicitly assumes that the color sensor is at 90 degrees from the wheel axis

  int bottom_left = -1, bottom_right = -1, top_left = -1, top_right = -1;
  // turn color sesnor motor to the left, until you detect either green, white or blue
  BT_motor_port_start(MOTOR_C, MAX_SPEED);

  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);
    fprintf(stdout, "current_color: %d\n", current_colour);

    // stop when u detect one of the colors and put them in some variable
    if (current_colour == 2 || current_colour == 3 || current_colour == 6) {
        BT_motor_port_start(MOTOR_C, 0); // stopping colour sesnor motor
        bottom_left = current_colour; // TODO: confrim assumption that motor first turns left
        break;
    }
  }

  fprintf(stdout, "done first bit\n");

  // keep turning right, until you see yellow
  BT_motor_port_start(MOTOR_C, -1 * MAX_SPEED );

  while (1) {
    int current_colour = read_colour(NUM_SMALL_SAMPLES);
        fprintf(stdout, "current_color: %d\n", current_colour);

    if (current_colour == 4) {
        BT_motor_port_start(MOTOR_C, 0); // stopping colour sesnor motor
        break;
    }
  }
  fprintf(stdout, "done first.5 bit\n");

  // now keep turning right until you seel green, white or blue
  BT_motor_port_start(MOTOR_C, -1 * MAX_SPEED);

  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);
            fprintf(stdout, "current_color: %d\n", current_colour);

    if (current_colour == 2 || current_colour == 3 || current_colour == 6) {
        BT_motor_port_start(MOTOR_C, 0); // stopping colour sesnor motor
        bottom_right = current_colour; // TODO: confrim assumption that motor first turns left
        break;
    }
  }
  // store in var

  // move robot fwd till you see black
  // move robot fwd, until you see green, white or blue
  // store that in var

  // maninder's code
  // arm should be all the way to left
  BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED); // move forward
  int state = 0;                                     // 0-before_black
                                                     // 1-black
                                                     // 2-after_black
  int col_ind = 0;
  while (1)
  { // go forward until crosses black
    col_ind = read_colour(NUM_SAMPLES);
            fprintf(stdout, "current_color: %d\n", col_ind);

    if (state == 0 && col_ind == 1) { // found road
      state = 1;
    } else if (state == 1 && (col_ind == 2 || col_ind == 3 || col_ind == 6))
    { // found colour after road
      state = 2;
      top_right = col_ind;
      BT_motor_port_start(MOTOR_A | MOTOR_D, 0);
      break;
    }
  }




  // turn color sensor left until you see black

  BT_motor_port_start(MOTOR_C, MAX_SPEED);

  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);
    
    if (current_colour == 1) {
        BT_motor_port_start(MOTOR_C, 0); // stopping colour sesnor motor
        break;
    }
  }

  // turn color sensor left until you see green, white or blue
  // store that in var
  BT_motor_port_start(MOTOR_C, MAX_SPEED);

  while (1) {
    int current_colour = read_colour(NUM_SAMPLES);
    
    if (current_colour == 2 || current_colour == 3 || current_colour == 6) {
        BT_motor_port_start(MOTOR_C, 0); // stopping colour sesnor motor
        top_left = current_colour; // TODO: confrim assumption that motor first turns left
        break;
    }
  }

  fprintf(stdout, "\n inside get_colours_at_intersection() \n");
  fprintf(stdout, "%d %d %d %d \n", bottom_left, bottom_right, top_right, top_left);



  // realign color sesnor motor - has to be at 90 degrees again
  // turn color sesnor right until you see black
}

// float get_colours(int *col_array, int *right_col, int *left_col)
// {
//   /***
//   Parses col_array to find the correct colours at the left and right of the array
//   Contains the correct logic to find these colours

//   ***/
//   int colors[10];

//   // c^b^y^b^c
//   // expecting: blue, white, black, yellow, green

//   // this will compress and remove noise
//   int cur_col=-1;
//   int cur_index=0;
//   for(int i=0; i<INTERSECTION_SAMPLES; i++){
//       if (cur_col != col_array[i]){
          
//       }
//   }

//   *right_col = 1; // black
//   *left_col = 3;  // green?
// }

// void read_colour_at_intersection()
// {
//   // Assume at intersection and robot stopped and that arm is ideal length
//   int col_array[INTERSECTION_SAMPLES];
//   int sample = INTERSECTION_SAMPLES;
//   int bottom_right_col, bottom_left_col, top_right_col, top_left_col;

//   // rotate color sensor motor right for 1 sec (hope its all the way)
//   BT_motor_port_start(MOTOR_C,MAX_SPEED);
//   // turn motor left slowly
//   // start sampling the arc motion
//   for (int i = 0; i < INTERSECTION_SAMPLES; i++)
//   {
//     col_array[i] = BT_read_colour_sensor(PORT_3);
//   }

//   get_colours(col_array, &bottom_right_col, &bottom_left_col); // this will somehow get colors from the array

//   // HALF WAY POINT - now go forward and get samples

//   // arm should be all the way to left
//   BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED); // move forward
//   int state = 0;                                     // 0-before_black
//                                                      // 1-black
//                                                      // 2-after_black
//   int col_ind = 0;
//   while (1)
//   { // go forward until crosses black
//     col_ind = read_colour(NUM_SAMPLES);
//     if (state == 0 && col_ind == 1) { // found road
//       state = 1;
//     } else if (state == 1 && col_ind != 1)
//     { // found colour after road
//       state = 2;
//       BT_motor_port_start(MOTOR_A | MOTOR_D, 0);
//       break;
//     }
//   }

//   // turn arm all the way to the right
//   // turn motor left slowly
//   // start sampling the arc motion
//   for (int i = 0; i < INTERSECTION_SAMPLES; i++)
//   {
//     col_array[i] = BT_read_colour_sensor(PORT_3);
//   }

//   get_colours(col_array, &top_right_col, &top_left_col); // this will somehow get colors from the array
// }


void turn_at_intersection(int direction)
{
  // ASSUME AT YELLOW AND ROAD FOUND
  traverse_road(&road_detected);
  traverse_road(&intersection_or_boundary_detected);
  int colourID = read_colour(NUM_SAMPLES);

  // reverse till back on road
  BT_motor_port_start(MOTOR_D | MOTOR_A, -20);
  while (!road_detected(DO_COLOUR_READING))
  {
  }
  BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);

  int onCurrentRoad = 1;
  BT_motor_port_start(MOTOR_A, direction * MAX_SPEED);
  BT_motor_port_start(MOTOR_D, direction * -MAX_SPEED);
  while (onCurrentRoad || !road_detected(DO_COLOUR_READING))
  {

    if (onCurrentRoad && !road_detected(DO_COLOUR_READING))
    {
      onCurrentRoad = 0;
    }
  }

  BT_motor_port_stop(MOTOR_D | MOTOR_A, 0);
}

void traverse_road(int (*exit_conditon_func)(int))
// keeps going forward until no more black
{
  fprintf(stdout, "\n inside traverse_road() \n");
  int col_ind;
  int turnDirection = 1;

  while (1)
  {
    fprintf(stdout, "traverse before read\n");

    fprintf(stdout, "traverse after read\n");
    col_ind = read_colour(NUM_SAMPLES);
    if (exit_conditon_func(col_ind))
    {
      fprintf(stdout, "exit cond true");
      BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);

      return;
    }
    BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED);

    if (col_ind != 1 && col_ind != 4 && col_ind != 7) // not black and not yellow
    {
      fprintf(stdout, "colour ID: %d\n", col_ind);
      BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
      // sleep(1);
      turnDirection = turn_till_black(turnDirection);
    }
  }
}

int turn_till_black(int turnDirection)
{
  fprintf(stdout, "\ninside turn_till_black()\n");
  // turn on way
  int time_stamp;
  int returnDirection = turnDirection;
  int col_ind;

  time_stamp = curr_time(); // time(NULL);

  int PENDULUM_DURATION = 10;

  while (1) // curr_time() - time_stamp < PENDULUM_DURATION
  {
    // fprintf(stdout, "\n difference:  %ld \n", curr_time() - time_stamp);

    if ((curr_time() - time_stamp) < (PENDULUM_DURATION / 4))
    {
      // fprintf(stdout, "return direction: %d\n", returnDirection);
      BT_motor_port_start(MOTOR_D, turnDirection * -MAX_SPEED);
      BT_motor_port_start(MOTOR_A, turnDirection * MAX_SPEED);
    }
    else if ((PENDULUM_DURATION / 4) <= (curr_time() - time_stamp) && (curr_time() - time_stamp) < (3 * PENDULUM_DURATION / 4))
    {
      returnDirection = turnDirection * -1;
      // fprintf(stdout, "return direction: %d\n", returnDirection);
      BT_motor_port_start(MOTOR_D, turnDirection * MAX_SPEED);
      BT_motor_port_start(MOTOR_A, turnDirection * -MAX_SPEED);
    }
    else if ((3 * PENDULUM_DURATION / 4) <= (curr_time() - time_stamp) && (curr_time() - time_stamp) < PENDULUM_DURATION)
    {
      // fprintf(stdout, "return direction: %d\n", returnDirection);
      BT_motor_port_start(MOTOR_D, turnDirection * -MAX_SPEED);
      BT_motor_port_start(MOTOR_A, turnDirection * MAX_SPEED);
    }
    else
    {
      time_stamp = curr_time(); // time(NULL);
      PENDULUM_DURATION += 5;
      printf("Pendulum: in %d", PENDULUM_DURATION);
    }

    col_ind = read_colour(NUM_SMALL_SAMPLES);
    if (col_ind == 1 || col_ind == 4 || col_ind == 7)
    {
      printf("\n just found black and yellow\n");
      // usleep(300*1000);
      BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
      printf("\n found black and yellow\n");
      // sleep(1);
      return returnDirection;
    }
  }

  // BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
  // return returnDirection;

  // for (int time_diff = 200; time_diff < 1000; time_diff += 200)
  // {
  //   // this loop increments time 200,400,600,800,1000
  //   //one direction
  //   BT_motor_port_start(MOTOR_D, 25);
  //   BT_motor_port_start(MOTOR_A, -25);
  //   while (curr_time - time_stamp < time_diff)
  //   {

  //     col_ind = read_colour(NUM_SAMPLES);
  //     if (col_ind == 1)
  //     { //is black
  //       BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
  //       return;
  //     }
  //   }
  //   //undo
  //   BT_motor_port_start(MOTOR_D, -25);
  //   BT_motor_port_start(MOTOR_A, 25);
  //   while (curr_time - time_stamp < time_diff)
  //   {

  //     // col_ind = read_colour(NUM_SAMPLES);
  //     // if (col_ind == 1)
  //     // { //is black
  //     //   BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
  //     //   return;
  //     // }
  //   }
  //   //other direction
  //   BT_motor_port_start(MOTOR_D, -25);
  //   BT_motor_port_start(MOTOR_A, 25);
  //   while (curr_time - time_stamp < time_diff)
  //   {

  //     // if (curr_time - time_stamp > 100) {
  //     //   BT_motor_port_start(MOTOR_D, 25);
  //     //   BT_motor_port_start(MOTOR_A, -25);
  //     // }

  //     col_ind = read_colour(NUM_SAMPLES);
  //     if (col_ind == 1)
  //     { //is blackBT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
  //       BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
  //       return;
  //     }
  //   }
  //   //undo

  //   BT_motor_port_start(MOTOR_D, -25);
  //   BT_motor_port_start(MOTOR_A, 25);
  //   while (curr_time - time_stamp < 200)
  //   {

  //     col_ind = read_colour(NUM_SAMPLES);
  //     if (col_ind == 1)
  //     { //is black
  //       BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
  //       return;
  //     }
  //   }
  // }
}