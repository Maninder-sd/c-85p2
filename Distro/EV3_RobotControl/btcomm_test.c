// Initial testing of a bare-bones BlueTooth communication
// library for the EV3 - Thank you Lego for changing everything
// from the NXT to the EV3!

#include "btcomm.h"
#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <time.h>
#include <math.h>

#define MAX_SPEED 10

void road_find();
void traverse_road(int (*exit_conditon_func)(void));
int turn_till_black(int);
long curr_time();
void turn_at_intersection(int);
void led_test();

long curr_time()
// returns time in milliseconds since program started
{
  clock_t time=0;

  time = clock();
  time = (double) time /CLOCKS_PER_SEC *1000; 

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

//just uncomment your bot's hex key to compile for your bot, and comment the other ones out.
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

  int TURN_DIRECTION = 1;
  turn_at_intersection(TURN_DIRECTION);
  //led_test();
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
    // BT_read_colour_sensor_RGB( PORT_1 ,RGB);
    // RGB[0] *=0.25;
    // RGB[1] *=0.25;
    // RGB[2] *=0.25;
    // printf("\n (R,G,B) is (%d,%d,%d)\n", RGB[0], RGB[1], RGB[2]);
    col_ind = BT_read_colour_sensor(PORT_1);
    printf("\n %d \n", col_ind);

    sleep(1);
  }
}

int read_colour(int x){
  // function does multiple reads and returns most frequent COLOR
  int col_readings[8] = {0,0,0,0,0,0,0,0};
  for(int i=0; i<9;i++){
    int index = BT_read_colour_sensor(PORT_1);
    col_readings[index] +=1; 
  }

  int max_ind = 0;
  for(int i=0;i<8;i++){
    if (col_readings[i] > col_readings[max_ind] ){
      max_ind = i;
    }
  }

  return max_ind;
}

void road_find()
// go forward until finds black
// calls traverse_road()
{
  fprintf(stdout, "\ninside road_find()\n");
  int is_lost = 1;
  int col_ind;

  while (is_lost)
  {
    BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED); 
    col_ind = BT_read_colour_sensor(PORT_1);
    if (col_ind == 1)
    { // find black
    BT_motor_port_start(MOTOR_A | MOTOR_D, 0);
      is_lost = 0;
      return; 
    }
  }
}

int road_detected() {
  return (BT_read_colour_sensor(PORT_1) == 1);
}

int intersection_or_boundary_detected() {
  int colourID = BT_read_colour_sensor(PORT_1);
  printf("int or bound : %d\n",colourID) ;
  return (colourID == 4 || colourID == 5); 
}

void printColour(int colourID) {
  fprintf(stdout, "Current colour reading: %d\n", colourID);
}
void turn_at_intersection(int direction)
{
// ASSUME AT YELLOW AND ROAD FOUND
  traverse_road(&road_detected);
  traverse_road(&intersection_or_boundary_detected);
  int colourID = BT_read_colour_sensor(PORT_1);

  //reverse till back on road
  BT_motor_port_start(MOTOR_D | MOTOR_A, -20);
  while (!road_detected()) {}
  BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);

  int onCurrentRoad = 1;
  BT_motor_port_start(MOTOR_A, direction * MAX_SPEED);
  BT_motor_port_start(MOTOR_D, direction * -MAX_SPEED);
  while(onCurrentRoad || !road_detected()) {
    
    if (onCurrentRoad && !road_detected()) onCurrentRoad = 0;
  }

  BT_motor_port_stop(MOTOR_D | MOTOR_A, 0);
}




void traverse_road(int (*exit_conditon_func)(void))
// keeps going forward until no more black
{
  fprintf(stdout, "\n inside traverse_road() \n");
  int col_ind;
  int turnDirection = 1;

  col_ind = BT_read_colour_sensor(PORT_1);
  while (1)
  {

    if (exit_conditon_func()) {
      BT_motor_port_stop(MOTOR_D | MOTOR_A, 1);
      
      return;
    }

    BT_motor_port_start(MOTOR_A | MOTOR_D, MAX_SPEED);
    col_ind = BT_read_colour_sensor(PORT_1);
    if (col_ind != 1 && col_ind != 4 && col_ind != 7)//not black and not yellow
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
  //turn on way
  int time_stamp;
  int returnDirection = turnDirection;
  int col_ind;

  time_stamp = curr_time(); //time(NULL);

  int PENDULUM_DURATION = 15;
  BT_motor_port_start(MOTOR_D, turnDirection * -MAX_SPEED);
  BT_motor_port_start(MOTOR_A, turnDirection * MAX_SPEED);
  while (curr_time() - time_stamp < PENDULUM_DURATION)
  {
    // fprintf(stdout, "\n difference:  %ld \n", curr_time() - time_stamp);
    if (curr_time() - time_stamp > PENDULUM_DURATION / 3)
    {
      returnDirection = turnDirection * -1;
      // fprintf(stdout, "return direction: %d\n", returnDirection);
      BT_motor_port_start(MOTOR_D, turnDirection * MAX_SPEED);
      BT_motor_port_start(MOTOR_A, turnDirection * -MAX_SPEED);
      
    }
    col_ind = BT_read_colour_sensor(PORT_1);
    if (col_ind == 1 || col_ind == 4 || col_ind == 7)
    {
      printf("\n just found black and yellow\n");
      //usleep(300*1000);
      BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
      printf("\n found black and yellow\n");
      //sleep(1);
      return returnDirection;
    }
  }
  
  BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
  return returnDirection;

  // for (int time_diff = 200; time_diff < 1000; time_diff += 200)
  // {
  //   // this loop increments time 200,400,600,800,1000
  //   //one direction
  //   BT_motor_port_start(MOTOR_D, 25);
  //   BT_motor_port_start(MOTOR_A, -25);
  //   while (curr_time - time_stamp < time_diff)
  //   {

  //     col_ind = BT_read_colour_sensor(PORT_1);
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

  //     // col_ind = BT_read_colour_sensor(PORT_1);
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

  //     col_ind = BT_read_colour_sensor(PORT_1);
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

  //     col_ind = BT_read_colour_sensor(PORT_1);
  //     if (col_ind == 1)
  //     { //is black
  //       BT_motor_port_stop(MOTOR_A | MOTOR_D, 0);
  //       return;
  //     }
  //   }
  // }
}