// Initial testing of a bare-bones BlueTooth communication
// library for the EV3 - Thank you Lego for changing everything
// from the NXT to the EV3!

#include "btcomm.h"
#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <time.h>
#include <math.h>


long curr_time();

long curr_time()
// returns time in milliseconds since program started
{
  clock_t time=0;

  time = clock();
  time = (double) time /CLOCKS_PER_SEC *1000; 

  return time; 
}

int main(){
    clock_t start_t, end_t, total_t;

    start_t = clock();
    for(int i =0;i<10000000;i++){

    }
    end_t = clock();


    printf("\n start is %ld\n", start_t);
    printf("\n end is %ld\n",end_t);
    printf("\n total is %ld\n",end_t - start_t);
    return 0;
}