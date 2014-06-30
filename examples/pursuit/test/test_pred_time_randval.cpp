#include "../ship.h"
#include <stdio.h>

int main( int argc, char* argv[] ) {
  double time = 0.0;
  double dtime = 0.001;

  for( unsigned i = 0; i < 10000; i++ ) {
    double rand_time = ship_c::compute_predator_time_randval( time, dtime );
    printf( "rand_time[%f], time[%f], dtime[%f]\n", rand_time, time, dtime );
    time += dtime;
  }

  return 0;
}
