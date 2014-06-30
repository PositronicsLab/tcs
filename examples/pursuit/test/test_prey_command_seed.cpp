#include "../ship.h"
#include <stdio.h>

int main( int argc, char* argv[] ) {
  double time = 0.0;
  double dtime = 0.001;

  for( unsigned i = 0; i < 10000; i++ ) {
    unsigned long seed = ship_c::compute_prey_command_seed( time, dtime );
    printf( "seed[%lu], time[%f], dtime[%f]\n", seed, time, dtime );
    time += dtime;
  }

  return 0;
}
