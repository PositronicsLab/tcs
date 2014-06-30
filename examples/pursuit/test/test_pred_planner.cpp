#include "../moby_space.h"
#include "../ship.h"
#include "../moby_xml.h"
//-----------------------------------------------------------------------------

moby_space_p space;
ship_p prey;
ship_p predator;

std::vector<double> prey_state;
std::vector<double> pred_state;
std::vector<double> prey_control;
std::vector<double> pred_control;

//-----------------------------------------------------------------------------

int main( int argc, char* argv[] ) {

  //-------------------------
  // read the initial state from the moby configuration file
  init_moby_xml( argc, argv );

  space = moby_space_p( new moby_space_c() );
  space->read( READ_MAP );

  for( unsigned i = 0; i < space->ships.size(); i++ ) {
    ship_p ship = space->ships[i];

    if( ship->role == ship_c::PREY )
      prey = ship;
    else if( ship->role == ship_c::PREDATOR )
      predator = ship;
  }

  prey_state.resize( 13 );
  pred_state.resize( 13 );
  prey_control.resize( 6 );
  pred_control.resize( 6 );

  for( unsigned i = 0; i < 6; i++ )
    pred_control[i] = 0;

  prey_state[0]  = -5;
  prey_state[1]  =  0;
  prey_state[2]  =  0;
  prey_state[3]  =  0;
  prey_state[4]  =  0;
  prey_state[5]  =  0;
  prey_state[6]  =  1;
  prey_state[7]  =  0;
  prey_state[8]  =  0;
  prey_state[9]  =  0;
  prey_state[10] =  0;
  prey_state[11] =  0;
  prey_state[12] =  0;

  pred_state[0]  =  5;
  pred_state[1]  =  0;
  pred_state[2]  =  0;
  pred_state[3]  =  0;
  pred_state[4]  =  0;
  pred_state[5]  =  0;
  pred_state[6]  =  1;
  pred_state[7]  =  0;
  pred_state[8]  =  0;
  pred_state[9]  =  0;
  pred_state[10] =  0;
  pred_state[11] =  0;
  pred_state[12] =  0;

  predator->time = 0;
  predator->dtime = 0.01;

  predator->plan_rrt( pred_state, prey_state, pred_control );

  printf( "pred: control[%f,%f,%f,%f,%f,%f]\n", pred_control[0], pred_control[1], pred_control[2], pred_control[3], pred_control[4], pred_control[5] );

}

