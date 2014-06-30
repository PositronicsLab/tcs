#include "space.h"

//-----------------------------------------------------------------------------
// Constructors
//-----------------------------------------------------------------------------
space_c::space_c( void ) { 
  _prey_idx = -1;
  _pred_idx = -1;
}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
space_c::~space_c( void ) {

}

//-----------------------------------------------------------------------------
ship_p space_c::predator( void ) {
  assert( ships[_pred_idx] );

  return ships[_pred_idx];
}

//-----------------------------------------------------------------------------
ship_p space_c::prey( void ) {
  assert( ships[_prey_idx] );

  return ships[_prey_idx];
}

//-----------------------------------------------------------------------------
void space_c::update( void ) {
  assert( ships.size() == 2 );

  _prey_idx = -1;
  _pred_idx = -1;

  for( unsigned i = 0; i < ships.size(); i++ ) {
    if( ships[i]->role == ship_c::PREDATOR )
      _pred_idx = i;
    if( ships[i]->role == ship_c::PREY )
      _prey_idx = i;
  }

  assert( _pred_idx != -1 && _prey_idx != -1 );
}

//-----------------------------------------------------------------------------
