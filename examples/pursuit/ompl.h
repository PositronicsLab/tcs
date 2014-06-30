#ifndef _PURSUIT_OMPL_
#define _PURSUIT_OMPL_

//-----------------------------------------------------------------------------

#include <vector>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

//-----------------------------------------------------------------------------
void pp_vector_from_ship_vectors( const std::vector<double> x_pred, const std::vector<double>& x_prey, std::vector<double>& x_pp ) { 
  //assert( x_prey.size() == 13 );
  //assert( x_pred.size() == 13 );
  //assert( x_pp.size() == 26 );

  // x_pp needs to be sized before calling
  x_pp.resize( 26 );  // sanity check
  for( unsigned i = 0; i < 13; i++ ) {
    x_pp[i] = x_pred[i];
    x_pp[i + 13] = x_prey[i];
  }
}

//-----------------------------------------------------------------------------
void pp_ompl_state_to_vector( const ompl::base::StateSpace* space, const ompl::base::State* x_ompl, std::vector<double>& x_vector ) { 
  //assert( x_ompl.size() == 26 );
  //assert( x_vector.size() == 26 );

  // x_vector needs to be sized before calling
  x_vector.resize( 26 );  // sanity check
  for( unsigned i = 0; i < 26; i++ )
    x_vector[i] = *space->getValueAddressAtIndex( x_ompl, i);
}

//-----------------------------------------------------------------------------
void pp_ompl_state_to_pred_state_vector( const ompl::base::StateSpace* space, const ompl::base::State* x_ompl, std::vector<double>& x_vector ) { 
  //assert( x_ompl.size() == 26 );
  //assert( x_vector.size() == 13 );

  // x_vector needs to be sized before calling
  x_vector.resize( 13 );  // sanity check
  for( unsigned i = 0; i < 13; i++ )
    x_vector[i] = *space->getValueAddressAtIndex( x_ompl, i);
}

//-----------------------------------------------------------------------------
void pp_ompl_state_to_prey_state_vector( const ompl::base::StateSpace* space, const ompl::base::State* x_ompl, std::vector<double>& x_vector ) { 
  //assert( x_ompl.size() == 26 );
  //assert( x_vector.size() == 13 );

  // x_vector needs to be sized before calling
  x_vector.resize( 13 );  // sanity check
  for( unsigned i = 0; i < 13; i++ )
    x_vector[i] = *space->getValueAddressAtIndex( x_ompl, i+13);
}

//-----------------------------------------------------------------------------
void pp_vector_to_ompl_state( const std::vector<double>& x_vector, ompl::base::StateSpace *statespace, ompl::base::State *x_ompl ) {
  for( unsigned i = 0; i < 26; i++ )
    *statespace->getValueAddressAtIndex(x_ompl, i) = x_vector[i];
}

//-----------------------------------------------------------------------------
void ompl_control_to_vector( const ompl::control::Control* u_ompl, std::vector<double>& u_vector ) { 
  //assert( u_ompl.size() == 6 );
  //assert( u_vector.size() == 6 );

  // u_vector needs to be sized before calling
  u_vector.resize( 6 );  // sanity check
  for( unsigned i = 0; i < 6; i++ )
   u_vector[i] = u_ompl->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i];
}

//-----------------------------------------------------------------------------
void ompl_state_to_vector( ompl::base::StateSpace* space, const ompl::base::State* q_ompl, std::vector<double>& q_vector ) { 
  //assert( u_ompl.size() == 6 );
  //assert( u_vector.size() == 6 );

  // q_vector needs to be sized before calling
  q_vector.resize( 26 );  // sanity check
  for( unsigned i = 0; i < 26; i++ )
   q_vector[i] = *space->getValueAddressAtIndex( q_ompl, i );
}

//-----------------------------------------------------------------------------

#endif // _PURSUIT_OMPL_
