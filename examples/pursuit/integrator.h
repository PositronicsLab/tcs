#ifndef _INTEGRATOR_H_
#define _INTEGRATOR_H_

#include "utilities.h"
#include "space.h"
#include "ship.h"
#include "ompl.h"

#include <valarray>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/Control.h>

//-----------------------------------------------------------------------------

// the integrator for the predator and the prey
template<typename F>
class pp_integrator_c {
private:
  ompl::base::StateSpace *_statespace;
  double _time_step;
  F      _ode;

  //---------------------------------------------------------------------------
public:
  pp_integrator_c( ompl::base::StateSpace *statespace, const double& time_step ) : 
    _statespace( statespace ), 
    _time_step( time_step), 
    _ode( statespace )
  { }

  //---------------------------------------------------------------------------
  void propagate( const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result, space_c* space, ship_c* pred, ship_c* prey ) const {
    std::vector<double> state_pred( 13, 0 );
    std::vector<double> state_prey( 13, 0 );
    std::vector<double> dstate_pred( 13, 0 );
    std::vector<double> dstate_prey( 13, 0 );
    std::vector<double> u_pred( 6, 0 );
    std::vector<double> u_prey( 6, 0 );

    _statespace->copyState( result, start );

    // get the states of the predator and prey
    pp_ompl_state_to_pred_state_vector( _statespace, start, state_pred );
    pp_ompl_state_to_prey_state_vector( _statespace, start, state_prey );

    // get the current control of the predator
    ompl_control_to_vector( control, u_pred );

    // compute the anticipated prey command
    ship_c::compute_prey_command( state_pred, state_prey, u_prey, pred->time, pred->dtime, space );

    // get the ODEs for the predator and prey
    ship_c::ode( space, state_pred, u_pred, dstate_pred, pred->_inertial );
    ship_c::ode( space, state_prey, u_prey, dstate_prey, prey->_inertial );

std::cout << "predator state space before integrating:";
for (unsigned i=0; i< state_pred.size(); i++)
  std::cout << state_pred[i] << " ";
std::cout << std::endl;
std::cout << "prey state space before integrating:";
for (unsigned i=0; i< state_prey.size(); i++)
  std::cout << state_prey[i] << " ";
std::cout << std::endl;
std::cout << "predator control:";
for (unsigned i=0; i< u_pred.size(); i++)
  std::cout << u_pred[i] << " ";
std::cout << std::endl;
std::cout << "prey control:";
for (unsigned i=0; i< u_prey.size(); i++)
  std::cout << u_prey[i] << " ";
std::cout << std::endl;

    // update the predator and prey states
    for (unsigned i=0; i< state_pred.size(); i++) {
      state_pred[i] += dstate_pred[i] * _time_step;
      state_prey[i] += dstate_prey[i] * _time_step;
    }

    ship_c::renormalize_state_quat( state_pred );
    ship_c::renormalize_state_quat( state_prey );

std::cout << "predator state space after integrating:";
for (unsigned i=0; i< state_pred.size(); i++)
  std::cout << state_pred[i] << " ";
std::cout << std::endl;
std::cout << "prey state space after integrating:";
for (unsigned i=0; i< state_prey.size(); i++)
  std::cout << state_prey[i] << " ";
std::cout << std::endl;
    // convert back to the state space
    std::vector<double> qf( 26 );
    pp_vector_from_ship_vectors( state_pred, state_prey, qf );
    pp_vector_to_ompl_state( qf, _statespace, result );

    _statespace->enforceBounds( result );
  }

  //---------------------------------------------------------------------------
  double get_time_step( void ) const {
    return _time_step;
  }

  //---------------------------------------------------------------------------
  void set_time_step( const double& time_step ) {
    _time_step = time_step;
  }
};


#endif // _INTEGRATOR_H_

