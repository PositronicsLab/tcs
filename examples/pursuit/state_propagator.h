#ifndef _PURSUIT_SHIP_STATE_PROPAGATOR_H_
#define _PURSUIT_SHIP_STATE_PROPAGATOR_H_

#include "space.h"
#include "ship.h"
#include "integrator.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/SpaceInformation.h>

//-----------------------------------------------------------------------------

class pp_state_propagator_c : public ompl::control::StatePropagator {
public:
  pp_integrator_c< ship_c > _integrator;
  const ompl::base::StateSpace *_statespace;
  space_c* _space;
  ship_c* _pred;
  ship_c* _prey;

  //---------------------------------------------------------------------------
  pp_state_propagator_c( const ompl::control::SpaceInformationPtr &si, space_c* space, ship_c* pred, ship_c* prey ) : 
    ompl::control::StatePropagator( si ),
    _integrator( si->getStateSpace().get(), 0.0 ),
    _statespace( si->getStateSpace().get() ),
    _space( space ),
    _pred( pred ),
    _prey( prey )
  { 
  }

  //---------------------------------------------------------------------------
  virtual void propagate( const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result ) const {
    _integrator.propagate( state, control, duration, result, _space, _pred, _prey );
  }

  //---------------------------------------------------------------------------
  void setIntegrationTimeStep( const double& time_step ) {
    _integrator.set_time_step( time_step );
  }

  //---------------------------------------------------------------------------
  double getIntegrationTimeStep( void ) const {
    return _integrator.get_time_step();
  }
};

//-----------------------------------------------------------------------------

#endif // _PURSUIT_SHIP_STATE_PROPAGATOR_H_

