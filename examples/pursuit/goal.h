#ifndef _PURSUIT_SHIP_GOAL_H_
#define _PURSUIT_SHIP_GOAL_H_

//-----------------------------------------------------------------------------

//#include "state.h"

#include <ompl/base/State.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>

//-----------------------------------------------------------------------------
/*
class ship_goal_c : public ompl::base::GoalState {
public:
  const ompl::base::StateSpace *statespace;
  double SATISFACTION_THRESHOLD;

  //---------------------------------------------------------------------------
  ship_goal_c( const ompl::base::SpaceInformationPtr& si ) : 
    ompl::base::GoalState(si),
    statespace( si->getStateSpace().get() )
  {
    SATISFACTION_THRESHOLD = 1e-4;

  }

  //---------------------------------------------------------------------------
  virtual bool isSatisfied( const ompl::base::State* state ) const {
    ship_state_c q( statespace, state );
    if( distanceGoal( state ) < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

  //---------------------------------------------------------------------------
  virtual bool isSatisfied( const ompl::base::State* state, double *distance ) const {
    ship_state_c q( statespace, state );

    *distance = distanceGoal( state );
    if( *distance < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

  //---------------------------------------------------------------------------
  virtual double distanceGoal( const ompl::base::State* state ) const {
    std::vector<double> g( 13 );
    for( unsigned i=0; i< 13; i++ )
      g[i] = value(i);
    ship_state_c q( statespace, state );
    std::vector<double> d( 13 );
    for( unsigned i = 0; i < d.size(); i++ ) 
      d[i] = q.value(i) - g[i];

    return sqrt( d[0] * d[0] + d[1] * d[1] + d[2] * d[2] );
  }

  //---------------------------------------------------------------------------
  double value( const unsigned int i ) const {
    assert( i < 13 );
    return *statespace->getValueAddressAtIndex(state_, i);
  }
};
*/
//-----------------------------------------------------------------------------

class pp_goal_c : public ompl::base::GoalRegion {
public:
  const ompl::base::StateSpace *statespace;
  double SATISFACTION_THRESHOLD;

  //---------------------------------------------------------------------------
  pp_goal_c( const ompl::base::SpaceInformationPtr& si ) : 
    ompl::base::GoalRegion(si),
    statespace( si->getStateSpace().get() )
  {
    SATISFACTION_THRESHOLD = 1e-4;

  }

  //---------------------------------------------------------------------------
  virtual bool isSatisfied( const ompl::base::State* state ) const {
    //pp_state_c q( statespace, state );
    if( distanceGoal( state ) < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

  //---------------------------------------------------------------------------
  virtual bool isSatisfied( const ompl::base::State* state, double *distance ) const {
    //ship_state_c q( statespace, state );

    *distance = distanceGoal( state );
    if( *distance < SATISFACTION_THRESHOLD ) return true;
    return false;
  }

  //---------------------------------------------------------------------------
  // TODO: Potential optimization can remove the vectors entirely from this
  // method allowing for no short term mallocs.
  virtual double distanceGoal( const ompl::base::State* state ) const {
    //pp_state_c q( statespace, state );
    std::vector<double> q( 26 );
    pp_ompl_state_to_vector( statespace, state, q );
        
    //std::vector<double> pred_q = q.pred_vector();
    //std::vector<double> prey_q = q.prey_vector();
    std::vector<double> dq( 13 );

    // compute the difference vector
    for( unsigned i = 0; i < 13; i++ ) 
      //dq[i] = prey_q[i] - pred_q[i];
      dq[i] = q[i+13] - q[i];

    // normalize difference quaternion
    double me = 0;
    for( unsigned i = 3; i < 7; i++ )
      me += dq[i] * dq[i];
    double rme = sqrt(me);
    for( unsigned i = 3; i < 7; i++ )
      dq[i] /= rme;

// EMD: zero angular components
for (unsigned i=3; i< 7; i++)
  dq[i] = 0.0;

  // EMD: shoot for predator angular velocity of 0
  for (unsigned i=10; i< 13; i++)
    dq[i] = q[i];

// EMD: scale linear and angular velocities by step size 
for (unsigned i=7; i< 13; i++)
//  dq[i] = q[i];
  dq[i] *= PLANNER_STEP_SIZE;

    // compute the magnitude (squared) of dq   
    double mdq = 0;
    for( unsigned i = 0; i < 13; i++ ) 
      mdq += dq[i] * dq[i];
 
    // return the magnitude of dq 
    return sqrt(mdq);
  }
/*
  //---------------------------------------------------------------------------
  double value( const unsigned int i ) const {
    assert( i < pp_state_c::size() );
    return *statespace->getValueAddressAtIndex(state_, i);
  }
*/
};

//-----------------------------------------------------------------------------

#endif // _PURSUIT_SHIP_GOAL_H_

