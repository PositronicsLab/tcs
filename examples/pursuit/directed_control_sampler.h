#ifndef _PURSUIT_SHIP_DIRECTED_CONTROL_SAMPLER_H_
#define _PURSUIT_SHIP_DIRECTED_CONTROL_SAMPLER_H_

//-----------------------------------------------------------------------------

#include "experiment.h"
#include "ship.h"
#include "space.h"

#include "utilities.h"
#include "ompl.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

//-----------------------------------------------------------------------------
/// Control Sampler for Predator and Prey pursuit scenario
class pp_control_sampler_c : public ompl::control::SimpleDirectedControlSampler {
public:
  space_c* _space;
  ship_c* _pred;
  ship_c* _prey;

  //---------------------------------------------------------------------------
  // Contructors
  //---------------------------------------------------------------------------
  pp_control_sampler_c( const ompl::control::SpaceInformation *si, space_c* space, ship_c* pred, ship_c* prey, unsigned int k = 1 ) :
    SimpleDirectedControlSampler(si, k) 
  {
    _space = space;
    _pred = pred;
    _prey = prey;
  }

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~pp_control_sampler_c( void ) {

  }

  //---------------------------------------------------------------------------
  // Overloading base class
  //---------------------------------------------------------------------------
  virtual unsigned int getBestControl( ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest, const ompl::control::Control *previous ) {
    const unsigned NUM_STEPS = 1;
    std::vector<double> q0_pred( 13, 0 );
    std::vector<double> q0_prey( 13, 0 );
    std::vector<double> q1_pred( 13, 0 );
    //std::vector<double> q1_prey( 13, 0 );
    std::vector<double> dq_pred( 13, 0 );
    std::vector<double> dq_prey( 13, 0 );

    std::vector<double> u_prey( 6, 0 );
    std::vector<double> u_pred( 6, 0 );
    std::vector<double> qf( 26, 0 );

    ompl::base::StateSpace *statespace = si_->getStateSpace().get();
    const double DT = PLANNER_STEP_SIZE;

    // initialize state vectors from ompl state information
    pp_ompl_state_to_pred_state_vector( statespace, source, q0_pred );
    pp_ompl_state_to_pred_state_vector( statespace, dest, q1_pred );
    pp_ompl_state_to_prey_state_vector( statespace, source, q0_prey );
    //pp_ompl_state_to_prey_state_vector( statespace, dest, q1_prey );

    // normalize the destination state quaternion for the predator; the
    // prey's state cannot be determined using 'dest' (b/c we do not plan
    // for commands to get the prey from one state to another)
    ship_c::renormalize_state_quat(q1_pred);

    // generate differential vector
    for( unsigned i = 0; i < 13; i++ )
      dq_pred[i] = q1_pred[i] - q0_pred[i];

    // update dq to update position
    dq_pred[7] += dq_pred[0];
    dq_pred[8] += dq_pred[1];
    dq_pred[9] += dq_pred[2];

    // update dq to update velocity components
    Ravelin::Quatd edot(dq_pred[3], dq_pred[4], dq_pred[5], dq_pred[6]);
    Ravelin::Quatd e(q0_pred[3], q0_pred[4], q0_pred[5], q0_pred[6]);
    Ravelin::Vector3d omega = Ravelin::Quatd::to_omega(e, edot);
    dq_pred[10] += omega[0];
    dq_pred[11] += omega[1];
    dq_pred[12] += omega[2];

    // call inverse dynamics to generate command for predator
    ship_c::inv_dyn( _space, q0_pred, dq_pred, u_pred, _pred->_inertial );

    // use prey policy to generate command for prey
    // NOTE: we *might* want to try multiple random commands generated using 
    // ship_c::prey_command(.) [we can effect that using multiple random inputs 
    // for the time input]. We can then integrate the prey state multiple times
    // and see which the predator can get closest to. This strategy would
    // only need to be tried if the planner has a really hard time planning. 

    //double time_input = (double) rand();
    double time_input = ship_c::compute_predator_time_randval( _pred->time, _pred->dtime );

    ship_c::compute_prey_command( q0_pred, q0_prey, u_prey, time_input, _pred->dtime, _space );

    // copy u to control
    double *ctl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    for( unsigned i = 0; i < 6; i++ )
      ctl[i] = u_pred[i];

    // integrate for the desired number of steps
    for( unsigned k = 0; k < NUM_STEPS; k++ ) {
      // get the ODEs
      ship_c::ode( _space, q0_pred, u_pred, dq_pred, _pred->_inertial );
      ship_c::ode( _space, q0_prey, u_prey, dq_prey, _prey->_inertial );

      // first, update the velocity components using Euler integration
      for( unsigned i = 7; i < 13; i++ ) {
        q0_pred[i] += DT*dq_pred[i];
        q0_prey[i] += DT*dq_prey[i];
      }

      // now update the position components using Euler integration
      q0_pred[0] += DT*q0_pred[7];
      q0_pred[1] += DT*q0_pred[8];
      q0_pred[2] += DT*q0_pred[9];
      q0_prey[0] += DT*q0_prey[7];
      q0_prey[1] += DT*q0_prey[8];
      q0_prey[2] += DT*q0_prey[9];

      // convert the angular velocities to quaternion time derivatives 
      Ravelin::Quatd e_pred( q0_pred[3], q0_pred[4], q0_pred[5], q0_pred[6] );
      Ravelin::Vector3d omega_pred( q0_pred[10], q0_pred[11], q0_pred[12] );
      Ravelin::Quatd edot_pred = Ravelin::Quatd::deriv( e_pred, omega_pred );

      Ravelin::Quatd e_prey( q0_prey[3], q0_prey[4], q0_prey[5], q0_prey[6] );
      Ravelin::Vector3d omega_prey( q0_prey[10], q0_prey[11], q0_prey[12] );
      Ravelin::Quatd edot_prey = Ravelin::Quatd::deriv( e_prey, omega_prey );

      // update the orientation components using the angular velocity and
      // Euler integration
      q0_pred[3] += DT*edot_pred[0];
      q0_pred[4] += DT*edot_pred[1];
      q0_pred[5] += DT*edot_pred[2];
      q0_pred[6] += DT*edot_pred[3];

      q0_prey[3] += DT*edot_prey[0];
      q0_prey[4] += DT*edot_prey[1];
      q0_prey[5] += DT*edot_prey[2];
      q0_prey[6] += DT*edot_prey[3];

      // renormalize quaternions
      ship_c::renormalize_state_quat(q0_pred);
      ship_c::renormalize_state_quat(q0_prey);
    }

    // update destination state from q0_pred, q0_prey
    pp_vector_from_ship_vectors( q0_pred, q0_prey, qf );
    pp_vector_to_ompl_state( qf, statespace, dest );

    // return number of steps taken 
    return (unsigned) NUM_STEPS;
  }

};
//-----------------------------------------------------------------------------

#endif // _PURSUIT_SHIP_DIRECTED_CONTROL_SAMPLER_H_

