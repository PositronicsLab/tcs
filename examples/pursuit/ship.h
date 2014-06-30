#ifndef _PURSUIT_SHIP_H_
#define _PURSUIT_SHIP_H_

//-----------------------------------------------------------------------------

#include <boost/shared_ptr.hpp>

#include <Ravelin/Pose3d.h>
#include <Ravelin/SpatialRBInertiad.h>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multifit_nlin.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/ODESolver.h>

#include "experiment.h"
#include "aabb.h"
#include "control_space.h"

//-----------------------------------------------------------------------------

class space_c;
typedef boost::shared_ptr<space_c> space_p;
class ship_c;
typedef boost::shared_ptr<ship_c> ship_p;

//-----------------------------------------------------------------------------
class ship_c {
public:

  //---------------------------------------------------------------------------
  // Members
  //---------------------------------------------------------------------------

  Ravelin::Pose3d _pose;
  Ravelin::SVelocityd _velocity;

  // the inertial mass matrix of the ship
  Ravelin::SpatialRBInertiad _inertial;

  space_p _space;
  //----------

  // the set of possible player types
  enum role_e {
    NONE = 0,
    PREY,
    PREDATOR
  };

  role_e role;

  ship_p adversary;

  double time;
  double dtime;

  double integration_step;
  double controller_step;

  // the ship's bounding box in the ship's frame of reference
  aabb_c ship_frame_bb;

   // whether or not a capture event has occurred
  bool capture;
  bool stopped;

  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  ship_c( role_e role );
  ship_c( ompl::base::StateSpace *_statespace );
 
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~ship_c( void );

  //---------------------------------------------------------------------------
  // Base Class Definitions
  //---------------------------------------------------------------------------
  virtual bool open( void ) { return false; }
  virtual bool read( std::vector<double>& q ) { return false; }
  virtual bool write( const std::vector<double>& u ) { return false; }
  virtual void reset( void ) { }

  virtual bool read_ke( double& ke ) { return false; }

public:
  static unsigned long compute_prey_command_seed( double time, double dtime );
  static double compute_predator_time_randval( double time, double dtime );

  // computes commands (forces) if the ship is prey
  static void compute_prey_command( const std::vector<double>& pred_state, const std::vector<double>& prey_state, std::vector<double>& prey_u, const double& time, const double& dtime, space_c* space );

  // normalizes the quaternion components of the state
  static void renormalize_state_quat(std::vector<double>& q);

  // compute the bounding box for the ship given a state
  aabb_c aabb( const std::vector<double>& q );

  // query whether a bounding box intersects another bounding box
  //bool intersects_any_obstacle( const aabb_c& mybb, aabb_c& obstacle );

  // query whether the ship intersects the world bounding box
  bool intersects_world_bounds( const aabb_c& mybb );

  // compute any force(field) that the boundary contributes to repel collision
  static Ravelin::Vector3d boundary_force( space_c* space, const Ravelin::Vector3d& pos, const Ravelin::Vector3d& vel );

  static Ravelin::SForced drag_force( Ravelin::SVelocityd& v, boost::shared_ptr<Ravelin::Pose3d> P_CoM );

  // computes a repulsive force for a given distance
  static double repulsive_force( double repulsion_factor, double dist );
  static double flee_force( double repulsion_factor, double dist );

  // - Feedback Control -
  // compute the desired state based on the current command and current state
  void compute_desired_state( const std::vector<double>& u, const std::vector<double>& x_current, std::vector<double>& x_desired );
 // computes the feedback command based on error in the current state
  void compute_feedback( const std::vector<double>& x_current, std::vector<double>& x_desired, const std::vector<double>& u_current, std::vector<double>& u_feedback );

  // - Dynamics -
  // compute the inverse dynamics for the ship 
  static void inv_dyn( space_c* space,  const std::vector<double>& q, const std::vector<double>& qdot_des, std::vector<double>& u, const Ravelin::SpatialRBInertiad& inertial );

  // ordinary differential equations for the ship
  static void ode( space_c* space, const std::vector<double>& q, const std::vector<double>& u, std::vector<double>& dq, const Ravelin::SpatialRBInertiad& inertial );

  // - OMPL --
  // plans using the ompl rrt planner
  //bool plan_rrt( const std::vector<double>& pred_q, const std::vector<double>& prey_q, std::vector<double>& u );

  bool compute_predator_plan( const std::vector<double>& pred_q, const std::vector<double>& prey_q, std::vector< std::vector<double> >& us, std::vector<double>& durations, unsigned& control_count );

 // the reference to the statespace the planner has generated
  ompl::base::StateSpace *_statespace;

  // query whether or not the state generated by the planner is valid 
  bool is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state);

  // allocates the predator/prey control sampler for the planner
  ompl::control::DirectedControlSamplerPtr allocate_pp_control_sampler( const ompl::control::SpaceInformation* si );

  // ode.  called by the eular_integrator_c in integrator.h
  void operator()( const ompl::base::State* state, const ompl::control::Control* control, std::vector<double>& dstate, const Ravelin::SpatialRBInertiad& inertial ) const;

  // post integration update method
  void update( ompl::base::State* state, const std::vector<double>& dstate ) const;

  static double compute_distance( const std::vector<double>& pred_state, const std::vector<double>& prey_state );

  private:
    void get_predator_control(const std::vector<double>& last_pred_q, const std::vector<double>& new_prey_q, const Ravelin::SpatialRBInertiad& predJ, space_c* space, double DT, std::vector<double>& u);
    void integrate_prey(const std::vector<double>& prey_q, const std::vector<double>& pred_q, const Ravelin::SpatialRBInertiad& Jprey, double time, space_c* space, double DT, std::vector<double>& qnew);
    void integrate_predator(const std::vector<double>& q, const std::vector<double>& u, const Ravelin::SpatialRBInertiad& J, space_c* space, double DT, std::vector<double>& qnew);
};

#endif // _PURSUIT_SHIP_H_
