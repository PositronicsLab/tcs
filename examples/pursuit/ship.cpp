#include <tcs/time.h>
#include <tcs/cpu.h>
#include "ship.h"

#include "space.h"
#include "ompl.h"
#include "control_space.h"
#include "goal.h"
#include "utilities.h"

#include "directed_control_sampler.h"
#include "state_propagator.h"

#include <tcs/log.h>

//-----------------------------------------------------------------------------
// Constructors
//-----------------------------------------------------------------------------
ship_c::ship_c( ship_c::role_e role ) { 
  capture = false;
  this->role = role;
}

//-----------------------------------------------------------------------------
/// Constructs a temporary for ompl planning
ship_c::ship_c( ompl::base::StateSpace *statespace ) : _statespace(statespace) 
{
  capture = false;
}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
ship_c::~ship_c( void ) {

}

//-----------------------------------------------------------------------------
// Utilities
//-----------------------------------------------------------------------------
/// Renormalize the quaternion component of a state vector
void ship_c::renormalize_state_quat( std::vector<double>& q ) {
  double qnorm = 0.0;
  for( unsigned i = 3; i < 7; i++ )
    qnorm += q[i] * q[i];
  qnorm = std::sqrt( qnorm );
  for( unsigned i = 3; i < 7; i++ )
    q[i] /= qnorm;
}

//-----------------------------------------------------------------------------
// Spatial Queries
//-----------------------------------------------------------------------------
/// Build an axis aligned bounding box of the ship based on state vector
aabb_c ship_c::aabb( const std::vector<double>& q ) {

  //pull the center of the ship from the state vector
  Ravelin::Origin3d c( q[0], q[1], q[2] );
  //rotate the ship frame aabb
  Ravelin::Quatd e( q[3], q[4], q[5], q[6] );
  e.normalize();  // Just in case

  // TODO: This needs to use a local buffer to minimize mallocs
  std::vector<Ravelin::Origin3d> verts(8);
  //  x,  y,  z
  verts[0] = Ravelin::Origin3d( ship_frame_bb.extens[0] - ship_frame_bb.center[0], ship_frame_bb.extens[1] - ship_frame_bb.center[1], ship_frame_bb.extens[2] - ship_frame_bb.center[2] );
  //  x,  y, -z
  verts[1] = Ravelin::Origin3d( ship_frame_bb.extens[0] - ship_frame_bb.center[0], ship_frame_bb.extens[1] - ship_frame_bb.center[1], -(ship_frame_bb.extens[2] - ship_frame_bb.center[2]) );
  //  x, -y,  z
  verts[2] = Ravelin::Origin3d( ship_frame_bb.extens[0] - ship_frame_bb.center[0], -(ship_frame_bb.extens[1] - ship_frame_bb.center[1]), ship_frame_bb.extens[2] - ship_frame_bb.center[2] );
  //  x, -y, -z
  verts[3] = Ravelin::Origin3d( ship_frame_bb.extens[0] - ship_frame_bb.center[0], -(ship_frame_bb.extens[1] - ship_frame_bb.center[1]), -(ship_frame_bb.extens[2] - ship_frame_bb.center[2]) );
  // -x,  y,  z
  verts[4] = Ravelin::Origin3d( -(ship_frame_bb.extens[0] - ship_frame_bb.center[0]), ship_frame_bb.extens[1] - ship_frame_bb.center[1], ship_frame_bb.extens[2] - ship_frame_bb.center[2] );
  // -x,  y, -z
  verts[5] = Ravelin::Origin3d( -(ship_frame_bb.extens[0] - ship_frame_bb.center[0]), ship_frame_bb.extens[1] - ship_frame_bb.center[1], -(ship_frame_bb.extens[2] - ship_frame_bb.center[2]) );
  // -x, -y,  z
  verts[6] = Ravelin::Origin3d( -(ship_frame_bb.extens[0] - ship_frame_bb.center[0]), -(ship_frame_bb.extens[1] - ship_frame_bb.center[1]), ship_frame_bb.extens[2] - ship_frame_bb.center[2] );
  // -x, -y, -z
  verts[7] = Ravelin::Origin3d( -(ship_frame_bb.extens[0] - ship_frame_bb.center[0]), -(ship_frame_bb.extens[1] - ship_frame_bb.center[1]), -(ship_frame_bb.extens[2] - ship_frame_bb.center[2]) );

  double ext_x = 0, ext_y = 0, ext_z = 0;
  for( unsigned i = 0; i < 8; i++ ) {
    // compute vertex in rotation frame
    Ravelin::Origin3d v_rot = e * verts[i];

    ext_x = std::max( ext_x, fabs( v_rot.x() ) );
    ext_y = std::max( ext_y, fabs( v_rot.y() ) );
    ext_z = std::max( ext_z, fabs( v_rot.z() ) );
  }
  
  return aabb_c ( c, Ravelin::Origin3d( ext_x, ext_y, ext_z ) );
}
/*
//-----------------------------------------------------------------------------
/// Queries whether the ship intersects any obstacle in the world
bool ship_c::intersects_any_obstacle( const aabb_c& mybb, aabb_c& obstacle ) {
  for( unsigned i = 0; i < obstacles.size(); i++ ) {
    if( aabb_c::intersects( mybb, obstacles[i] ) ) {
      obstacle = obstacles[i];
      return true;
    }
  }
  return false;
}
*/
//-----------------------------------------------------------------------------
/// Queries whether the ship remains inside the world boundary
bool ship_c::intersects_world_bounds( const aabb_c& mybb ) {
  return !aabb_c::intersects( mybb, _space->bounds );
}

//------------------------------------------------------------------------------
// State Management
//------------------------------------------------------------------------------
/// Compute the desired state given a command 
void ship_c::compute_desired_state( const std::vector<double>& u, const std::vector<double>& x_current, std::vector<double>& x_desired ) {

  double DT = dtime;
  std::vector<double> dq( 13, 0 );

  printf( "ode: DT[%f]\n", DT );

  // use forward dynamics to compute the change in state applying the command causes
  ode( _space.get(), x_current, u, dq, _inertial );

  // integrate the change in state with the current state to get the desired state
  for( unsigned i = 0; i < 13; i++ ) {
    x_desired[i] = x_current[i] + dq[i] * DT;
  }

  renormalize_state_quat( x_desired );
}

//-----------------------------------------------------------------------------
// Dynamics
//-----------------------------------------------------------------------------
/// Computes the differential equations of motion for a ship
void ship_c::ode( space_c* space, const std::vector<double>& q, const std::vector<double>& u, std::vector<double>& dq, const Ravelin::SpatialRBInertiad& inertial ) {

  // - Newton-Euler -

  // setup the ship quaternion
  Ravelin::Quatd e(q[3], q[4], q[5], q[6]);

  // setup the ship's pose in a center-of-mass frame
  boost::shared_ptr<Ravelin::Pose3d> P( new Ravelin::Pose3d() );
  P->x = Ravelin::Origin3d(q[0], q[1], q[2]);
  P->q.set_identity();

  // the inertia is setup in the body frame
  boost::shared_ptr<Ravelin::Pose3d> Pi( new Ravelin::Pose3d() );
  Pi->x = P->x;
  Pi->q = e; 

  // get the linear velocity 
  Ravelin::Vector3d omega(q[10], q[11], q[12], P);
  Ravelin::SVelocityd vel(q[10], q[11], q[12], q[7], q[8], q[9], P);
  Ravelin::Quatd edot = Ravelin::Quatd::deriv(e, omega);

  // get the external force
  Ravelin::SForced force(u[0], u[1], u[2], u[3], u[4], u[5], P);

  // transform the inertia to frame P
  Ravelin::SpatialRBInertiad J_f = inertial;
  J_f.pose = Pi;
  Ravelin::SpatialRBInertiad J = Ravelin::Pose3d::transform(P, J_f);

  Ravelin::SForced drag = drag_force( vel, P );
  Ravelin::Vector3d bf = boundary_force( space, Ravelin::Vector3d(P->x[0],P->x[1],P->x[2]), vel.get_linear() );
  Ravelin::SForced bforce( bf[0], bf[1], bf[2], 0.0, 0.0, 0.0, P );

  // get the acceleration
// TODO: fix this
//  Ravelin::SAcceld acc = J.inverse_mult(force + drag + bforce - vel.cross(J * vel));
  Ravelin::SAcceld acc = J.inverse_mult(force + bforce - vel.cross(J * vel));
  Ravelin::Vector3d xdd = acc.get_linear();
  Ravelin::Vector3d alpha = acc.get_angular();

  // update dq (linear velocity)
  dq[0] = q[7];
  dq[1] = q[8];
  dq[2] = q[9];

  // update dq (quaternion derivative) 
  dq[3] = edot.x;
  dq[4] = edot.y;
  dq[5] = edot.z;
  dq[6] = edot.w; 
  // Note: Renormalize? => Not now! Only After integrating!

  // update dq (linear acceleration)
  dq[7] = xdd[0];
  dq[8] = xdd[1];
  dq[9] = xdd[2];

  // update dq (angular acceleration)
  dq[10] = alpha[0];
  dq[11] = alpha[1];
  dq[12] = alpha[2];

}

//-----------------------------------------------------------------------------
/// Compute the inverse dynamics of the ship
void ship_c::inv_dyn( space_c* space, const std::vector<double>& q, const std::vector<double>& qdot_des, std::vector<double>& u, const Ravelin::SpatialRBInertiad& inertial ) {

  // - Newton-Euler -

  // setup the ship quaternion
  Ravelin::Quatd e(q[3], q[4], q[5], q[6]);

  // setup the ship's pose in a center-of-mass frame
  boost::shared_ptr<Ravelin::Pose3d> P( new Ravelin::Pose3d() );
  P->x = Ravelin::Origin3d(q[0], q[1], q[2]);
  P->q.set_identity();

  // the inertia is setup in the body frame
  boost::shared_ptr<Ravelin::Pose3d> Pi( new Ravelin::Pose3d() );
  Pi->x = P->x;
  Pi->q = e; 

  // get the linear velocity 
  Ravelin::Vector3d omega(q[10], q[11], q[12], P);
  Ravelin::SVelocityd vel(q[10], q[11], q[12], q[7], q[8], q[9], P);
  //Ravelin::Quatd edot = Quatd::deriv(e, omega);

  // get desired linear acceleration 
  Ravelin::Vector3d xdd(P);
  xdd[0] = qdot_des[7];
  xdd[1] = qdot_des[8];
  xdd[2] = qdot_des[9];

  // get desired angular acceleration 
  Ravelin::Vector3d alpha(P);
  alpha[0] = qdot_des[10];
  alpha[1] = qdot_des[11];
  alpha[2] = qdot_des[12];

  // setup the acceleration
  Ravelin::SAcceld acc(P);
  acc.set_linear(xdd);
  acc.set_angular(alpha);

  // transform the inertia to frame P
  Ravelin::SpatialRBInertiad J_f = inertial; 
  J_f.pose = Pi;
  Ravelin::SpatialRBInertiad J = Ravelin::Pose3d::transform(P, J_f);

  // get the acceleration
  Ravelin::SForced pseudo = vel.cross(J*vel);
  Ravelin::SForced force = J*acc + pseudo;

  Ravelin::SForced drag = drag_force( vel, P );
//  force -= drag;
  Ravelin::Vector3d bf = boundary_force( space, Ravelin::Vector3d(P->x[0],P->x[1],P->x[2]), vel.get_linear() );
  Ravelin::SForced bforce( bf[0], bf[1], bf[2], 0.0, 0.0, 0.0, P );
  force -= bforce;  

  // store the forces necessary to achieve that acceleration
  u.resize(6);
  Ravelin::Vector3d f = force.get_force();
  Ravelin::Vector3d tau = force.get_torque();

  //TODO: add perturbation to u[.]
  const double NOISE_SCAL = 1.0;
  u[0] = f[0];
  u[1] = f[1];
  u[2] = f[2];
  u[3] = 0.0;//tau[0];  
  u[4] = 0.0;//tau[1];
  u[5] = 0.0;//tau[2];
//  u[0] += (double) rand() * NOISE_SCAL / RAND_MAX - NOISE_SCAL * 0.5;
//  u[1] += (double) rand() * NOISE_SCAL / RAND_MAX - NOISE_SCAL * 0.5;
//  u[2] += (double) rand() * NOISE_SCAL / RAND_MAX - NOISE_SCAL * 0.5;
}

//------------------------------------------------------------------------------
unsigned long ship_c::compute_prey_command_seed( double time, double dtime ) {
  assert( dtime > 0 );

  unsigned long factor = (unsigned long)PREY_COMMAND_SEED_FACTOR;
  unsigned long seed = (unsigned long)(time / dtime) / factor;
  seed += EXPERIMENTAL_SEED;
  return seed;
}

//------------------------------------------------------------------------------
double ship_c::compute_predator_time_randval( double time, double dtime ) {
  assert( dtime > 0 );

  unsigned int seed = (unsigned int)(time / dtime);
  srand( seed );

  double rval = ((double)rand() / (double)RAND_MAX) * (double)PREDATOR_TIME_RAND_FACTOR;
  return rval + time;
}

//------------------------------------------------------------------------------
double ship_c::compute_distance( const std::vector<double>& pred_state, const std::vector<double>& prey_state ) {
  // get the predator and prey positions
  Ravelin::Vector3d pred_x( pred_state[0], pred_state[1], pred_state[2] );
  Ravelin::Vector3d prey_x( prey_state[0], prey_state[1], prey_state[2] );

  // determine distance to the predator
  Ravelin::Vector3d pp_vec = prey_x - pred_x;
  return pp_vec.norm();
}
//------------------------------------------------------------------------------
// computes commands (forces) for if the ship is a prey
void ship_c::compute_prey_command( const std::vector<double>& pred_state, const std::vector<double>& prey_state, std::vector<double>& prey_u, const double& time, const double& dtime, space_c* space ) {

  Ravelin::Vector3d walk_force( 0.0, 0.0, 0.0 );     //computed force component
  Ravelin::Vector3d walk_torque( 0.0, 0.0, 0.0 );    //computed torque component
  Ravelin::Vector3d space_force( 0.0, 0.0, 0.0 );    // boundary force component
  Ravelin::Vector3d flee( 0.0, 0.0, 0.0 );     // flee force component
  Ravelin::Vector3d weighted_force( 0.0, 0.0, 0.0 ); // final weighted force 
  Ravelin::Vector3d weighted_torque( 0.0, 0.0, 0.0 );// final weighted torque 

  // sanity checks
  assert( prey_u.size() == 6 );
  assert( prey_state.size() == 13 );
  assert( pred_state.size() == 13 );

  // get the predator and prey positions
  Ravelin::Vector3d pred_x( pred_state[0], pred_state[1], pred_state[2] );
  Ravelin::Vector3d prey_x( prey_state[0], prey_state[1], prey_state[2] );

  // determine distance to the predator
  Ravelin::Vector3d pp_vec = prey_x - pred_x;

  double dist = compute_distance( pred_state, prey_state );

  // if outside of flee triggering radius compute a (semi) random walk
  if( dist > FLEE_DISTANCE ) {

    // compute the seed for the random number generator
    unsigned long seed = compute_prey_command_seed( time, dtime );

    // TODO: Determine if rng needs to be warmed up to avoid system calls

    // setup the random number generator
    gsl_rng * r =  gsl_rng_alloc( gsl_rng_default );
    if( r == NULL ) std::cout << "failed to initialize rng\n";
    gsl_rng_set( r, seed );

    // generate command values from gaussian
    const double SIGMA = GAUSSIAN_STDDEV;
    const double MU = GAUSSIAN_MEAN;
    for( unsigned i = 0; i < 3; i++ ) {
      walk_force[i] = gsl_ran_gaussian( r, SIGMA ) + MU;
      walk_torque[i] = 0.0;
    }

    // free the random number generator
    gsl_rng_free( r );

    // NOTE: make sure command is limited to max force/torque
    // if the force exceeds the maximum force, limit the force to the max
    //if( walk_force.norm() > PREY_MAX_FORCE ) {
    //  walk_force = Ravelin::Vector3d::normalize( walk_force ) * PREY_MAX_FORCE;
    //}

    // if the torque exceeds the maximum torque, limit the torque to the max
    //if( walk_torque.norm() > PREY_MAX_TORQUE ) { 
    //  walk_torque = Ravelin::Vector3d::normalize( walk_torque ) * PREY_MAX_TORQUE;
    //}
  }

  // flee behavior
  if( dist <= FLEE_DISTANCE ) {

    flee = Ravelin::Vector3d::normalize( pp_vec ) * flee_force( PREDATOR_REPULSION_FACTOR, dist ) * PREY_FLEE_FORCE_WEIGHT;
  }

  // sum all component forces 
  weighted_force = walk_force + flee;
  weighted_torque = walk_torque;

  // apply weight if necessary
  if( weighted_force.norm() > PREY_MAX_FORCE )
    weighted_force = Ravelin::Vector3d::normalize( weighted_force ) * PREY_MAX_FORCE;
  if( weighted_torque.norm() > PREY_MAX_TORQUE ) 
    weighted_torque = Ravelin::Vector3d::normalize( weighted_torque ) * PREY_MAX_TORQUE;

  //--

  // get the boundary force
  space_force = boundary_force( space, Ravelin::Vector3d(prey_state[0],prey_state[1],prey_state[2]), Ravelin::Vector3d(prey_state[7],prey_state[8],prey_state[9]) );
  space_force *= PREY_BOUNDARY_FORCE_WEIGHT;

  // get the drag force
  boost::shared_ptr<Ravelin::Pose3d> P( new Ravelin::Pose3d() );
  P->x = Ravelin::Origin3d(prey_state[0], prey_state[1], prey_state[2]);
  P->q.set_identity();
  Ravelin::SVelocityd v(prey_state[10], prey_state[11], prey_state[12], prey_state[7],prey_state[8],prey_state[9]);
  Ravelin::SForced drag_forced = drag_force( v, P );
  
  Ravelin::Vector3d drag_force = drag_forced.get_force();
  Ravelin::Vector3d drag_torque = drag_forced.get_torque();
  boost::shared_ptr<Ravelin::Pose3d> GLOBAL;

  weighted_force += space_force;
  weighted_force += Ravelin::Pose3d::transform_vector( GLOBAL, drag_force );
  weighted_torque += Ravelin::Pose3d::transform_vector( GLOBAL, drag_torque );

  // update the control vector
  for( unsigned i = 0; i < 3; i++ ) 
    prey_u[i] = weighted_force[i];
  for( unsigned i = 0; i < 3; i++ ) 
    prey_u[i+3] = weighted_torque[i];
}

//------------------------------------------------------------------------------
/// Computes a force to repel the ship away from the spatial boundary if the 
/// ship gets too close
Ravelin::Vector3d ship_c::boundary_force( space_c* space, const Ravelin::Vector3d& pos, const Ravelin::Vector3d& vel ) {

  // compute the repulsive force from the boundary of space (bos)
  Ravelin::Vector3d f(0,0,0);
  for( unsigned i = 0; i < space->planes.size(); i++ ) {

    // plane is defined as ax + by + cz + d = 0
    // compute the signed distance from the i'th plane to the ship
    double sdist = space->planes[i].signed_dist(pos);

    double repulsion = repulsive_force( BOUNDARY_REPULSION_FACTOR, sdist);
    f += space->planes[i].normal * repulsion;
  }
  return f; 
}

//------------------------------------------------------------------------------
/// Computes a force to dissipate energy from the ship 
Ravelin::SForced ship_c::drag_force( Ravelin::SVelocityd& v, boost::shared_ptr<Ravelin::Pose3d> P_CoM ) {
  const double NEAR_ZERO = 1e-6;
  Ravelin::Vector3d lv = v.get_linear();
  Ravelin::Vector3d av = v.get_angular();
  double lv_sq = lv.norm_sq();
  double av_sq = av.norm_sq();
  if (lv_sq > NEAR_ZERO)
    lv /= std::sqrt(lv_sq);
  if (av_sq > NEAR_ZERO)
    av /= std::sqrt(av_sq);
  Ravelin::SForced f(P_CoM);
  f.set_force( DRAG_COEFFICIENT * -lv * lv_sq );
  f.set_torque( DRAG_COEFFICIENT * -av * av_sq );
  return f;
}

//------------------------------------------------------------------------------
// computes a repulsive force for a given distance
double ship_c::repulsive_force( double repulsion_factor, double dist ) {

  if (dist - BOUNDARY_FIELD_DIST >= 0.0)
    return 0.0;
  else
    return -(dist - BOUNDARY_FIELD_DIST) * BOUNDARY_GAIN;
}

//------------------------------------------------------------------------------
// computes a repulsive force for a given distance
double ship_c::flee_force( double repulsion_factor, double dist ) {
  // logarithmic function will produce negative values for inputs in [0,1] and
  // positive values for inputs > 1
  return (std::log(repulsion_factor*dist) <= 0.0) ? -std::log(repulsion_factor*dist) : 0.0;
}

//-----------------------------------------------------------------------------
// Feedback Controller
//-----------------------------------------------------------------------------
/// Computes a feedback command using PD control
void ship_c::compute_feedback( const std::vector<double>& x_current, std::vector<double>& x_desired, const std::vector<double>& u_current, std::vector<double>& u_feedback ) {

  std::vector<double> deltax( 13 );

  compute_desired_state( u_current, x_current, x_desired );

  //printf( "pred: x_desired[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]\n", x_desired[0], x_desired[1], x_desired[2], x_desired[3], x_desired[4], x_desired[5], x_desired[6], x_desired[7], x_desired[8], x_desired[9], x_desired[10], x_desired[11], x_desired[12] );

  // get quaternion from current state
  Ravelin::Quatd e( x_current[3], x_current[4], x_current[5], x_current[6]);

  for( unsigned i = 0; i < 13; i++ )
    deltax[i] = x_desired[i] - x_current[i];

  const double K_pl = FEEDBACK_GAIN_PROPORTIONAL_POSITION;
  const double K_vl = FEEDBACK_GAIN_DERIVATIVE_POSITION;

  const double K_pr = FEEDBACK_GAIN_PROPORTIONAL_ROTATION;
  const double K_vr = FEEDBACK_GAIN_DERIVATIVE_ROTATION;

  Ravelin::Vector3d pos_err( deltax[0], deltax[1], deltax[2] );
  Ravelin::Vector3d vel_err( deltax[7], deltax[8], deltax[9] );

  Ravelin::Quatd rot_err( deltax[3], deltax[4], deltax[5], deltax[6] );
  Ravelin::Vector3d rotv_err( deltax[10], deltax[11], deltax[12] );

  Ravelin::Vector3d F = K_pl * pos_err + K_vl * vel_err;

  // convert differential quaternion to angular velocity
  Ravelin::Vector3d rotp = Ravelin::Quatd::to_omega(e, rot_err);

  // multiply by positional gain
  rotp *= K_pr;

  // compute the derivative error of orientation
  Ravelin::Vector3d rotd = rotv_err;

  // multiply by derivative gain
  rotd *= K_vr;

  // compute the combined control
  Ravelin::Vector3d tau = rotp + rotd;

  // build the command
  for( unsigned i = 0; i < 3; i++ )
    u_feedback[i] = F[i];
  for( unsigned i = 0; i < 3; i++ )
    u_feedback[i+3] = tau[i];

}

//-----------------------------------------------------------------------------
/// Plans motion for the ship using Evan's brilliant planner
bool ship_c::compute_predator_plan( const std::vector<double>& pred_q, const std::vector<double>& prey_q, std::vector< std::vector<double> >& us, std::vector<double>& durations, unsigned& control_count ) {

  // mark the start time for planning
  timestamp_t ts = generate_timestamp();  
    
  // setup the cpu speed for planning
  cpu_speed_t cpu = 2799844000;

  // init the control count
  control_count = 0;

  // setup predator/prey stuff
  static std::vector<double> new_pred_q(13), new_prey_q(13);
  static std::vector<double> last_pred_q(13), last_prey_q(13);
  last_pred_q = pred_q;
  last_prey_q = prey_q; 

  // get stuff we need for integrating, getting controls
  // space
  space_c* space = _space.get();

  // time is passed through to the prey's command function for generating
  // seeds
  double time = this->time;

  // inertias
  const Ravelin::SpatialRBInertiad& J = _inertial;

  // iterate
  while (true) {

    // only generate commands if possible
    if (control_count < PLANNER_MAX_CONTROL_COUNT) {

      // integrate the prey state model forward in time
      integrate_prey(last_prey_q, last_pred_q, J, time, space, PLANNER_STEP_SIZE, new_prey_q);

      // get the predator control to take us near the new prey state
      get_predator_control(last_pred_q, new_prey_q, J, space, PLANNER_STEP_SIZE, us[control_count]);
      durations[control_count] = PLANNER_STEP_SIZE;

      // integrate the predator state model forward in time
      integrate_predator(last_pred_q, us[control_count], J, space, PLANNER_STEP_SIZE, new_pred_q);

      // setup last prey and last predator states
      last_prey_q = new_prey_q;
      last_pred_q = new_pred_q;

      // update control count
      control_count++;
    }

    // update the elapsed planning time and check whether too much time
    // has elapsed
    timestamp_t te = generate_timestamp();  
    double s = cycles_to_seconds(te-ts, cpu);
    if (s > PLANNER_MAX_PLANNING_TIME)
      break;
  }

  // always return true
  return true;
}

//-----------------------------------------------------------------------------
/// Gets the command to drive the predator toward the prey
void ship_c::get_predator_control(const std::vector<double>& last_pred_q, const std::vector<double>& new_prey_q, const Ravelin::SpatialRBInertiad& predJ, space_c* space, double DT, std::vector<double>& u)
{
  // generate differential vector
  static std::vector<double> dq(13);

  // generate differential vector
  for( unsigned i = 0; i < 13; i++ )
    dq[i] = new_prey_q[i] - last_pred_q[i];

  // update dq to update position
  dq[7] += dq[0];
  dq[8] += dq[1];
  dq[9] += dq[2];

  // update dq to update velocity components
  Ravelin::Quatd edot(dq[3], dq[4], dq[5], dq[6]);
  Ravelin::Quatd e(last_pred_q[3], last_pred_q[4], last_pred_q[5], last_pred_q[6]);
  Ravelin::Vector3d omega = Ravelin::Quatd::to_omega(e, edot);
  dq[10] += omega[0];
  dq[11] += omega[1];
  dq[12] += omega[2];

  // call inverse dynamics to generate a command
  u.resize(6);
  inv_dyn(space, last_pred_q, dq, u, predJ);
}

//-----------------------------------------------------------------------------
/// Integrates the predator forward in time
void ship_c::integrate_predator(const std::vector<double>& q, const std::vector<double>& u, const Ravelin::SpatialRBInertiad& J, space_c* space, double DT, std::vector<double>& qnew)
{
  // q(0:2) [position], q(3:6) [orientation], q(7:9) [linear velocity],
  // q(10:12) [angular velocity]
  static std::vector<double> dq(13);

  // setup q
  qnew = q;

  // normalize quaternion just to be shure
  renormalize_state_quat(qnew);

  // evaluate the ODE
  ode(space, q, u, dq, J); 

  // do semi-implicit integration
  // first update velocity components
  for( unsigned i = 7; i < 13; i++ ) 
    qnew[i] += DT*dq[i];

  // update linear components
  qnew[0] += DT*qnew[7];
  qnew[1] += DT*qnew[8];
  qnew[2] += DT*qnew[9];

  // convert the angular velocities to quaternion time derivatives 
  Ravelin::Quatd e( q[3], q[4], q[5], q[6] );
  Ravelin::Vector3d omega ( qnew[10], qnew[11], qnew[12] );
  Ravelin::Quatd edot = Ravelin::Quatd::deriv( e, omega );

  // update the orientation components using the angular velocity and
  // Euler integration
  qnew[3] += DT*edot[0];
  qnew[4] += DT*edot[1];
  qnew[5] += DT*edot[2];
  qnew[6] += DT*edot[3];

  // renormalize quaternion
  renormalize_state_quat(qnew);
} 

//-----------------------------------------------------------------------------
/// Integrates the prey forward in time
void ship_c::integrate_prey(const std::vector<double>& prey_q, const std::vector<double>& pred_q, const Ravelin::SpatialRBInertiad& J, double time, space_c* space, double DT, std::vector<double>& qnew)
{
  // q(0:2) [position], q(3:6) [orientation], q(7:9) [linear velocity],
  // q(10:12) [angular velocity]
  qnew = prey_q;

  // normalize quaternion just to be shure
  renormalize_state_quat(qnew);

  // get the prey command
  static std::vector<double> u(6);
  double time_input = compute_predator_time_randval(time, DT);
  compute_prey_command(pred_q, prey_q, u, time_input, DT, space);

  // evaluate the ODE
  static std::vector<double> dq(13);
  ode(space, prey_q, u, dq, J); 

  // do semi-implicit integration
  // first update velocity components
  for( unsigned i = 7; i < 13; i++ ) 
    qnew[i] += DT*dq[i];

  // update linear components
  qnew[0] += DT*qnew[7];
  qnew[1] += DT*qnew[8];
  qnew[2] += DT*qnew[9];

  // convert the angular velocities to quaternion time derivatives 
  Ravelin::Quatd e( prey_q[3], prey_q[4], prey_q[5], prey_q[6] );
  Ravelin::Vector3d omega ( qnew[10], qnew[11], qnew[12] );
  Ravelin::Quatd edot = Ravelin::Quatd::deriv( e, omega );

  // update the orientation components using the angular velocity and
  // Euler integration
  qnew[3] += DT*edot[0];
  qnew[4] += DT*edot[1];
  qnew[5] += DT*edot[2];
  qnew[6] += DT*edot[3];

  // renormalize quaternion
  renormalize_state_quat(qnew);
}

//-----------------------------------------------------------------------------
/// Plans motion for the ship using an rrt planner
/*
bool ship_c::compute_predator_plan( const std::vector<double>& pred_q, const std::vector<double>& prey_q, std::vector< std::vector<double> >& us, std::vector<double>& durations, unsigned& control_count ) {

  ompl::base::StateSpacePtr sspace( new ompl::base::RealVectorStateSpace( 26 ) );
  _statespace = sspace.get();

  // Define bounds for the state space
  ompl::base::RealVectorBounds bounds( 26 );
  // Set Bounds
  for( unsigned i = 0; i < 3; i++ )  {
    // x, y, z extens [predator]
    bounds.setLow(i, -_space->bounds.extens[i]);
    bounds.setHigh(i, _space->bounds.extens[i]);
    // x, y, z extens [prey]
    unsigned j = i + 13;
    bounds.setLow(j, -_space->bounds.extens[i]);
    bounds.setHigh(j, _space->bounds.extens[i]);
  }
  for( unsigned i = 3; i < 7; i++ ) {
    // quaternion extens [predator]
    bounds.setLow(i, -1.00001);
    bounds.setHigh(i, 1.00001);
    // quaternion extens [prey]
    unsigned j = i + 13;
    bounds.setLow(j, -1.00001);
    bounds.setHigh(j, 1.00001);
  }

  // set the linear bounds
  for( unsigned i=7; i< 10; i++) {
    bounds.setLow(i, std::min(-PLANNER_MAX_DERIVATIVE_LIN, pred_q[i]) - PLANNER_DERIVATIVE_LIN_TOL);
    bounds.setLow(i+13, std::min(-PLANNER_MAX_DERIVATIVE_LIN, prey_q[i]) - PLANNER_DERIVATIVE_LIN_TOL);
    bounds.setHigh(i, std::max(PLANNER_MAX_DERIVATIVE_LIN, pred_q[i]) + PLANNER_DERIVATIVE_LIN_TOL);
    bounds.setHigh(i+13, std::max(PLANNER_MAX_DERIVATIVE_LIN, prey_q[i]) + PLANNER_DERIVATIVE_LIN_TOL);
  }

  // set the angular bounds
  for( unsigned i=10; i< 13; i++) {
    bounds.setLow(i, std::min(-PLANNER_MAX_DERIVATIVE_ANG, pred_q[i]) - PLANNER_DERIVATIVE_ANG_TOL);
    bounds.setLow(i+13, std::min(-PLANNER_MAX_DERIVATIVE_ANG, prey_q[i]) - PLANNER_DERIVATIVE_ANG_TOL);
    bounds.setHigh(i, std::max(PLANNER_MAX_DERIVATIVE_ANG, pred_q[i]) + PLANNER_DERIVATIVE_ANG_TOL);
    bounds.setHigh(i+13, std::max(PLANNER_MAX_DERIVATIVE_ANG, prey_q[i]) + PLANNER_DERIVATIVE_ANG_TOL);
  }

  sspace->as<ompl::base::RealVectorStateSpace>()->setBounds( bounds );

  // create a control space
  ompl::control::ControlSpacePtr cspace( new control_space_c( sspace ) );

  // set the bounds for the control space
  ompl::base::RealVectorBounds cbounds( 6 );
  for( unsigned i = 0; i < 3; i++ )  {
    // x, y, z extens
    cbounds.setLow(i, -PLANNER_MAX_FORCE);
    cbounds.setHigh(i, PLANNER_MAX_FORCE);
    // x, y, z extens
    unsigned j = i + 3;
    cbounds.setLow(i, -PLANNER_MAX_TORQUE);
    cbounds.setHigh(i, PLANNER_MAX_TORQUE);
  }
  cspace->as<control_space_c>()->setBounds( cbounds );

  // define a simple setup class
  ompl::control::SimpleSetup ss( cspace );

  ompl::control::SpaceInformationPtr si( ss.getSpaceInformation() );
  si->setDirectedControlSamplerAllocator( boost::bind(&ship_c::allocate_pp_control_sampler, this, _1) );

  ompl::base::ProblemDefinition pdef( si );

  /// set state validity checking for this space
  ss.setStateValidityChecker( boost::bind(&ship_c::is_state_valid, this, si.get(), _1 ) );

  /// set the propagation routine for this space
  ss.setStatePropagator( ompl::control::StatePropagatorPtr( new pp_state_propagator_c( si, _space.get(), this, adversary.get() ) ) );

  // form a compound predator, prey state
  std::vector<double> pp_q( 26 );

  pp_vector_from_ship_vectors( pred_q, prey_q, pp_q );

   /// create a start state
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start( sspace );
  pp_vector_to_ompl_state( pp_q, _statespace, start.get() );

  /// create a  goal state
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal( sspace );
  boost::shared_ptr <pp_goal_c> pgoal( new pp_goal_c( si ) );

  /// set the start and goal
  ss.setStartState( start );
  ss.setGoal( pgoal );

  ompl::control::RRT* rrt = new ompl::control::RRT(si);
  rrt->setGoalBias( PLANNER_GOAL_BIAS );
  ompl::base::PlannerPtr planner( rrt );

  ss.setPlanner( planner );

  ss.getSpaceInformation()->setPropagationStepSize( PLANNER_STEP_SIZE );
  ss.setup();

  static_cast<pp_state_propagator_c*>(ss.getStatePropagator().get())->setIntegrationTimeStep(ss.getSpaceInformation()->getPropagationStepSize());

  ompl::base::PlannerStatus solved;
  solved = ss.solve( PLANNER_MAX_PLANNING_TIME );

  // if it failed to solve, bail out
  if (solved != ompl::base::PlannerStatus::APPROXIMATE_SOLUTION &&
      solved != ompl::base::PlannerStatus::EXACT_SOLUTION ) return false;

  unsigned count = ss.getSolutionPath().getControlCount();
  control_count = count < PLANNER_MAX_CONTROL_COUNT ? count : PLANNER_MAX_CONTROL_COUNT;

  // EMD: now we're going to cut down our plan; we'll do this by looking
  // to see when the predator gets farther from the previous state after
  // applying a control

  // get the distance from the goal at the initial state

  // get the states, the controls, and the control durations
  std::vector<ompl::base::State*>& state_vec = ss.getSolutionPath().getStates();
  std::vector<ompl::control::Control*>& controls = ss.getSolutionPath().getControls();
  std::vector<double>& control_durations = ss.getSolutionPath().getControlDurations();

  for (unsigned i=1; i< state_vec.size(); i++) {
    if (pgoal->distanceGoal(state_vec[i]) > pgoal->distanceGoal(state_vec[0])) {
      if (i == 1)
        return false;
      while (state_vec.size() > i)
        state_vec.pop_back();
      while (controls.size() > i-1) {
        controls.pop_back();
        control_durations.pop_back();
        control_count = i-1;
      }
      break;
    }
  }

  for( unsigned i = 0; i < control_count; i++ ) {
    ompl_control_to_vector( controls[i], us[i] );
    durations[i] = control_durations[i]; 
  }

  return true;
}
*/

//-----------------------------------------------------------------------------
/// Determines whether the a state is a valid configuration for a ship
bool ship_c::is_state_valid(const ompl::control::SpaceInformation *si, const ompl::base::State *state) {

  std::vector<double> values( 13 );
  from_state( si->getStateSpace().get(), state, values );
 
  aabb_c bb = aabb( values );
  aabb_c obstacle;
  
  if( intersects_world_bounds( bb ) )
    return false;
  //if( intersects_any_obstacle( bb, obstacle ) )
  //  return false;

  return true;
}

//-----------------------------------------------------------------------------
/// Computes the differential equations of motion for a ship when called by
/// the state propagator
void ship_c::operator()( const ompl::base::State* ompl_q, const ompl::control::Control* ompl_u, std::vector<double>& dq, const Ravelin::SpatialRBInertiad& inertial ) const {

  std::vector<double> q( 13 );
  from_state( _statespace, ompl_q, q );

  std::vector<double> u( 6 );
  const double *_u = ompl_u->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  for( unsigned i = 0; i < 6; i++ ) {
    u[i] = _u[i];
  }

  ode( _space.get(), q, u, dq, inertial );
}

//-----------------------------------------------------------------------------
/// Update the state of the ship after integration (Called by state propagator)
void ship_c::update( ompl::base::State* ompl_q, const std::vector<double>& dq ) const {
  std::vector<double> q( 13 );

  from_state( _statespace, ompl_q, q );
  for( unsigned i=0; i< 13; i++ )
    q[i] += dq[i];

  renormalize_state_quat( q );

  to_state( _statespace, q, ompl_q );
  _statespace->enforceBounds( ompl_q );

}

//------------------------------------------------------------------------------
/// Allocates a predator/prey scenario control sampler
ompl::control::DirectedControlSamplerPtr ship_c::allocate_pp_control_sampler( const ompl::control::SpaceInformation* si ) {
  int k = 1;
  return ompl::control::DirectedControlSamplerPtr( new pp_control_sampler_c( si, _space.get(), this, adversary.get(), k ) );
}

//------------------------------------------------------------------------------

