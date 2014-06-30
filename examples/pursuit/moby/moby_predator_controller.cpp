/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

controller.cpp
-----------------------------------------------------------------------------*/

#include <assert.h>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>

#include <unistd.h>
#include <sstream>

#include <Ravelin/VectorNd.h>

#include <Moby/Simulator.h>

#include "../moby_space.h"
#include "../moby_ship.h"

#include <tcs/log.h>
#include "../experiment.h"
#include <tcs/cpu.h>
#include <tcs/time.h>

//-----------------------------------------------------------------------------

static bool suppress_console_io = true;

//-----------------------------------------------------------------------------

using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

moby_space_p space;
ship_p prey;
ship_p predator;

//-----------------------------------------------------------------------------

unsigned _plan_count;
unsigned _plan_idx;
double _plan_start;
double _plan_duration;
std::vector<double> _plan_durations;
std::vector<std::vector<double> > _plan_us;
std::vector<double> prey_state;
std::vector<double> pred_state;
std::vector<double> prey_control;
std::vector<double> pred_control;
std::vector<double> pred_desired_state;
std::vector<double> pred_planned_control;
std::vector<double> pred_feedback_control;

static std::vector< std::vector<double> > us;
static std::vector<double> durations;
static unsigned control_count;

//-----------------------------------------------------------------------------
typedef double Real;
#define PI 3.14159265359
//-----------------------------------------------------------------------------

extern Real STEP_SIZE;

//-----------------------------------------------------------------------------

static log_c info;
static log_c summary;
static char infostr[512];

static unsigned planner_calls;
static unsigned planner_failures;

//-----------------------------------------------------------------------------

static bool captured;

//-----------------------------------------------------------------------------

log_c timelog_controller;
log_c timelog_planner;

timestamp_t controller_activation_ts;
timestamp_t controller_yield_ts;
timestamp_t controller_waitbegin_ts;
timestamp_t planner_activation_ts;
timestamp_t planner_yield_ts;
timestamp_t planner_waitbegin_ts;

cpu_speed_t cpu_speed;

//-----------------------------------------------------------------------------
/// The main control loop for Moby plugin controller
/// Note: Moby Plugin Code
void control_predator( DynamicBodyPtr predator_dbody, Real time, void* data ) {
  bool refreshed_plan = false;
  cycle_t plan_exec_time_cycle, plan_wait_time_cycle;
  double plan_exec_time_sec, plan_wait_time_sec;
  cycle_t control_exec_time_cycle, control_wait_time_cycle;
  double control_exec_time_sec, control_wait_time_sec;

  controller_activation_ts = generate_timestamp();

  double dtime = STEP_SIZE;

  // update state
  prey->read( prey_state );
  predator->read( pred_state );
  // update the prey ship structure as necessary
  prey->time = time;
  prey->dtime = dtime; 
  predator->time = time;
  predator->dtime = dtime;

  double dist = ship_c::compute_distance( pred_state, prey_state );
  if( dist < CAPTURE_DISTANCE ) {
    captured = true;
    sprintf( infostr, "[CAPTURED]" );
    info.write( infostr );  
  } else {
    captured = false;
  }

  // see which command to use
  if (time - _plan_start > _plan_duration)
  {
    planner_activation_ts = generate_timestamp();
    // need to formulate a plan
    // planned control
    //bool result = predator->plan_rrt( pred_state, prey_state, pred_planned_control );
    bool failed_to_plan = !predator->compute_predator_plan( pred_state, prey_state, us, durations, control_count );
    if( failed_to_plan ) {
      for( unsigned i = 0; i < 6; i++ )
        pred_planned_control[i] = 0.0;
      planner_failures++;
      _plan_start = -std::numeric_limits<double>::max();
      _plan_duration = 0.0;
      _plan_idx = 0;
      _plan_count = 0;
    } else {
      _plan_start = time;
      _plan_duration = std::accumulate(durations.begin(), durations.end(), 0.0);
      _plan_duration = std::min(_plan_duration, PLANNER_MAX_PLAN_DURATION);
      _plan_durations = durations;
      _plan_us = us;
      _plan_count = durations.size();
      _plan_idx = 0;
      for( unsigned i = 0; i < 6; i++ )
        pred_planned_control[i] = us[0][i];
    }
    planner_calls++;

    refreshed_plan = true;

    planner_yield_ts = generate_timestamp();

    plan_exec_time_cycle = planner_yield_ts - planner_activation_ts;
    plan_wait_time_cycle = planner_activation_ts - planner_waitbegin_ts;
    plan_exec_time_sec = cycles_to_seconds( plan_exec_time_cycle, cpu_speed );
    //plan_wait_time_sec = cycles_to_seconds( plan_wait_time_cycle, cpu_speed );
    plan_wait_time_sec = 0.0;

    // write sim time, plan time, wait time to planner timelog
    sprintf( infostr, "%f %f %f\n", time, plan_exec_time_sec, plan_wait_time_sec );
    timelog_planner.write( infostr );

    planner_waitbegin_ts = generate_timestamp();
  }

  // see which command to use
  double t = -dtime;
  while (t < 0.0 && _plan_idx < _plan_count)
    t += _plan_durations[_plan_idx];
  if (_plan_idx >= _plan_count) {
    for( unsigned i = 0; i < 6; i++ )
      pred_planned_control[i] = 0.0;
  }
  else {
    pred_planned_control = _plan_us[_plan_idx];
  }

  // feedback control
  predator->compute_feedback( pred_state, pred_desired_state, pred_planned_control, pred_feedback_control );

  // compute the boundary force
  Ravelin::Vector3d boundary_force = predator->boundary_force(space.get(), Ravelin::Vector3d(pred_state[0],pred_state[1],pred_state[2]), Ravelin::Vector3d(pred_state[7],pred_state[8],pred_state[9]) );
  boundary_force *= PREY_BOUNDARY_FORCE_WEIGHT;

  // compute the drag
  boost::shared_ptr<Ravelin::Pose3d> P( new Ravelin::Pose3d() );
  P->x = Ravelin::Origin3d(pred_state[0], pred_state[1], pred_state[2]);
  P->q.set_identity();
  Ravelin::SVelocityd v(pred_state[10], pred_state[11], pred_state[12], pred_state[7],pred_state[8],pred_state[9]);
  Ravelin::SForced drag_force = ship_c::drag_force(v , P );
  boost::shared_ptr<Ravelin::Pose3d> GLOBAL;
  Ravelin::SForced drag = Ravelin::Pose3d::transform(GLOBAL, drag_force);
 
  for( unsigned i = 0; i < 6; i++ ) {
    pred_control[i] = pred_planned_control[i] + pred_feedback_control[i];
  }

  // add in the spatial forces
  for (unsigned i=0; i< 3; i++) {
    pred_control[i] += boundary_force[i];
  }

  // add in the drag force
  for (unsigned i=0; i< 6; i++)
    pred_control[i] += drag[i];

  // EMD 5/19/14 -- remove torques
  pred_control[3] = pred_control[4] = pred_control[5] = 0.0;

  // reset accumulators
  predator->reset();

  // update control
  predator->write( pred_control );

  controller_yield_ts = generate_timestamp();

  sprintf( infostr, "%f", time );
  info.write( infostr );

  sprintf( infostr, " %f %f %f %f %f %f %f %f %f %f %f %f %f", pred_state[0], pred_state[1], pred_state[2], pred_state[3], pred_state[4], pred_state[5], pred_state[6], pred_state[7], pred_state[8], pred_state[9], pred_state[10], pred_state[11], pred_state[12] );
  info.write( infostr );

  sprintf( infostr, " %f %f %f %f %f %f %f %f %f %f %f %f %f\n", prey_state[0], prey_state[1], prey_state[2], prey_state[3], prey_state[4], prey_state[5], prey_state[6], prey_state[7], prey_state[8], prey_state[9], prey_state[10], prey_state[11], prey_state[12] );
  info.write( infostr );
/*
  if( failed_to_plan ) {
    sprintf( infostr, "[FAILED] " );
    info.write( infostr );
  }

  sprintf( infostr, "time[%f], dtime[%f], distance[%f]", time, dtime, dist );
  info.write( infostr );

  sprintf( infostr, ", q_prey[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", prey_state[0], prey_state[1], prey_state[2], prey_state[3], prey_state[4], prey_state[5], prey_state[6], prey_state[7], prey_state[8], prey_state[9], prey_state[10], prey_state[11], prey_state[12] );
  info.write( infostr );

  sprintf( infostr, ", q_pred[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", pred_state[0], pred_state[1], pred_state[2], pred_state[3], pred_state[4], pred_state[5], pred_state[6], pred_state[7], pred_state[8], pred_state[9], pred_state[10], pred_state[11], pred_state[12] );
  info.write( infostr );

  sprintf( infostr, ", u_plan[%f,%f,%f,%f,%f,%f]", pred_planned_control[0], pred_planned_control[1], pred_planned_control[2], pred_planned_control[3], pred_planned_control[4], pred_planned_control[5] );
  info.write( infostr );
  
  sprintf( infostr, ", u_fb[%f,%f,%f,%f,%f,%f]", pred_feedback_control[0], pred_feedback_control[1], pred_feedback_control[2], pred_feedback_control[3], pred_feedback_control[4], pred_feedback_control[5] );
  info.write( infostr );
  
  sprintf( infostr, ", u[%f,%f,%f,%f,%f,%f]\n", pred_control[0], pred_control[1], pred_control[2], pred_control[3], pred_control[4], pred_control[5] );
  info.write( infostr );
  
  if( info.size() >= info.capacity() * 9 / 10 ) 
    info.flush();
*/

  if( info.size() >= info.capacity() * 9 / 10 || time + dtime >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
    info.flush();
  }
  if( summary.size() >= info.capacity() * 9 / 10 || time + dtime >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
    summary.flush();
  }
 
  if( time > EXPERIMENTAL_TRIAL_DURATION_SEC ) {
    sprintf( infostr, "planner_calls[%u], planner_failures[%u], failure_percent[%f]\n", planner_calls, planner_failures, ((float)planner_failures/(float)planner_calls)*100.0);
    summary.write( infostr );
    summary.flush();
    exit( 0 );
  }

  control_exec_time_cycle = controller_yield_ts - controller_activation_ts;
  if( refreshed_plan )
    control_exec_time_cycle -= (planner_waitbegin_ts - planner_activation_ts);

  control_wait_time_cycle = controller_activation_ts - controller_waitbegin_ts;
  if( refreshed_plan )
    control_wait_time_cycle += (planner_yield_ts - planner_activation_ts);

  control_exec_time_sec = cycles_to_seconds( control_exec_time_cycle, cpu_speed );
  //control_wait_time_sec = cycles_to_seconds( control_wait_time_cycle, cpu_speed );
  control_wait_time_sec = std::max( PREDATOR_CONTROLLER_PERIOD_SEC - control_exec_time_sec, 0.0 );

  // write sim time, plan time, wait time to planner timelog
  sprintf( infostr, "%f %f %f\n", time, control_exec_time_sec, control_wait_time_sec );
  timelog_controller.write( infostr );

  if( timelog_planner.size() >= timelog_planner.capacity() * 9 / 10 || time + dtime >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
    timelog_planner.flush();
  }

  if( timelog_controller.size() >= timelog_controller.capacity() * 9 / 10 || time + dtime >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
    timelog_controller.flush();
  }

  controller_waitbegin_ts = generate_timestamp();
}

//-----------------------------------------------------------------------------
// Mody Plugin Interface
//-----------------------------------------------------------------------------

// plugin must be "extern C"
extern "C" {

/**
    Interface to compile as a Moby Plugin
/// Note: Moby Plugin Code
*/
void init( void* separator, const std::map<std::string, BasePtr>& read_map, Real time ) {

  _plan_start = -std::numeric_limits<double>::max();
  _plan_duration = 0.0;
  captured = false;
  planner_calls = 0;
  planner_failures = 0;

  log_c::error_e log_err;

  // experimental data log
  std::stringstream ss_info;
  ss_info << "moby_pursuit_trial_" << EXPERIMENTAL_SEED;
  std::string log_name = ss_info.str();
  info = log_c( log_name.c_str(), true );
  log_err = info.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(moby/predator_controller.cpp) failed to allocate log\n" );
    printf( "%s", infostr );
  }
  sprintf( infostr, "time pred_q[13] prey_q[13]\n" );
  info.write( infostr );

  // summary log
  std::stringstream ss_summary;
  ss_summary << "moby_pursuit_summary_" << EXPERIMENTAL_SEED;
  std::string summary_name = ss_summary.str();
  summary = log_c( summary_name.c_str(), true );
  log_err = summary.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(moby/predator_controller.cpp) failed to allocate summary\n" );
    printf( "%s", infostr );
  }

  // timelogs
  std::stringstream ss_timelog_controller;
  ss_timelog_controller << "moby_pursuit_pred_controller_time_trial_" << EXPERIMENTAL_SEED ;
  std::string timelog_controller_name = ss_timelog_controller.str();
  timelog_controller = log_c( timelog_controller_name.c_str(), true );
  log_err = timelog_controller.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(moby/predator_controller.cpp) failed to allocate timelog_controller\n" );
    printf( "%s", infostr );
  }

  std::stringstream ss_timelog_planner;
  ss_timelog_planner << "moby_pursuit_pred_planner_time_trial_" << EXPERIMENTAL_SEED ;
  std::string timelog_planner_name = ss_timelog_planner.str();
  timelog_planner = log_c( timelog_planner_name.c_str(), true );
  log_err = timelog_planner.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(moby/predator_controller.cpp) failed to allocate timelog_planner\n" );
    printf( "%s", infostr );
  }

  if( cpu_c::get_speed( cpu_speed, DEFAULT_CPU ) != cpu_c::ERROR_NONE ) {
    sprintf( infostr, "(moby/predator_controller.cpp) init() failed calling cpu_c::get_frequency(cpu_speed,cpu)\nExiting\n" );
    //if( info ) info->write( spstr );
    printf( "%s", infostr );
    //exit( 1 );
  }
  sprintf( infostr, "cpu speed(hz): %llu\n", cpu_speed );
  //if( info ) info->write( spstr );
  printf( "%s", infostr );

  // find the simulator in the configuration map
  if( read_map.find("simulator") == read_map.end() )
    throw std::runtime_error( "moby/predator_controller.cpp:init()- unable to find simulator!" );

  printf( "predator contoller operating speed: %f\n", STEP_SIZE );

  // find the predator body in the configuration map
  Moby::RigidBodyPtr predator_rbody = dynamic_pointer_cast<RigidBody>( read_map.find("predator")->second );
  if( !predator_rbody )
    throw std::runtime_error( "(moby/prey_controller.cpp) init() - unable to cast predator_rbody to type RigidBody" );

  // setup the control function
  predator_rbody->controller = control_predator;

  //-----------------------
  // set up all other reference information drawn from the moby configuration

  space = moby_space_p( new moby_space_c() );
  space->read( read_map );
  prey = space->prey();
  predator = space->predator();

  prey_state.resize( 13 );
  pred_state.resize( 13 );
  prey_control.resize( 6 );
  pred_control.resize( 6 );
  pred_desired_state.resize( 13 );
  pred_planned_control.resize( 6 );
  pred_feedback_control.resize( 6 );

  for( unsigned i = 0; i < 6; i++ ) {
    pred_control[i] = 0;
    pred_planned_control[i] = 0;
  }

  // size the plan vectors
  us.resize( MAX_PLANNER_CONTROLS ); 
  for( unsigned i = 0; i < MAX_PLANNER_CONTROLS; i++ )
    us[i].resize( 6 );
  durations.resize( MAX_PLANNER_CONTROLS );

  // initialize the other variable 
  control_count = 0; 

  if( suppress_console_io ) {
    printf( "NOTICE: client I/O from predator_controller will be supressed.  I/O using stdout, stdin, and stderr will be forwarded to /dev/null.\n" );

    // map stdin, stdout, stderr to /dev/null
    int fd;
    fd = __open( "/dev/null", O_RDWR );
    dup2(fd,0);
    dup2(fd,1);
    //dup2(fd,2);
  }

  controller_waitbegin_ts = generate_timestamp();
  planner_waitbegin_ts = controller_waitbegin_ts;
}

} // end extern C

//-----------------------------------------------------------------------------
