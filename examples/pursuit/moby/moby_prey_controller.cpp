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

#include <Ravelin/VectorNd.h>

#include <Moby/Simulator.h>

#include "../moby_space.h"
#include "../moby_ship.h"

#include <tcs/log.h>
#include "../experiment.h"
#include <tcs/cpu.h>
#include <tcs/time.h>

//-----------------------------------------------------------------------------

#define LOG_CAPACITY 1048576  // 1MB
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

std::vector<double> prey_state;
std::vector<double> pred_state;
std::vector<double> prey_control;

//-----------------------------------------------------------------------------
typedef double Real;
#define PI 3.14159265359
//-----------------------------------------------------------------------------

extern Real STEP_SIZE;

//-----------------------------------------------------------------------------

static log_c info;
static char infostr[512];

//-----------------------------------------------------------------------------

static bool captured;

//-----------------------------------------------------------------------------

log_c timelog_controller;

timestamp_t controller_activation_ts;
timestamp_t controller_yield_ts;
timestamp_t controller_waitbegin_ts;

cpu_speed_t cpu_speed;

//-----------------------------------------------------------------------------

/// The main control loop for Moby plugin controller
/// Note: Moby Plugin Code
void control_prey( DynamicBodyPtr prey_dbody, Real time, void* data ) {
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

  // compute control
  space_p space_base = dynamic_pointer_cast<space_c>( space );
  ship_c::compute_prey_command( pred_state, prey_state, prey_control, prey->time, prey->dtime, space_base.get() );

  // reset accumulators
  prey->reset();

  // update control
  prey->write( prey_control );

  controller_yield_ts = generate_timestamp();

  sprintf( infostr, "time[%f], dtime[%f], distance[%f]", time, dtime, dist );
  info.write( infostr );

  sprintf( infostr, ", q[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", prey_state[0], prey_state[1], prey_state[2], prey_state[3], prey_state[4], prey_state[5], prey_state[6], prey_state[7], prey_state[8], prey_state[9], prey_state[10], prey_state[11], prey_state[12] );
  info.write( infostr );

  sprintf( infostr, ", u[%f,%f,%f,%f,%f,%f]\n", prey_control[0], prey_control[1], prey_control[2], prey_control[3], prey_control[4], prey_control[5] );
  info.write( infostr );
  
  if( info.size() >= info.capacity() * 9 / 10 ) 
    info.flush();

  control_exec_time_cycle = controller_yield_ts - controller_activation_ts;
  control_wait_time_cycle = controller_activation_ts - controller_waitbegin_ts;
  control_exec_time_sec = cycles_to_seconds( control_exec_time_cycle, cpu_speed );
  //control_wait_time_sec = cycles_to_seconds( control_wait_time_cycle, cpu_speed );
  control_wait_time_sec = std::max( PREY_CONTROLLER_PERIOD_SEC - control_exec_time_sec, 0.0 );

  // write sim time, sim dtime, plan time, wait time to planner timelog
  sprintf( infostr, "%f %f %f\n", time, control_exec_time_sec, control_wait_time_sec );
  timelog_controller.write( infostr );

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

  captured = false;

  std::string log_name = PREY_CONTROLLER_LOG_FILE;
  info = log_c( log_name.c_str(), true );
  log_c::error_e log_err;
  log_err = info.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(moby/prey_controller.cpp) failed to allocate log\n" );
    printf( "%s", infostr );
  }

  // timelogs
  std::stringstream ss_timelog_controller;
  ss_timelog_controller << "moby_pursuit_prey_controller_time_trial_" << EXPERIMENTAL_SEED ;
  std::string timelog_controller_name = ss_timelog_controller.str();
  timelog_controller = log_c( timelog_controller_name.c_str(), true );
  log_err = timelog_controller.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(moby/prey_controller.cpp) failed to allocate timelog_controller\n" );
    printf( "%s", infostr );
  }

  if( cpu_c::get_speed( cpu_speed, DEFAULT_CPU ) != cpu_c::ERROR_NONE ) {
    sprintf( infostr, "(moby/prey_controller.cpp) init() failed calling cpu_c::get_frequency(cpu_speed,cpu)\nExiting\n" );
    //if( info ) info->write( spstr );
    printf( "%s", infostr );
    //exit( 1 );
  }
  sprintf( infostr, "cpu speed(hz): %llu\n", cpu_speed );
  //if( info ) info->write( spstr );
  printf( "%s", infostr );

  // find the simulator in the configuration map
  if( read_map.find("simulator") == read_map.end() )
    throw std::runtime_error( "moby/prey_controller.cpp:init()- unable to find simulator!" );

  printf( "prey contoller operating speed: %f\n", STEP_SIZE );

  // find the prey body in the configuration map
  Moby::RigidBodyPtr prey_rbody = dynamic_pointer_cast<RigidBody>( read_map.find("prey")->second );
  if( !prey_rbody )
    throw std::runtime_error( "(moby/prey_controller.cpp) init() - unable to cast prey_rbody to type RigidBody" );

  // setup the control function
  prey_rbody->controller = control_prey;

  //-----------------------
  // set up all other reference information drawn from the moby configuration

  space = moby_space_p( new moby_space_c() );
  space->read( read_map );
  prey = space->prey();
  predator = space->predator();

  prey_state.resize( 13 );
  pred_state.resize( 13 );
  prey_control.resize( 6 );

  for( unsigned i = 0; i < 6; i++ )
    prey_control[i] = 0;

  if( suppress_console_io ) {
    printf( "NOTICE: client I/O from prey_controller will be supressed.  I/O using stdout, stdin, and stderr will be forwarded to /dev/null.\n" );

    // map stdin, stdout, stderr to /dev/null
    int fd;
    fd = __open( "/dev/null", O_RDWR );
    dup2(fd,0);
    dup2(fd,1);
    //dup2(fd,2);
  }

  controller_waitbegin_ts = generate_timestamp();
}

} // end extern C

//-----------------------------------------------------------------------------
