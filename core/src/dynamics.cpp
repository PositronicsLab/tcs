#include <tcs/dynamics.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <sstream>

// TODO: the following reference must be changed
//#include "experiment/experiment.h"
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

/*
//-----------------------------------------------------------------------------
dynamics_c::dynamics_c( void ) {
  
}
*/
//-----------------------------------------------------------------------------
dynamics_c::dynamics_c( const char* name, const timesink_p& owner, const cpu_speed_t& cpu_speed, const char* plugin_file, int log_index ) : 
  timesink_c( name, owner, scheduler_c::PROGRESS )
{
  assert( plugin_file != NULL );

//  sprintf( this->name, "%s", name );
  this->owner = owner;
  _info = info;
  _seeded = false;

  temporal_progress = 0;
  computational_progress = 0;

  _cpu_speed = cpu_speed;
  _simulated = true;

  _step_size_cycles = 0;  // undefined.
  _step_size_realtime = 0;  // undefined.

  // initialize the buffer
  // - header
  buffer.t = 0;
  buffer.dt = 0;

  // - state -
  buffer.prey_state.ts = 0;
  buffer.pred_state.ts = 0;
  for( unsigned i = 0; i < 6; i++ ) {
    buffer.prey_state.q[i] = 0;
    buffer.pred_state.q[i] = 0;
  }
  buffer.prey_state.q[6] = 1;  // quaternion w term
  buffer.pred_state.q[6] = 1;  // quaternion w term
  for( unsigned i = 0; i < 6; i++ ) {
    buffer.prey_state.dq[i] = 0;
    buffer.pred_state.dq[i] = 0;
  }

  // - control -
  buffer.prey_control.ts = 0;
  buffer.pred_control.ts = 0;
  for( unsigned i = 0; i < 6; i++ ) {
    buffer.prey_control.u[i] = 0;
    buffer.pred_control.u[i] = 0;
  }

  // - kinetic energy -
  buffer.prey_ke = 0;
  buffer.pred_ke = 0;

  _filename = std::string( plugin_file );

  std::stringstream ss;
  ss << "tcs_pursuit_trial_" << log_index;

  std::string log_name = ss.str();
  log = log_c( log_name.c_str(), true );
  log_c::error_e log_err;
  log_err = log.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( spstr, "(tcs/dynamics.cpp) failed to allocate log\n" );
    printf( "%s", spstr );
  }
  sprintf( spstr, "time pred_q[13] prey_q[13]\n" );
  log.write( spstr );
}

//-----------------------------------------------------------------------------
dynamics_c::~dynamics_c( void ) {

}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void dynamics_c::dispatch( thread_p& current_thread ) {

  current_thread = shared_from_this();

  if( info != NULL ) {
    sprintf( spstr, "dispatching dynamics: computational_progress[%llu], temporal_progress[%llu], _step_size_cycles[%llu]\n", temporal_progress, computational_progress, _step_size_cycles );
    info->write( spstr );
  }

  if( !_seeded ) {
    _plugin.get_state( &buffer );
    _seeded = true;
  }

  if( _simulated )
    step( _step_size_cycles );
  else
    step();

  // TODO: expose plugin method to query dynamics_time;
  //current_thread->progress = dynamics_time();

  if( info != NULL ) {
    sprintf( spstr, "dynamics idle: computational_progress[%llu], temporal_progress[%llu], _step_size_cycles[%llu]\n", temporal_progress, computational_progress, _step_size_cycles );
    info->write( spstr );
  }

  // TEMPORARY
  //if( info ) info->flush();
}

//-----------------------------------------------------------------------------
void dynamics_c::terminate( void ) {

}

//-----------------------------------------------------------------------------
void dynamics_c::step( const cycle_t& dt ) {
  temporal_progress += dt;
  computational_progress += dt; 
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
dynamics_c::error_e dynamics_c::init( int argc, char* argv[] ) {
  _simulated = false;

  // TODO: more Sanity/Safety checking
  //dynamics_plugin_c::error_e result = _plugin.read( DYNAMICS_PLUGIN );
  dynamics_plugin_c::error_e result = _plugin.read( _filename.c_str() );
  if( result != dynamics_plugin_c::ERROR_NONE ) 
    return ERROR_READ;

  if( !_plugin.init( argc, argv ) )
    return ERROR_INIT;

  _step_size_realtime = _plugin.get_step_size();
  _step_size_cycles = seconds_to_cycles( _step_size_realtime, _cpu_speed );

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
void dynamics_c::shutdown( void ) {

  _plugin.shutdown();

}

//-----------------------------------------------------------------------------
void dynamics_c::log_buffer( void ) {
  if( info != NULL ) {
    sprintf( spstr, " t[%f], dt[%f]", buffer.t, buffer.dt );
    info->write( spstr );

    sprintf( spstr, " prey_state[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", buffer.prey_state.q[0], buffer.prey_state.q[1], buffer.prey_state.q[2], buffer.prey_state.q[3], buffer.prey_state.q[4], buffer.prey_state.q[5], buffer.prey_state.q[6], buffer.prey_state.dq[0], buffer.prey_state.dq[1], buffer.prey_state.dq[2], buffer.prey_state.dq[3], buffer.prey_state.dq[4], buffer.prey_state.dq[5] );
    info->write( spstr );

    sprintf( spstr, ", prey_control[%f,%f,%f,%f,%f,%f]", buffer.prey_control.u[0], buffer.prey_control.u[1], buffer.prey_control.u[2], buffer.prey_control.u[3], buffer.prey_control.u[4], buffer.prey_control.u[5] );
    info->write( spstr );
 
    sprintf( spstr, ", prey_ke[%f]", buffer.prey_ke );
    info->write( spstr );
 
    sprintf( spstr, ", pred_state[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", buffer.pred_state.q[0], buffer.pred_state.q[1], buffer.pred_state.q[2], buffer.pred_state.q[3], buffer.pred_state.q[4], buffer.pred_state.q[5], buffer.pred_state.q[6], buffer.pred_state.dq[0], buffer.pred_state.dq[1], buffer.pred_state.dq[2], buffer.pred_state.dq[3], buffer.pred_state.dq[4], buffer.pred_state.dq[5] );
    info->write( spstr );

    sprintf( spstr, ", pred_control[%f,%f,%f,%f,%f,%f]", buffer.pred_control.u[0], buffer.pred_control.u[1], buffer.pred_control.u[2], buffer.pred_control.u[3], buffer.pred_control.u[4], buffer.pred_control.u[5] );
    info->write( spstr );
 
    sprintf( spstr, ", pred_ke[%f]\n", buffer.pred_ke );
    info->write( spstr );
  }
}
//-----------------------------------------------------------------------------
void dynamics_c::flush_log( void ) {
  log.flush();
}
//-----------------------------------------------------------------------------
void dynamics_c::step( void ) {

  if( info != NULL ) {
    sprintf( spstr, "dynamics buffer state (before step) : " );
    info->write( spstr );
    log_buffer();
  } 

  temporal_progress += _step_size_cycles;
  computational_progress += _step_size_cycles; 
  _plugin.step( &buffer );

  if( info != NULL ) {
    sprintf( spstr, "dynamics buffer state (after step) : " );
    info->write( spstr );
    log_buffer();
  }

  if( info != NULL ) {
    sprintf( spstr, "[DYNAMICS] : " );
    info->write( spstr );
    log_buffer();
  }

  sprintf( spstr, "%f", buffer.t );
  log.write( spstr );

  sprintf( spstr, " %f %f %f %f %f %f %f %f %f %f %f %f %f", buffer.pred_state.q[0], buffer.pred_state.q[1], buffer.pred_state.q[2], buffer.pred_state.q[3], buffer.pred_state.q[4], buffer.pred_state.q[5], buffer.pred_state.q[6], buffer.pred_state.dq[0], buffer.pred_state.dq[1], buffer.pred_state.dq[2], buffer.pred_state.dq[3], buffer.pred_state.dq[4], buffer.pred_state.dq[5] );
  log.write( spstr );

  sprintf( spstr, " %f %f %f %f %f %f %f %f %f %f %f %f %f\n", buffer.prey_state.q[0], buffer.prey_state.q[1], buffer.prey_state.q[2], buffer.prey_state.q[3], buffer.prey_state.q[4], buffer.prey_state.q[5], buffer.prey_state.q[6], buffer.prey_state.dq[0], buffer.prey_state.dq[1], buffer.prey_state.dq[2], buffer.prey_state.dq[3], buffer.prey_state.dq[4], buffer.prey_state.dq[5] );
  log.write( spstr );

  if( log.size() >= log.capacity() * 9 / 10 ) 
    log.flush();
}

//-----------------------------------------------------------------------------
cycle_t dynamics_c::step_size_cycles( void ) {
  return _step_size_cycles;
}

//-----------------------------------------------------------------------------
realtime_t dynamics_c::step_size_realtime( void ) {
  return _step_size_realtime;
}

//-----------------------------------------------------------------------------
