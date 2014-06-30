#include "prey_controller.h"

//-----------------------------------------------------------------------------

GZ_REGISTER_MODEL_PLUGIN( prey_controller_c )


//-----------------------------------------------------------------------------
prey_controller_c::prey_controller_c( void ) { 
  //capture = false;
}

//-----------------------------------------------------------------------------
prey_controller_c::~prey_controller_c( void ) {
    gazebo::event::Events::DisconnectWorldUpdateBegin( _updateConnection );
}

//-----------------------------------------------------------------------------
void prey_controller_c::Load( gazebo::physics::ModelPtr model, sdf::ElementPtr sdf ) {
  _updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind( &prey_controller_c::Update, this ) );

  captured = false;
  log_c::error_e log_err;

  //controller_step = PREY_CONTROLLER_PERIOD_SEC;
/*
  std::string log_name = PREY_CONTROLLER_LOG_FILE;
  info = log_c( log_name.c_str(), true );
  log_c::error_e log_err;
  log_err = info.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(gazebo/prey_controller.cpp) failed to allocate log\n" );
    printf( "%s", infostr );
  }
*/
  // timelogs
  std::stringstream ss_timelog_controller;
  ss_timelog_controller << "gazebo_pursuit_prey_controller_time_trial_" << EXPERIMENTAL_SEED ;
  std::string timelog_controller_name = ss_timelog_controller.str();
  timelog_controller = log_c( timelog_controller_name.c_str(), true );
  log_err = timelog_controller.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(gazebo/prey_controller.cpp) failed to allocate timelog_controller\n" );
    printf( "%s", infostr );
  }

  if( cpu_c::get_speed( cpu_speed, DEFAULT_CPU ) != cpu_c::ERROR_NONE ) {
    sprintf( infostr, "(gazebo/prey_controller.cpp) init() failed calling cpu_c::get_frequency(cpu_speed,cpu)\nExiting\n" );
    //if( info ) info->write( spstr );
    printf( "%s", infostr );
    //exit( 1 );
  }
  sprintf( infostr, "cpu speed(hz): %llu\n", cpu_speed );
  //if( info ) info->write( spstr );
  printf( "%s", infostr );

  _world = model->GetWorld();

  space = gazebo_space_p( new gazebo_space_c() );
  space->read( _world );
  prey = space->prey();
  predator = space->predator();

  prey_state.resize( 13 );
  pred_state.resize( 13 );
  prey_control.resize( 6 );

  for( unsigned i = 0; i < 6; i++ )
    prey_control[i] = 0;

  time_start = _world->GetSimTime().Double();
  time_last = time_start;

  next_activation_time = time_start + prey->controller_step;
  last_activation_time = 0.0;

  printf( "gazebo/prey_controller has initialized\n" );

  //sprintf( infostr, "time,dtime,distance,q_prey_1,q_prey_2,q_prey_3,q_prey_4,q_prey_5,q_prey_6,q_prey_7,q_prey_8,q_prey_9,q_prey_10,q_prey_11,q_prey_12,q_prey_13,u_prey_1,u_prey_2,u_prey_3,u_prey_4,u_prey_5,u_prey_6\n" );
  //info.write( infostr );
  //info.flush( );

  controller_waitbegin_ts = generate_timestamp();
}

//-----------------------------------------------------------------------------
void prey_controller_c::Update( ) {
  cycle_t control_exec_time_cycle, control_wait_time_cycle;
  double control_exec_time_sec, control_wait_time_sec;

  controller_activation_ts = generate_timestamp();

  time = _world->GetSimTime().Double() - time_start;
  dtime = time - time_last;
  integration_step = dtime;
  time_last = time;
/*
  if( time < next_activation_time ) return;
  next_activation_time = time + controller_step;
  last_activation_time = time;
*/
  // update state
  prey->read( prey_state );
  predator->read( pred_state );
  // update the prey ship structure as necessary

  prey->time = time;
  prey->dtime = dtime;
  predator->time = time;
  predator->dtime = dtime;

/*
  prey->time = time;
  prey->dtime = controller_step;
  predator->time = time;
  predator->dtime = controller_step;
*/
  double dist = ship_c::compute_distance( pred_state, prey_state );
/*
  if( dist < CAPTURE_DISTANCE ) {
    captured = true;
    sprintf( infostr, "[CAPTURED]" );
    info.write( infostr );  
  } else {
    captured = false;
  }
*/
  // compute control
  space_p space_base = boost::dynamic_pointer_cast<space_c>( space );
  ship_c::compute_prey_command( pred_state, prey_state, prey_control, prey->time, prey->dtime, space_base.get() );

  // reset accumulators
  prey->reset();

  // update control
  prey->write( prey_control );

  controller_yield_ts = generate_timestamp();



/*
  sprintf( infostr, "%f;%f;%f", time, dtime, dist );
  info.write( infostr );

  sprintf( infostr, ";%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", prey_state[0], prey_state[1], prey_state[2], prey_state[3], prey_state[4], prey_state[5], prey_state[6], prey_state[7], prey_state[8], prey_state[9], prey_state[10], prey_state[11], prey_state[12] );
  info.write( infostr );

  sprintf( infostr, ";%f,%f,%f,%f,%f,%f\n", prey_control[0], prey_control[1], prey_control[2], prey_control[3], prey_control[4], prey_control[5] );
  info.write( infostr );

  if( info.size() >= info.capacity() * 9 / 10 || time + dtime >= EXPERIMENTAL_TRIAL_DURATION_SEC ) 
    info.flush();
  */
  //info.flush();

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
/*
void prey_controller_c::Reset( ) {

}
*/
//-----------------------------------------------------------------------------
