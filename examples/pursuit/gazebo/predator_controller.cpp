#include "predator_controller.h"

#include <sstream>

//-----------------------------------------------------------------------------

GZ_REGISTER_MODEL_PLUGIN( predator_controller_c )


//-----------------------------------------------------------------------------
predator_controller_c::predator_controller_c( void ) { 
  //capture = false;
  _plan_start = -std::numeric_limits<double>::max();
  _plan_duration = 0.0;
}

//-----------------------------------------------------------------------------
predator_controller_c::~predator_controller_c( void ) {
    gazebo::event::Events::DisconnectWorldUpdateBegin( _updateConnection );
}

//-----------------------------------------------------------------------------
void predator_controller_c::Load( gazebo::physics::ModelPtr model, sdf::ElementPtr sdf ) {
  _updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind( &predator_controller_c::Update, this ) );

  captured = false;
  planner_calls = 0;
  planner_failures = 0;

  //controller_step = PREDATOR_CONTROLLER_PERIOD_SEC;

  std::stringstream ss;
  ss << "gazebo_pursuit_trial_" << EXPERIMENTAL_SEED ;

  log_c::error_e log_err;

  // experimental data log
  std::string log_name = ss.str();
  info = log_c( log_name.c_str(), true );
  log_err = info.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(gazebo/predator_controller.cpp) failed to allocate log\n" );
    printf( "%s", infostr );
  }
  sprintf( infostr, "time pred_q[13] prey_q[13]\n" );
  info.write( infostr );

  // summary log
  std::stringstream ss_summary;
  ss_summary << "gazebo_pursuit_summary_" << EXPERIMENTAL_SEED;
  std::string summary_name = ss_summary.str();
  summary = log_c( summary_name.c_str(), true );
  log_err = summary.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(gazebo/predator_controller.cpp) failed to allocate summary\n" );
    printf( "%s", infostr );
  }

  // timelogs
  std::stringstream ss_timelog_controller;
  ss_timelog_controller << "gazebo_pursuit_pred_controller_time_trial_" << EXPERIMENTAL_SEED ;
  std::string timelog_controller_name = ss_timelog_controller.str();
  timelog_controller = log_c( timelog_controller_name.c_str(), true );
  log_err = timelog_controller.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(gazebo/predator_controller.cpp) failed to allocate timelog_controller\n" );
    printf( "%s", infostr );
  }

  std::stringstream ss_timelog_planner;
  ss_timelog_planner << "gazebo_pursuit_pred_planner_time_trial_" << EXPERIMENTAL_SEED ;
  std::string timelog_planner_name = ss_timelog_planner.str();
  timelog_planner = log_c( timelog_planner_name.c_str(), true );
  log_err = timelog_planner.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( infostr, "(gazebo/predator_controller.cpp) failed to allocate timelog_planner\n" );
    printf( "%s", infostr );
  }

  if( cpu_c::get_speed( cpu_speed, DEFAULT_CPU ) != cpu_c::ERROR_NONE ) {
    sprintf( infostr, "(gazebo/predator_controller.cpp) init() failed calling cpu_c::get_frequency(cpu_speed,cpu)\nExiting\n" );
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

  time_start = _world->GetSimTime().Double();
  time_last = time_start;

  next_activation_time = time_start + predator->controller_step;
  last_activation_time = 0.0;

  printf( "gazebo/predator_controller has initialized\n" );

  //sprintf( infostr, "time,dtime,distance,q_prey_1,q_prey_2,q_prey_3,q_prey_4,q_prey_5,q_prey_6,q_prey_7,q_prey_8,q_prey_9,q_prey_10,q_prey_11,q_prey_12,q_prey_13,q_pred_1,q_pred_2,q_pred_3,q_pred_4,q_pred_5,q_pred_6,q_pred_7,q_pred_8,q_pred_9,q_pred_10,q_pred_11,q_pred_12,q_pred_13,u_pred_plan_1,u_pred_plan_2,u_pred_plan_3,u_pred_plan_4,u_pred_plan_5,u_pred_plan_6,u_pred_fb_1,u_pred_fb_2,u_pred_fb_3,u_pred_fb_4,u_pred_fb_5,u_pred_fb_6,u_pred_1,u_pred_2,u_pred_3,u_pred_4,u_pred_5,u_pred_6\n" );
  //info.write( infostr );
  //info.flush( );

  controller_waitbegin_ts = generate_timestamp();
  planner_waitbegin_ts = controller_waitbegin_ts;
}

//-----------------------------------------------------------------------------
void predator_controller_c::Update( ) {
  bool refreshed_plan = false;
  cycle_t plan_exec_time_cycle, plan_wait_time_cycle;
  double plan_exec_time_sec, plan_wait_time_sec;
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
///*
  prey->time = time;
  prey->dtime = dtime;
  predator->time = time;
  predator->dtime = dtime;
//*/
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
  // planned control
  //bool result = predator->plan_rrt( pred_state, prey_state, pred_planned_control );

/*
  bool failed_to_plan = !predator->compute_predator_plan( pred_state, prey_state, us, durations, control_count );
  if( failed_to_plan ) {
    for( unsigned i = 0; i < 6; i++ )
      pred_planned_control[i] = 0.0;
    planner_failures++;
  } else {
    for( unsigned i = 0; i < 6; i++ )
      pred_planned_control[i] = us[0][i];
  }
  planner_calls++;
*/

 // see which command to use
  if (time - _plan_start > _plan_duration)
  {
    // need to formulate a plan
    // planned control
    //bool result = predator->plan_rrt( pred_state, prey_state, pred_planned_control );
    planner_activation_ts = generate_timestamp();
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


/*
  if( !result ) {
    sprintf( infostr, "[FAILED] " );
    info.write( infostr );
  }
*/

  sprintf( infostr, "%f", time );
  info.write( infostr );

  sprintf( infostr, " %f %f %f %f %f %f %f %f %f %f %f %f %f", pred_state[0], pred_state[1], pred_state[2], pred_state[3], pred_state[4], pred_state[5], pred_state[6], pred_state[7], pred_state[8], pred_state[9], pred_state[10], pred_state[11], pred_state[12] );
  info.write( infostr );

  sprintf( infostr, " %f %f %f %f %f %f %f %f %f %f %f %f %f\n", prey_state[0], prey_state[1], prey_state[2], prey_state[3], prey_state[4], prey_state[5], prey_state[6], prey_state[7], prey_state[8], prey_state[9], prey_state[10], prey_state[11], prey_state[12] );
  info.write( infostr );

  //sprintf( infostr, ";%f,%f,%f,%f,%f,%f", pred_planned_control[0], pred_planned_control[1], pred_planned_control[2], pred_planned_control[3], pred_planned_control[4], pred_planned_control[5] );
  //info.write( infostr );
  
  //sprintf( infostr, ";%f,%f,%f,%f,%f,%f", pred_feedback_control[0], pred_feedback_control[1], pred_feedback_control[2], pred_feedback_control[3], pred_feedback_control[4], pred_feedback_control[5] );
  //info.write( infostr );
  
  //sprintf( infostr, ";%f,%f,%f,%f,%f,%f\n", pred_control[0], pred_control[1], pred_control[2], pred_control[3], pred_control[4], pred_control[5] );
  //info.write( infostr );

  if( info.size() >= info.capacity() * 9 / 10 || time + dtime >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
    info.flush();
  }
  if( summary.size() >= summary.capacity() * 9 / 10 || time + dtime >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
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
/*
void predator_controller_c::Reset( ) {

}
*/
//-----------------------------------------------------------------------------
