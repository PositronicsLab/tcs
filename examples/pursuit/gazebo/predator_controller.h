#ifndef _GAZEBO_PREDATOR_CONTROLLER_H_
#define _GAZEBO_PREDATOR_CONTROLLER_H_

//-----------------------------------------------------------------------------

#include "gazebo_space.h"
#include "gazebo_ship.h"

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <tcs/log.h>
#include "../experiment.h"

#include <tcs/cpu.h>
#include <tcs/time.h>

//-----------------------------------------------------------------------------

class predator_controller_c : public gazebo::ModelPlugin {

private:
  // the reference so that this ship is inserted into gazebo's callback system
  gazebo::event::ConnectionPtr _updateConnection;
  // the gazebo reference to the world in which the ship is located
  gazebo::physics::WorldPtr _world;

  double time;
  double dtime;

  double integration_step;
  //double controller_step;
  double last_activation_time;
  double next_activation_time;

  double time_start;
  double time_last;

  gazebo_space_p space;
  ship_p prey;
  ship_p predator;

  unsigned planner_calls;
  unsigned planner_failures;

  // EMD: added to stream plan commands
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

  std::vector< std::vector<double> > us;
  std::vector<double> durations;
  unsigned control_count;

  log_c info;
  log_c summary;
  char infostr[512];
  log_c timelog_controller;
  log_c timelog_planner;

  bool captured;

  timestamp_t controller_activation_ts;
  timestamp_t controller_yield_ts;
  timestamp_t controller_waitbegin_ts;
  timestamp_t planner_activation_ts;
  timestamp_t planner_yield_ts;
  timestamp_t planner_waitbegin_ts;

  cpu_speed_t cpu_speed;

public:
  predator_controller_c( void );
  virtual ~predator_controller_c( void );

  // Gazebo callback.  Called when the simulation is starting up
  virtual void Load( gazebo::physics::ModelPtr model, sdf::ElementPtr sdf );

  // Gazebo callback.  Called whenever the simulation advances a timestep
  virtual void Update( );

  // Gazebo callback.  Called whenever the simulation is reset
  //virtual void Reset( );

};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_PREDATOR_CONTROLLER_H_
