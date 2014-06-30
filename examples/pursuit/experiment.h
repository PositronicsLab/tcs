#ifndef _PURSUIT_EXPERIMENT_H_
#define _PURSUIT_EXPERIMENT_H_

#include <tcs/tcs.h>

//-----------------------------------------------------------------------------
// Experimental Open Parameters
//-----------------------------------------------------------------------------
#define EXPERIMENTAL_TRIAL_DURATION_SEC      20.0
#define EXPERIMENTAL_SEED                    1

#define BOUNDARY_REPULSION_FACTOR            10.0
#define PREDATOR_REPULSION_FACTOR            0.2
#define BOUNDARY_GAIN                        20.0
#define BOUNDARY_FIELD_DIST                  5.0
#define DRAG_COEFFICIENT                     0.0

#define FLEE_DISTANCE                        5.0
#define CAPTURE_DISTANCE                     0.5

#define PREY_MAX_FORCE                       8.0
#define PREY_MAX_TORQUE                      0.0
#define PREY_WALK_FORCE_WEIGHT               1.0
#define PREY_FLEE_FORCE_WEIGHT               2.0
#define PREY_BOUNDARY_FORCE_WEIGHT           10.0
#define PREY_COMBINED_FORCE_WEIGHT           1.0

#define PREY_COMMAND_SEED_FACTOR             10
#define PREDATOR_TIME_RAND_FACTOR            2.0

#define PLANNER_STEP_SIZE                    0.01
#define PLANNER_MAX_PLAN_DURATION            0.1
#define PLANNER_MAX_CONTROL_COUNT            MAX_PLANNER_CONTROLS
#define PLANNER_MAX_PLANNING_TIME            1.0 

#define PLANNER_DERIVATIVE_LIN_TOL           1e-3
#define PLANNER_DERIVATIVE_ANG_TOL           1e-3
#define PLANNER_MAX_DERIVATIVE_LIN           20.0
#define PLANNER_MAX_DERIVATIVE_ANG           0.0
#define PLANNER_MAX_FORCE                    1.0
#define PLANNER_MAX_TORQUE                   0.001
#define PLANNER_GOAL_BIAS                    0.99

#define PREDATOR_CONTROLLER_PERIOD_SEC       0.01
#define PREY_CONTROLLER_PERIOD_SEC           0.01

#define FEEDBACK_GAIN_PROPORTIONAL_POSITION  8.0
#define FEEDBACK_GAIN_DERIVATIVE_POSITION    2.0
#define FEEDBACK_GAIN_PROPORTIONAL_ROTATION  1.0
#define FEEDBACK_GAIN_DERIVATIVE_ROTATION    0.1

//#define LOG_CAPACITY 1048576  // 1MB
#define PREY_CONTROLLER_LOG_FILE             "prey.log"
#define PREDATOR_CONTROLLER_LOG_FILE         "predator.log"

// Variables for the Random Walk planning
#define GAUSSIAN_MEAN                        0.0
#define GAUSSIAN_VARIANCE                    35.0
#define GAUSSIAN_STDDEV                      5.91607978309962

/*
#define FLEE_DISTANCE                4.0
#define CAPTURE_DISTANCE             0.5
#define PREY_MIN_FORCE               5.0
#define PREY_MIN_TORQUE              0.0
#define PREY_MAX_FORCE               0.5
#define PREY_MAX_TORQUE              0.0
#define PREY_WALK_FORCE_WEIGHT       1.0
#define PREY_FLEE_FORCE_WEIGHT       10.0
#define PREY_BOUNDARY_FORCE_WEIGHT   100.0
#define PREY_COMBINED_FORCE_WEIGHT   1.0
//#define PREY_COMMAND_SEED_FACTOR     1000
#define PREY_COMMAND_SEED_FACTOR     100
#define PREDATOR_TIME_RAND_FACTOR    1.0

  // the time step used by the planner
#define PLANNER_STEP_SIZE            0.01
//#define PLANNER_STEP_SIZE            0.01
#define PLANNER_MAX_PLANNING_TIME    0.1
//#define PLANNER_MAX_DERIVATIVE       2e1
//#define PLANNER_MAX_DERIVATIVE       2.5
#define PLANNER_MAX_DERIVATIVE_LIN     5.0
#define PLANNER_MAX_DERIVATIVE_ANG     20.0
//#define PLANNER_MAX_FORCE            1e2
#define PLANNER_MAX_FORCE            1e3
#define PLANNER_GOAL_BIAS            0.05

#define PREDATOR_CONTROLLER_PERIOD_SEC  0.01
#define PREY_CONTROLLER_PERIOD_SEC      0.01

#define FEEDBACK_GAIN_PROPORTIONAL_POSITION   8.0
#define FEEDBACK_GAIN_DERIVATIVE_POSITION      2.0
#define FEEDBACK_GAIN_PROPORTIONAL_ROTATION    1.0
#define FEEDBACK_GAIN_DERIVATIVE_ROTATION      0.1

#define PREY_CONTROLLER_LOG_FILE       "prey.log"
#define PREDATOR_CONTROLLER_LOG_FILE   "predator.log"

#define DRAG                    1e2
//#define DRAG                    0.1
//#define BOUNDARY_REPULSION_FACTOR 0.01
#define BOUNDARY_REPULSION_FACTOR 0.05
//#define PREDATOR_REPULSION_FACTOR 0.05
#define PREDATOR_REPULSION_FACTOR 0.2

  // Variables for the Random Walk planning
#define GAUSSIAN_MEAN           0.0
#define GAUSSIAN_VARIANCE       35.0
#define GAUSSIAN_STDDEV         5.91607978309962
*/

/*
#define FLEE_DISTANCE                4.0
#define CAPTURE_DISTANCE             1.0
//#define PREY_MAX_FORCE               1e5
//#define PREY_MAX_TORQUE              1e5
#define PREY_MAX_FORCE               25.0
#define PREY_MAX_TORQUE              25.0
#define PREY_WALK_FORCE_WEIGHT       1.0
#define PREY_FLEE_FORCE_WEIGHT       5.0
#define PREY_BOUNDARY_FORCE_WEIGHT   10.0
#define PREY_COMBINED_FORCE_WEIGHT   1.0
//#define PREY_COMMAND_SEED_FACTOR     1000
#define PREY_COMMAND_SEED_FACTOR     100
#define PREDATOR_TIME_RAND_FACTOR    1.0

  // the time step used by the planner
#define PLANNER_STEP_SIZE            0.02
//#define PLANNER_STEP_SIZE            0.01
#define PLANNER_MAX_PLANNING_TIME    0.1
#define PLANNER_MAX_DERIVATIVE       2e1
//#define PLANNER_MAX_FORCE            1e2
#define PLANNER_MAX_FORCE            1e3
#define PLANNER_GOAL_BIAS            0.05

#define PREDATOR_CONTROLLER_PERIOD_SEC  0.01
#define PREY_CONTROLLER_PERIOD_SEC      0.01

#define FEEDBACK_GAIN_PROPORTIONAL_POSITION   100.0
#define FEEDBACK_GAIN_DERIVATIVE_POSITION      10.0
#define FEEDBACK_GAIN_PROPORTIONAL_ROTATION    10.0
#define FEEDBACK_GAIN_DERIVATIVE_ROTATION       1.0

#define PREY_CONTROLLER_LOG_FILE       "prey.log"
#define PREDATOR_CONTROLLER_LOG_FILE   "predator.log"

#define DRAG                    1e2
//#define DRAG                    0.1
#define BOUNDARY_REPULSION_FACTOR 0.005
#define PREDATOR_REPULSION_FACTOR 0.05

  // Variables for the Random Walk planning
#define GAUSSIAN_MEAN           0.0
#define GAUSSIAN_VARIANCE       35.0
#define GAUSSIAN_STDDEV         5.91607978309962
*/
//-----------------------------------------------------------------------------

#endif // _PURSUIT_EXPERIMENT_H_
