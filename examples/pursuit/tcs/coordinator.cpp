//#include <sstream>
#include <vector>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdio.h>
#include <sys/wait.h>
#include <atomic>
#include <cstring>
#include <sstream>

#include <tcs/cpu.h>
#include <tcs/timer.h>
#include <tcs/os.h>
#include <tcs/types.h>
#include <tcs/notification.h>
#include <tcs/message.h>
#include <tcs/processor.h>
#include <tcs/timesink.h>
#include <tcs/osthread.h>
#include <tcs/dynamics.h>

#include <tcs/channels.h>

#include "../experiment.h"

#define DYNAMICS_PLUGIN "./libpursuit-tcs-moby-plugin.so"

#define DO_LOGGING
//#undef DO_LOGGING

//-----------------------------------------------------------------------------
const bool             suppress_client_console_io = true;
//-----------------------------------------------------------------------------
static pid_t           coordinator_pid;
static int             coordinator_os_priority;
static cpu_speed_t     cpu_speed;
static cpu_id_t        cpu;
//-----------------------------------------------------------------------------
static int             CLIENT_OS_MAX_PRIORITY;
static int             CLIENT_OS_MIN_PRIORITY;
static thread_p        current_thread;

thread_p processor_thread;
thread_p prey_thread;
thread_p pred_thread;

//-----------------------------------------------------------------------------
bool                    wakeup_created = false;
std::atomic<int>        wakeup_enabled;
static notification_t   wakeup_note;
static int              wakeup_os_priority;
static int              wakeup_write_fd;
pid_t                   wakeup_pid;

//-----------------------------------------------------------------------------
static notification_t  serviced_read_note;

//-----------------------------------------------------------------------------
static int             timer_os_priority;
static notification_t  timer_note;
static int             timer_write_fd;
timer_c                timer;
unsigned               caught_timer_events;
unsigned               actual_timer_events;

//#define MAX_TIMER_EVENTS 100
#define MAX_TIMER_EVENTS 5000
//#define TIMER_PERIOD_NSECS 1000000
#define TIMER_PERIOD_NSECS 10000000  // 10 ms
//#define TIMER_PERIOD_NSECS 100000000  // 100 ms
//-----------------------------------------------------------------------------

//bool quit;
//int quit;
std::atomic<int>       quit;

//-----------------------------------------------------------------------------
//char errstr[ 256 ];
char spstr[8092];

//-----------------------------------------------------------------------------
log_p summary;
log_p info;
//-----------------------------------------------------------------------------

std::vector<int>        subscribed_fds;
fd_set                  pending_fds;

//-----------------------------------------------------------------------------
client_message_buffer_c msgbuffer;

//-----------------------------------------------------------------------------
// Pthreads
pthread_t wakeup_thread;

//-----------------------------------------------------------------------------
// Threads
boost::shared_ptr<processor_c> processor;

boost::shared_ptr<dynamics_c> dynamics;

boost::shared_ptr<timesink_c> prey;
boost::shared_ptr<osthread_c> prey_controller;

boost::shared_ptr<timesink_c> pred;
boost::shared_ptr<osthread_c> pred_controller;
boost::shared_ptr<osthread_c> pred_planner;

//std::vector<osthread_p>  clients;

//-----------------------------------------------------------------------------
//static control_t prey_controller_control;
//static control_t pred_controller_control;
static plan_t    plan;
static unsigned  plan_idx;
static bool      no_plan;
//static double    current_plan_duration;

static unsigned         planner_calls;
static unsigned         planner_failures;
static double       current_accum_plan_duration;

//-----------------------------------------------------------------------------
void kill_prey_controller( void ) {
  int status;

  if( !prey_controller ) return;

  if( info ){
    sprintf( spstr, "killing prey_controller\n" );
    info->write( spstr );
  }

  kill( prey_controller->pid, SIGTERM );
  waitpid( prey_controller->pid, &status, 0 );

  if( info ){
    sprintf( spstr, "killed prey_controller\n" );
    info->write( spstr );
  }
}

//-----------------------------------------------------------------------------
void kill_pred_controller( void ) {
  int status;

  if( !pred_controller ) return;

  if( info ){
    sprintf( spstr, "killing pred_controller\n" );
    info->write( spstr );
  }

  kill( pred_controller->pid, SIGTERM );
  waitpid( pred_controller->pid, &status, 0 );

  if( info ){
    sprintf( spstr, "killed pred_controller\n" );
    info->write( spstr );
  }
}

//-----------------------------------------------------------------------------
void kill_pred_planner( void ) {
  int status;

  if( !pred_planner ) return;

  if( info ){
    sprintf( spstr, "killing pred_planner\n" );
    info->write( spstr );
  }

  kill( pred_planner->pid, SIGTERM );
  waitpid( pred_planner->pid, &status, 0 );

  if( info ){
    sprintf( spstr, "killed pred_planner\n" );
    info->write( spstr );
  }
}

//-----------------------------------------------------------------------------
void kill_wakeup( void ) {
  if( !wakeup_created ) return;

  if( info ){
    sprintf( spstr, "killing wakeup\n" );
    info->write( spstr );
  }

  pthread_cancel( wakeup_thread );

  if( info ){
    sprintf( spstr, "killed wakeup\n" );
    info->write( spstr );
  }
}

//-----------------------------------------------------------------------------
void kill_timer( void ) {

  if( info ){
    sprintf( spstr, "killing timer\n" );
    info->write( spstr );
  }

  timer.block();
  timer.destroy();

  if( info ){
    sprintf( spstr, "killed timer\n" );
    info->write( spstr );
  }
}

//-----------------------------------------------------------------------------
void log_progress( osthread_p thread ) {

  double t = cycles_to_seconds(dynamics->temporal_progress,cpu_speed);
  double step = dynamics->step_size_realtime();

  if( thread == pred_planner ) {
    cycle_t prev_progress = thread->prev_computational_progress;
    cycle_t delta_progress = thread->computational_progress - prev_progress;
    
    double exec_time_sec = cycles_to_seconds( delta_progress, cpu_speed );
    double wait_time_sec = 0.0;

    sprintf( spstr, "%f %f %f\n", t, exec_time_sec, wait_time_sec );
    thread->timelog.write( spstr );
    thread->timelog.flush();
/*
    if( thread->timelog.size() >= thread->timelog.capacity() * 9 / 10 || t + step >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
      thread->timelog.flush();
    }
*/
    thread->prev_computational_progress = thread->computational_progress;

  } else if( thread == pred_controller ) {
    cycle_t prev_progress = thread->prev_computational_progress;
    cycle_t delta_progress = thread->computational_progress - prev_progress;
    
    double exec_time_sec = cycles_to_seconds( delta_progress, cpu_speed );
    double wait_time_sec = std::max(PREDATOR_CONTROLLER_PERIOD_SEC - exec_time_sec, 0.0);

    sprintf( spstr, "%f %f %f\n", t, exec_time_sec, wait_time_sec );
    thread->timelog.write( spstr );

    if( thread->timelog.size() >= thread->timelog.capacity() * 9 / 10 || t + step >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
      thread->timelog.flush();
    }

    thread->prev_computational_progress = thread->computational_progress;

  } else if( thread == prey_controller ) {
    cycle_t prev_progress = thread->prev_computational_progress;
    cycle_t delta_progress = thread->computational_progress - prev_progress;
    
    double exec_time_sec = cycles_to_seconds( delta_progress, cpu_speed );
    double wait_time_sec = std::max(PREY_CONTROLLER_PERIOD_SEC - exec_time_sec, 0.0);

    sprintf( spstr, "%f %f %f\n", t, exec_time_sec, wait_time_sec );
    thread->timelog.write( spstr );

    if( thread->timelog.size() >= thread->timelog.capacity() * 9 / 10 || t + step >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
      thread->timelog.flush();
    }

    thread->prev_computational_progress = thread->computational_progress;
  }
}

//-----------------------------------------------------------------------------
void service_command_publication( osthread_p thread ) {

  // read the updated command from the shared buffer
  client_message_t msg;

  if( thread == pred_planner ) 
    // make sure to read the plan section
    msgbuffer.read( msg, true );
  else
    msgbuffer.read( msg );

  // update the correlated field in the dynamics buffer
  if( thread == prey_controller ) {
    memcpy(&dynamics->buffer.prey_control, &msg.control, sizeof(struct control_t));
  } else if( thread == pred_controller ) {
    memcpy(&dynamics->buffer.pred_control, &msg.control, sizeof(struct control_t));
  } else if( thread == pred_planner ) {
    memcpy( &plan, &msg.plan, sizeof(struct plan_t) );
    plan_idx = 0;
    no_plan = false;
    //printf( "(coord) control_count:%d\n", plan.count );
    current_accum_plan_duration = 0;
    if( plan.failed ) planner_failures++;
  }
/*
  if( fabs( dynamics->buffer.prey_ke ) > 500 || fabs( dynamics->buffer.pred_ke ) > 500 ) {
    sprintf( spstr, "**Kinetic Energy High: prey_ke[%f], pred_ke[%f]**\n", dynamics->buffer.prey_ke, dynamics->buffer.pred_ke );
    info->write( spstr );
    printf( "%s", spstr );
  }
*/
}

//-----------------------------------------------------------------------------
void service_state_request( osthread_p thread ) {
  int fd;
  notification_t response;
  ssize_t bytes;
  client_message_t msg;

  // determine the correct channel to send the notification on
  if( thread == prey_controller ) {
    fd = FD_COORDINATOR_TO_PREYCONTROLLER_WRITE_CHANNEL;
  } else if( thread == pred_planner ) {
    fd = FD_COORDINATOR_TO_PREDPLANNER_WRITE_CHANNEL;
  } else if( thread == pred_controller ) {
    fd = FD_COORDINATOR_TO_PREDCONTROLLER_WRITE_CHANNEL;
  } else {
    // should never get here.  Sanity check.
    assert( true );  // ERROR
  }

  // fetch the correlated dynamics buffer data and update that part of message.
  if( thread == prey_controller ) {
    // fetch the prey and predator state data
    msg.header.t = cycles_to_seconds( prey_controller->temporal_progress, cpu_speed );
    msg.header.dt = prey_controller->desired_period_realtime;
    memcpy(&msg.prey_state,&dynamics->buffer.prey_state,sizeof(struct state_t));
    memcpy(&msg.pred_state,&dynamics->buffer.pred_state,sizeof(struct state_t));

    log_progress( prey_controller );
/*
    cycle_t wait_time_cycles = pred_controller->computational_progress + pred_planner->computational_progress;
    double wait_time_sec = cycles_to_seconds( wait_time_cycles, cpu_speed );
    double step = dynamics->step_size_realtime();
    double exec_time_sec = cycles_to_seconds( prey_controller->computational_progress, cpu_speed );
    double t = msg.header.t;

    sprintf( spstr, "%f %f %f\n", t, exec_time_sec, wait_time_sec );
    prey_controller->timelog.write( spstr );

    if( prey_controller->timelog.size() >= prey_controller->timelog.capacity() * 9 / 10 || t + step >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
      prey_controller->timelog.flush();
    }
*/
  } else if( thread == pred_planner ) {
    // fetch the prey and predator state data
    msg.header.t = cycles_to_seconds( pred_planner->temporal_progress, cpu_speed );
    msg.header.dt = pred_planner->desired_period_realtime;
    memcpy(&msg.prey_state,&dynamics->buffer.prey_state,sizeof(struct state_t));
    memcpy(&msg.pred_state,&dynamics->buffer.pred_state,sizeof(struct state_t));
    planner_calls++;

//    log_progress( pred_planner );
/*
    cycle_t wait_time_cycles = pred_controller->computational_progress + prey_controller->computational_progress;
    double wait_time_sec = cycles_to_seconds( wait_time_cycles, cpu_speed );
    double step = dynamics->step_size_realtime();
    double exec_time_sec = cycles_to_seconds( pred_planner->computational_progress, cpu_speed );
    double t = msg.header.t;

    sprintf( spstr, "%f %f %f\n", t, exec_time_sec, wait_time_sec );
    pred_planner->timelog.write( spstr );

//    if( pred_planner->timelog.size() >= pred_planner->timelog.capacity() * 9 / 10 || t + step >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
//      pred_planner->timelog.flush();
//    }
    pred_planner->timelog.flush();
*/
  } else if( thread == pred_controller ) {
    // fetch the prey and predator state and planner control data
    msg.header.t = cycles_to_seconds( pred_controller->temporal_progress, cpu_speed );
    msg.header.dt = pred_controller->desired_period_realtime;
    memcpy(&msg.prey_state,&dynamics->buffer.prey_state,sizeof(struct state_t));
    memcpy(&msg.pred_state,&dynamics->buffer.pred_state,sizeof(struct state_t));

    // send the current planned control through

    timestamp_t ts = generate_timestamp();
    msg.control.ts = ts;
///*
    // Fugly
    // TODO: Unfuglify
    //printf( "(coord) here 1\n" );

    // set the default command to zero
    for( unsigned i = 0; i < 6; i++ ) msg.control.u[i] = 0.0;

    if( plan.updated ) {
        if ( !plan.failed )
          memcpy( &msg.control, &plan.control[plan_idx], sizeof(struct control_t) );
        plan.updated = false;
    } else {
      // this code exists because we always want to send the first control
      // to be sent
      if( !plan.failed ) {
        double t = -PREDATOR_CONTROLLER_PERIOD_SEC;
        while( t < 0.0 && plan_idx < plan.count ) {
          t += plan.control[plan_idx++].duration;
        }
        if( plan_idx < plan.count ) {
          current_accum_plan_duration += PREDATOR_CONTROLLER_PERIOD_SEC;
          if( current_accum_plan_duration <= PLANNER_MAX_PLAN_DURATION ) {
            // take that control
            memcpy( &msg.control, &plan.control[plan_idx], sizeof(struct control_t) );
          }

          // take that control
//        memcpy( &msg.control, &plan.control[plan_idx], sizeof(struct control_t) );

        }
//*/
      } 
    }
      //printf( "(coord) here 2\n" );
//*/
/*
    // if a plan, the planner didnt fail and havent overrun the buffered plan
    if( !no_plan && !plan.failed && plan_idx < plan.count ) {
      // consume some of the control duration (Note: this is predecrement)
      plan.control[plan_idx].duration -= PREDATOR_CONTROLLER_PERIOD_SEC;
      // if the duration is fully consumed, move to the next control
      if( plan.control[plan_idx].duration < 0.0 ) plan_idx++;
    }

    // Note: following not nested with above because the above updates plan_idx
    // if no plan, the planner failed, or have overrun the buffered plan
    if( no_plan || plan.failed || plan_idx >= plan.count ) {
      // send a zero control
      for( unsigned i = 0; i < 6; i++ ) msg.control.u[i] = 0.0;
    } else {
      // otherwise send the current control
      memcpy( &msg.control, &plan.control[plan_idx], sizeof(struct control_t) );
    }
*/

    sprintf( spstr, "writing msg: t[%f], dt[%f]", msg.header.t, msg.header.dt );
    info->write( spstr );
    sprintf( spstr, ", prey_state[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", msg.prey_state.q[0], msg.prey_state.q[1], msg.prey_state.q[2], msg.prey_state.q[3], msg.prey_state.q[4], msg.prey_state.q[5], msg.prey_state.q[6], msg.prey_state.dq[0], msg.prey_state.dq[1], msg.prey_state.dq[2], msg.prey_state.dq[3], msg.prey_state.dq[4], msg.prey_state.dq[5] );
    info->write( spstr );
    sprintf( spstr, ", pred_state[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]", msg.pred_state.q[0], msg.pred_state.q[1], msg.pred_state.q[2], msg.pred_state.q[3], msg.pred_state.q[4], msg.pred_state.q[5], msg.pred_state.q[6], msg.pred_state.dq[0], msg.pred_state.dq[1], msg.pred_state.dq[2], msg.pred_state.dq[3], msg.pred_state.dq[4], msg.pred_state.dq[5] );
    info->write( spstr );
    sprintf( spstr, ", planner_control[%f,%f,%f,%f,%f,%f]", msg.control.u[0], msg.control.u[1], msg.control.u[2], msg.control.u[3], msg.control.u[4], msg.control.u[5] );
    info->write( spstr );
    sprintf( spstr, ", plan[ no_plan[%d], plan.failed[%d], plan.count[%u], plan_idx[%u] ]\n", no_plan, plan.failed, plan.count, plan_idx );
    info->write( spstr );

    log_progress( pred_controller );
/*
    cycle_t wait_time_cycles = pred_planner->computational_progress + prey_controller->computational_progress;
    double wait_time_sec = cycles_to_seconds( wait_time_cycles, cpu_speed );
    double step = dynamics->step_size_realtime();
    double exec_time_sec = cycles_to_seconds( pred_controller->computational_progress, cpu_speed );
    double t = msg.header.t;

    sprintf( spstr, "%f %f %f\n", t, exec_time_sec, wait_time_sec );
    pred_controller->timelog.write( spstr );

    if( pred_controller->timelog.size() >= pred_controller->timelog.capacity() * 9 / 10 || t + step >= EXPERIMENTAL_TRIAL_DURATION_SEC ) { 
      pred_controller->timelog.flush();
    }
*/
  }

  msg.header.ts = generate_timestamp();
  // write the message to the shared buffer
  msgbuffer.write( msg );

  //sprintf( spstr, "server responding to %s with notification: source[SERVER], type[READ]\n", thread->name );
  //info->write( spstr );
  //printf( "%s", spstr );

  // send the notification to the client that the read request has been serviced
  serviced_read_note.ts = generate_timestamp();
  // NOTE: notification timestamp should probably be same as msg timestamps
  if( __write( fd, &serviced_read_note, sizeof(notification_t), bytes ) != OS_ERROR_NONE ) {
    // TODO: Handle/Recover
    if( info ) {
      sprintf( spstr, "ERROR : (coordinator.cpp) write_state() failed calling __write(...)\n" );
      info->write( spstr );
    }
  }
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool select( void ) {
//  __select( subscribed_fds, pending_fds );

  int max_fd = 0;

  assert( subscribed_fds.size() > 0 );

  FD_ZERO( &pending_fds );
  for( int i = 0; i < subscribed_fds.size(); i++ ) {
    max_fd = std::max( max_fd, subscribed_fds[i] );
    FD_SET( subscribed_fds[i], &pending_fds );
    //printf( "fd:%d, ", fds[i] );
  }
  //printf( "max_fd:%d\n", max_fd );
  if( select( max_fd + 1, &pending_fds, NULL, NULL, NULL ) == -1 ) {

    if( info ) {
      char buf[16];
      if( errno == EBADF ) 
        sprintf( buf, "EBADF" );
      else if( errno == EINTR ) 
        sprintf( buf, "EINTR" );
      else if( errno == EINVAL ) 
        sprintf( buf, "EINVAL" );
      else if( errno == ENOMEM ) 
        sprintf( buf, "ENOMEM" );
      else 
        sprintf( buf, "UNKNOWN" );

      sprintf( spstr, "ERROR : (coordinator.cpp) select() failed calling __select(...) : errno[%s]\n", buf );
      info->write( spstr );
    }

    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
void read_notifications( void ) {
  notification_t note;
  int fd;

  // check for a timer event.  If interrupted due to timer (but not blocked)
  // will reschedule into the run queue
  if( FD_ISSET( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, &pending_fds ) != 0 ) {
    fd = FD_TIMER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_TIMER_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
      }

    } else {
      if( info ) {
        sprintf( spstr, "read_notifications( note.source=TIMER, caught_timer_events=%d, actual_timer_events=%d\n", ++caught_timer_events, actual_timer_events );
        info->write( spstr );
      }
      current_thread->message_queue.push( note );

      osthread_p current_osthread = boost::dynamic_pointer_cast<osthread_c>(current_thread);
      if( current_osthread && current_osthread == pred_planner )
        log_progress( pred_planner );

/*
      // exit condition for testing
      if( caught_timer_events >= MAX_TIMER_EVENTS ) {
        //quit.store( 1, std::memory_order_relaxed  );
        //quit.store( 1, std::memory_order_seq_cst  );
        //quit = true;
        //quit++;
        if( info ) {
          info->flush();
        }
        kill( coordinator_pid, SIGTERM );
      }
*/
      // experiment exit condition
      double sim_time=cycles_to_seconds(dynamics->temporal_progress,cpu_speed);
      if( sim_time > EXPERIMENTAL_TRIAL_DURATION_SEC ) {
        if( info ) {
          info->flush();
        }
        sprintf( spstr, "planner_calls[%u], planner_failures[%u], failure_percent[%f]\n", planner_calls, planner_failures, ((float)planner_failures/(float)planner_calls)*100.0);
        summary->write( spstr );
        summary->flush();

        dynamics->flush_log();

        kill( coordinator_pid, SIGTERM );
      }
    }
  }

  // - check for client specific notifications -
  // prey controller
  if( FD_ISSET( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL, &pending_fds ) != 0) {
    fd = FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
      }
      //printf( "%s\n", spstr );
    } else {
      if( info ) {
        char note_type[8];
        if( note.type == notification_t::IDLE )
          sprintf( note_type, "IDLE" );
        else if( note.type == notification_t::READY )
          sprintf( note_type, "READY" );
        else if( note.type == notification_t::CLOSE )
          sprintf( note_type, "CLOSE" );
        else if( note.type == notification_t::READ )
          sprintf( note_type, "READ" );
        else if( note.type == notification_t::WRITE )
          sprintf( note_type, "WRITE" );

        sprintf( spstr, "read_notifications( note.source=CLIENT, note.type=%s, client=prey_controller )\n", note_type );
        info->write( spstr );
      }

      prey_controller->message_queue.push( note );

//      log_progress( prey_controller );
    }
  }

  // predator planner
  if( FD_ISSET( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL, &pending_fds) != 0 ) {
    fd = FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
      }
      //printf( "%s\n", spstr );
    } else {
      if( info ) {
        char note_type[8];
        if( note.type == notification_t::IDLE )
          sprintf( note_type, "IDLE" );
        else if( note.type == notification_t::READY )
          sprintf( note_type, "READY" );
        else if( note.type == notification_t::CLOSE )
          sprintf( note_type, "CLOSE" );
        else if( note.type == notification_t::READ )
          sprintf( note_type, "READ" );
        else if( note.type == notification_t::WRITE )
          sprintf( note_type, "WRITE" );

        sprintf( spstr, "read_notifications( note.source=CLIENT, note.type=%s, client=pred_planner )\n", note_type );
        info->write( spstr );
      }

      pred_planner->message_queue.push( note );

      log_progress( pred_planner );
    }
  }

  // predator controller
  if( FD_ISSET( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL, &pending_fds ) != 0 ) {
    fd = FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
      }
      //printf( "%s\n", spstr );
    } else {
      if( info ) {
        char note_type[8];
        if( note.type == notification_t::IDLE )
          sprintf( note_type, "IDLE" );
        else if( note.type == notification_t::READY )
          sprintf( note_type, "READY" );
        else if( note.type == notification_t::CLOSE )
          sprintf( note_type, "CLOSE" );
        else if( note.type == notification_t::READ )
          sprintf( note_type, "READ" );
        else if( note.type == notification_t::WRITE )
          sprintf( note_type, "WRITE" );

        sprintf( spstr, "read_notifications( note.source=CLIENT, note.type=%s, client=pred_controller )\n", note_type );
        info->write( spstr );
      }

      pred_controller->message_queue.push( note );

//      log_progress( pred_controller );
    }
  }

  // check block detection first.  If client blocked, needs to be put into 
  // blocking queue.  If falls through, might get put into run queue instead.
  if( FD_ISSET( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, &pending_fds) != 0 ) {
    fd = FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL;
    if( __read( fd, &note, sizeof(notification_t) ) == -1 ) {
      if( info ) {
        sprintf( spstr, "ERROR : (coordinator.cpp) read_messages() failed calling __read(FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL,...)\n" );
        info->write( spstr );
      }
    } else {
      if( info ) {
        sprintf( spstr, "read_notifications( note.source=WAKEUP)\n" );
        info->write( spstr );
      }

      current_thread->message_queue.push( note );

      // reenable block detection notifications
      wakeup_enabled.store( 1, std::memory_order_seq_cst  );
    }
  }
}

//-----------------------------------------------------------------------------
/// API function to process notifications delivered to a thread.
/// @param thread the current thread.
/// @param runqueue the heap of threads ready to run for a given timesink.
/// @param waitqueue the heap of threads pending on I/O for a given timesink.
void process_notifications( const thread_p& caller, osthread_p& thread, thread_heap_c* runqueue, thread_heap_c* waitqueue ) {
  //cycle_t reschedule_time;
  char spstr[512];
/*
  // assert that the thread is valid 
  if( !thread ) { 
    printf( "thread was nothing\n" );
    return;
  }
*/
//#ifdef DEBUG
  if( info && thread ) {
    sprintf( spstr, "process_notifications( caller=%s, thread=%s, ", caller->name, thread->name );
    info->write( spstr );
    
    if( thread->owner ) {
      sprintf( spstr, "owner=%s, ... )\n", thread->owner->name );
    } else {
      sprintf( spstr, "owner=root, ... )\n" );
    }
    info->write( spstr );
  } else {
    sprintf( spstr, "process_notifications( caller=%s, thread=none, ... )\n", caller->name );
    info->write( spstr );
  }
//#endif

  // assert that the message queue is not empty
  // NOTE : This is probably only going to occur due to an implicit block event
  if( thread->message_queue.empty() ) {
//    assert( true );
    runqueue->push( thread );
    return;
  }

  // TODO: Need to service all pending messages here with a message loop
  // in the process of the loop determine what queue the thread belongs to
  // and set that after handling all messages

  //printf( "size of %s's message queue: %d\n", thread->name, thread->message_queue.size() );

  bool blocked = false;
  while( !thread->message_queue.empty() ) {
    // peek at the message at the head of the queue
    notification_t msg = thread->message_queue.front();
    // remove the message from the queue
    thread->message_queue.pop();

    if( msg.source == notification_t::CLIENT ) {
      if( msg.type == notification_t::IDLE ) {

        if( info ) {
          sprintf( spstr, "handling idle notification for %s\n", thread->name );
          info->write( spstr );
        }

        thread->temporal_progress = thread->blocktill();

        blocked = true; 
        thread->idle = true;

        if( info ) {
          sprintf( spstr, "slept %s\n", thread->name );
          info->write( spstr );
        }
      } else if( msg.type == notification_t::READY ) {
        // encapsulated process has fully initialized
      } else if( msg.type == notification_t::CLOSE ) {
        // encapsulated process is dying, knows so, and had a chance to notify
        // * NO GUARANTEES THAT THIS NOTIFICATION IS RECEIVED *
        thread->invalidated = true;
      } else if( msg.type == notification_t::READ ) {
        // client needs to read data from shared memory. Service when dispatched
        thread->needs_response = true;
        thread->initialized = true;
      } else if( msg.type == notification_t::WRITE ) {
        // client has written data to shared memory.  Needs servicing.
        thread->needs_response = false;
        service_command_publication( thread );
      }
    } else if ( msg.type == notification_t::TIMER ) {
    
    } else if ( msg.type == notification_t::WAKEUP ) {
      blocked = true;
    }

    if( info ) {
      // print the notification for debugging
      char note_type[8];
      char note_source[8];
      if( msg.type == notification_t::IDLE )
        sprintf( note_type, "IDLE" );
      else if( msg.type == notification_t::READY )
        sprintf( note_type, "READY" );
      else if( msg.type == notification_t::CLOSE )
        sprintf( note_type, "CLOSE" );
      else if( msg.type == notification_t::READ )
        sprintf( note_type, "READ" );
      else if( msg.type == notification_t::WRITE )
        sprintf( note_type, "WRITE" );

      if( msg.source == notification_t::TIMER )
        sprintf( note_source, "TIMER" );
      else if( msg.source == notification_t::WAKEUP )
        sprintf( note_source, "WAKEUP" );
      else if( msg.source == notification_t::CLIENT )
        sprintf( note_source, "CLIENT" );
  
      sprintf( spstr, "handled_notification: ts[%llu]\tsource[%s]\ttype[%s]\tthread[%s]\tcomputational_progress[%llu]\ttemporal_progress[%llu]\n", msg.ts, note_source, note_type, thread->name, thread->computational_progress, thread->temporal_progress );
      info->write( spstr );
    }
  }

  if( blocked ) {
    waitqueue->push( thread );
  } else {
    runqueue->push( thread );
  }
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void term_sighandler( int signum ) {
  printf( "coordinator received SIGTERM\n" );
  //quit = true;
  //quit.store( 1, std::memory_order_seq_cst  );
  quit.store( 1, std::memory_order_relaxed  );
}

//-----------------------------------------------------------------------------
void timer_sighandler( int signum, siginfo_t *si, void *data ) {
  //std::string err, eno;

  // grab a timestamp immediately
  timer_note.ts = generate_timestamp();

  // increment accounting
  actual_timer_events++;

  ssize_t bytes_written;
  // write the timer notification to the pipe
  if( __write( timer_write_fd, &timer_note, sizeof(notification_t), bytes_written ) != OS_ERROR_NONE ) {
    // TODO: restructure to reduce complexity here.
/*
    if( errno == EPIPE )
      eno = " errno: EPIPE";
    else if( errno == EAGAIN || errno == EWOULDBLOCK )
      eno = " errno: EWOULDBLOCK";
    else if( errno == EBADF )
      eno = " errno: EBADF";
    else if( errno == EDESTADDRREQ )
      eno = " errno: EDESTADDRREQ";
    else if( errno == EDQUOT )
      eno = " errno: EDQUOT";
    else if( errno == EFAULT )
      eno = " errno: EFAULT";
    else if( errno == EFBIG )
      eno = " errno: EFBIG";
    else if( errno == EINTR )
      eno = " errno: EINTR";
    else if( errno == EINVAL )
      eno = " errno: EINVAL";
    else if( errno == EIO )
      eno = " errno: EIO";
    else if( errno == ENOSPC )
      eno = " errno: ENOSPC";

    err = "(coordinator.cpp) timer_sighandler(...) failed making system call write(...)" + eno;
    //sprintf( spstr, "(coordinator.cpp) timer_sighandler(...) failed making system call write(...) %s\n", eno );
    printf( "%s\n", err );
    // TODO : determine if there is a need to recover
*/
  }
}

//-----------------------------------------------------------------------------
void* wakeup( void* ) {
  int allow_wakeup;

  while( 1 ) {
    allow_wakeup = wakeup_enabled.load( std::memory_order_seq_cst );
    if( allow_wakeup ) {
      wakeup_note.ts = generate_timestamp();

      // disable block detection notifications.  Coordinator will reenable.
      wakeup_enabled.store( 0, std::memory_order_seq_cst );
      if( write( wakeup_write_fd, &wakeup_note, sizeof(notification_t) ) == -1 ) {
        //printf( "(coordinator.cpp) wakeup() failed making system call write(...)\n" );
        // TODO : determine if there is a need to bomb or recover
      }
    }
  }
  return NULL;
}


//-----------------------------------------------------------------------------
#include <unistd.h>
#include <fcntl.h>
bool init_pipe( const int& read_fd, const int& write_fd, bool write_blocking = false ) {
  int flags;
  int fd[2];

  if( pipe( fd ) != 0 ) {
    return false;
  }
  flags = fcntl( fd[0], F_GETFL, 0 );
  fcntl( fd[0], F_SETFL, flags );
  flags = fcntl( fd[1], F_GETFL, 0 );
  if( write_blocking )
    fcntl( fd[1], F_SETFL, flags );
  else
    fcntl( fd[1], F_SETFL, flags | O_NONBLOCK );

  if( dup2( fd[0], read_fd ) == -1 ) {
    __close( fd[0] );
    __close( fd[1] );
    return false;
  }
  if( dup2( fd[1], write_fd ) == -1 ) {
    __close( read_fd );
    __close( fd[1] );
    return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_timer_pipe( void ) {
  return init_pipe( FD_TIMER_TO_COORDINATOR_READ_CHANNEL, FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_timer_pipe( void ) {
  __close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_wakeup_pipe( void ) {
  // should be write blocking or not?
  return init_pipe( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL, FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL, true );
}

//-----------------------------------------------------------------------------
void close_wakeup_pipe( void ) {
  __close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_coordinator_to_preycontroller_pipe( void ) {
  return init_pipe( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL, FD_COORDINATOR_TO_PREYCONTROLLER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
void close_coordinator_to_preycontroller_pipe( void ) {
  __close( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREYCONTROLLER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_preycontroller_to_coordinator_pipe( void ) {
  return init_pipe( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL, FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
void close_preycontroller_to_coordinator_pipe( void ) {
  __close( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_coordinator_to_predplanner_pipe( void ) {
  return init_pipe( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL, FD_COORDINATOR_TO_PREDPLANNER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
void close_coordinator_to_predplanner_pipe( void ) {
  __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDPLANNER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_predplanner_to_coordinator_pipe( void ) {
  return init_pipe( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL, FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
void close_predplanner_to_coordinator_pipe( void ) {
  __close( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_coordinator_to_predcontroller_pipe( void ) {
  return init_pipe( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL, FD_COORDINATOR_TO_PREDCONTROLLER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
void close_coordinator_to_predcontroller_pipe( void ) {
  __close( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDCONTROLLER_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
bool init_predcontroller_to_coordinator_pipe( void ) {
  return init_pipe( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL, FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
void close_predcontroller_to_coordinator_pipe( void ) {
  __close( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool init_pipes( void ) {
  if( !init_timer_pipe() ) {
    return false;
  }

  if( !init_wakeup_pipe() ) {
    close_timer_pipe();
    return false;
  }

  if( !init_coordinator_to_preycontroller_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    return false;
  }
  if( !init_preycontroller_to_coordinator_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    return false;
  }

  if( !init_coordinator_to_predplanner_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    return false;
  }
  if( !init_predplanner_to_coordinator_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    close_coordinator_to_predplanner_pipe();
    return false;
  }

  if( !init_coordinator_to_predcontroller_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    close_coordinator_to_predplanner_pipe();
    close_predplanner_to_coordinator_pipe();
    return false;
  }
  if( !init_predcontroller_to_coordinator_pipe() ) {
    close_timer_pipe();
    close_wakeup_pipe();
    close_coordinator_to_preycontroller_pipe();
    close_preycontroller_to_coordinator_pipe();
    close_coordinator_to_predplanner_pipe();
    close_predplanner_to_coordinator_pipe();
    close_coordinator_to_predcontroller_pipe();
    return false;
  }

  subscribed_fds.push_back( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  subscribed_fds.push_back( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );

  return true;
}

//-----------------------------------------------------------------------------
void close_pipes( void ) {
  close_timer_pipe();
  close_wakeup_pipe();
  close_coordinator_to_preycontroller_pipe();
  close_preycontroller_to_coordinator_pipe();
  close_coordinator_to_predplanner_pipe();
  close_predplanner_to_coordinator_pipe();
  close_coordinator_to_predcontroller_pipe();
  close_predcontroller_to_coordinator_pipe();
}

//-----------------------------------------------------------------------------
void init( int argc, char* argv[] ) {
  //quit = false;
  quit = 0;
  //quit.store( 0, std::memory_order_seq_cst  );
  //quit.store( 0, std::memory_order_relaxed  );

  actual_timer_events = 0;
  caught_timer_events = 0;
  current_accum_plan_duration = 0.0;

  // * set variables from constants *
  cpu = DEFAULT_CPU;

  // * install SIGTERM signal handler *
  struct sigaction action;
  memset( &action, 0, sizeof(struct sigaction) );
  action.sa_handler = term_sighandler;
  sigaction( SIGTERM, &action, NULL );

  // * open log *
  log_c* pinfo = NULL;
#ifdef DO_LOGGING
  info = log_p( new log_c( "info.log", true ) );
  log_c::error_e log_err = info->allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling log_c::allocate(...).\nExiting\n" ); 
    printf( spstr );
    exit( 1 );
  }
#endif
  if( info ) pinfo = info.get();

  // summary log
  std::stringstream ss_summary;
  ss_summary << "tcs_pursuit_summary_" << EXPERIMENTAL_SEED;
  std::string summary_name = ss_summary.str();
  summary = log_p( new log_c( summary_name.c_str(), true ) );
  log_err = summary->allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( spstr, "(tcs/coordinator.cpp) failed to allocate summary\nExiting\n" );
    printf( "%s", spstr );
    if( info ) info->write( spstr );
  }

  // * get the process identifier *
  coordinator_pid = getpid( );

  sprintf( spstr, "coordinator pid: %d\n", coordinator_pid );
  if( info ) info->write( spstr );
  printf( "%s", spstr );

  // * bind the process to a single cpu *
  if( cpu_c::bind( coordinator_pid, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling cpu_c::_bind(coordinator_pid,DEFAULT_CPU).\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }

  // * set the process to be scheduled with realtime policy and max priority *
  if( scheduler_c::set_realtime_policy( coordinator_pid, coordinator_os_priority ) != scheduler_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling schedule_set_realtime_max(coordinator_pid,coordinator_priority).\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }
  sprintf( spstr, "coordinator os priority: %d\n", coordinator_os_priority );
  if( info ) info->write( spstr );
  printf( "%s", spstr );

  // * determine if the OS supports high resolution timers *
  struct timespec res;
  clock_getres( CLOCK_MONOTONIC, &res );
  double clock_res = timespec_to_real( res );

  sprintf( spstr, "clock resolution (secs): %10.9f\n", clock_res );
  if( info ) info->write( spstr );
  printf( "%s", spstr );
 
  if( res.tv_sec != 0 && res.tv_nsec != 1 ) {
    sprintf( spstr, "(coordinator.cpp) init() failed.  The host operating system does not support high resolution timers.  Consult your system documentation on enabling this feature.\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }

  // * get the cpu speed *
  if( cpu_c::get_speed( cpu_speed, cpu ) != cpu_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling cpu_c::get_frequency(cpu_speed,cpu)\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }
  sprintf( spstr, "cpu speed(hz): %llu\n", cpu_speed );
  if( info ) info->write( spstr );
  printf( "%s", spstr );

  // * initialize pipes *
  if( !init_pipes() ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling init_pipes()\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    exit( 1 );
  }

  // * initialize shared memory *
  msgbuffer = client_message_buffer_c( CLIENT_MESSAGE_BUFFER_NAME, CLIENT_MESSAGE_BUFFER_MUTEX_NAME, true );
  if( msgbuffer.open( ) != client_message_buffer_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling client_msg_buffer_c.open(...,true)\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    close_pipes( );
    exit( 1 );
  }

  // - create a processor -
  processor = boost::shared_ptr<processor_c>( new processor_c( "processor_0" ) );
  timesink_p processor_timesink = boost::dynamic_pointer_cast<timesink_c>( processor );
  processor_thread = boost::dynamic_pointer_cast<thread_c>( processor );
  processor->info = pinfo;

  // - create dynamics process -
  dynamics = boost::shared_ptr<dynamics_c>( new dynamics_c( "dynamics", processor_timesink, cpu_speed, DYNAMICS_PLUGIN, EXPERIMENTAL_SEED ) );
  dynamics->info = pinfo;
  processor->run_queue.push( dynamics );

  if( dynamics->init( argc, argv ) != dynamics_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling dynamics.init(argc,argv)\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    msgbuffer.close( );
    close_pipes( );
    exit( 1 );
  }

  if( info ) info->flush();

/*
  // * initialize block detection, i.e. wakeup * 
  wakeup_os_priority = coordinator_os_priority - 2;
  wakeup_enabled.store( 1 );
  // set up the wakeup overhead to minimize what changes inside
  wakeup_note.source = notification_t::WAKEUP;
  wakeup_write_fd = FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL;

  if( scheduler_c::create( wakeup_thread, wakeup_os_priority, wakeup ) != scheduler_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling wakeup_thread.create(...,wakeup)\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
    msgbuffer.close( );
    close_pipes( );
    exit( 1 );
  }
  wakeup_created = true;
*/
/*
  if( scheduler_c::get_priority( wakeup_thread, wakeup_os_priority ) != scheduler_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling get_priority(wakeup_thread,...)\nExiting\n" );
    info.write( spstr );
    msgbuffer.close( );
    close_pipes( );
    // kill the wakeup thread?
    exit(1);
  }
  sprintf( spstr, "wakeup thread created... priority:%d\n", wakeup_os_priority );
  if( info ) info->write( spstr );
  printf( "%s", spstr );
*/
  // * initialize clients *
  sprintf( spstr, "initializing clients\n" );
  if( info ) info->write( spstr );
  printf( "%s", spstr );

  scheduler_c::error_e schedulererr;

  //double controller_floor_seconds = 0.001;
  //double controller_ceiling_seconds = 0.001;
  //double planner_floor_seconds = 0.01;
  //double planner_ceiling_seconds = 0.5;
  //std::string prey_controller_seed = "1";
  //std::string pred_controller_seed = "2";
  //std::string planner_seed = "1";

  char numbuf[16];

  //sprintf(numbuf,"%llu",seconds_to_cycles(controller_floor_seconds, cpu_speed));
  //std::string controller_floor = numbuf;
  //sprintf(numbuf,"%llu",seconds_to_cycles(controller_ceiling_seconds, cpu_speed));
  //std::string controller_ceiling = numbuf;
  //sprintf(numbuf,"%llu",seconds_to_cycles(planner_floor_seconds, cpu_speed));
  //std::string planner_floor = numbuf;
  //sprintf(numbuf,"%llu",seconds_to_cycles(planner_ceiling_seconds, cpu_speed));
  //std::string planner_ceiling = numbuf;

  CLIENT_OS_MAX_PRIORITY = coordinator_os_priority - 1;
  CLIENT_OS_MIN_PRIORITY = coordinator_os_priority - 3;
  int client_os_priority_step = CLIENT_OS_MAX_PRIORITY - CLIENT_OS_MIN_PRIORITY;

  printf( "client max_priority:%d min_priority:%d step:%d\n", CLIENT_OS_MAX_PRIORITY, CLIENT_OS_MIN_PRIORITY, client_os_priority_step );

  // - create prey processes -
  prey = boost::shared_ptr<timesink_c>( new timesink_c( "prey", processor_timesink, scheduler_c::PROGRESS) );
  prey->info = pinfo;
  //prey->priority = 0;
  processor->run_queue.push( prey );
  prey_thread = boost::dynamic_pointer_cast<thread_c>(prey);

  // TODO : change the create interface to pass the xml file in args to the controller (and others when they are ready)

  prey_controller = boost::shared_ptr<osthread_c>( new osthread_c( "prey_controller", prey, &select, &read_notifications, &process_notifications, &service_state_request, pinfo ) );
  prey_controller->_max_os_priority = CLIENT_OS_MAX_PRIORITY;
  prey_controller->_min_os_priority = CLIENT_OS_MIN_PRIORITY;
  prey_controller->_os_priority_step = client_os_priority_step;
  prey_controller->_cpu_speed = cpu_speed;
  prey_controller->priority = 0;
  prey_controller->desired_period_realtime = PREY_CONTROLLER_PERIOD_SEC;
  //printf( "controller_ceiling_seconds[%f], prey_controller->desired_period_realtime[%f]\n", controller_ceiling_seconds, prey_controller->desired_period_realtime );
  prey_controller->desired_period_cycles = seconds_to_cycles( prey_controller->desired_period_realtime, cpu_speed );
  prey_controller->next_desired_activation_time += prey_controller->desired_period_cycles;
  prey_controller->last_activation_time = 0;

  std::stringstream ss_prey_controller_timelog;
  ss_prey_controller_timelog << "tcs_pursuit_prey_controller_time_trial_" << EXPERIMENTAL_SEED ;
  std::string prey_controller_timelog_name = ss_prey_controller_timelog.str();
  prey_controller->timelog = log_c( prey_controller_timelog_name.c_str(), true );
  log_err = prey_controller->timelog.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( spstr, "(tcs/coordinator.cpp) failed to allocate prey_controller->timelog\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
  }
  prey->run_queue.push( prey_controller );

  schedulererr = scheduler_c::create( prey_controller, 3, DEFAULT_CPU, "pursuit-tcs-prey-controller", argc[argv-1], suppress_client_console_io );
  //schedulererr = scheduler_c::create( prey_controller, 3, DEFAULT_CPU, "pursuit-prey-controller", argc[argv-1], false );
  //prey_controller->block();

  sprintf( spstr, "created prey-controller: pid[%d], _os_priority[%d], _os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", prey_controller->pid, prey_controller->_os_priority, prey_controller->_os_priority_step, prey_controller->_max_os_priority, prey_controller->_min_os_priority );
  if( info ) info->write( spstr );
//
  // - create predator processes -
  pred = boost::shared_ptr<timesink_c>( new timesink_c( "pred", processor_timesink, scheduler_c::PRIORITY) );
  pred->info = pinfo;
  //prey->priority = 0;
  processor->run_queue.push( pred );
  pred_thread = boost::dynamic_pointer_cast<thread_c>(pred);

  // predator controller
  pred_controller = boost::shared_ptr<osthread_c>( new osthread_c( "pred_controller", pred, &select, &read_notifications, &process_notifications, &service_state_request, pinfo ) );
  pred_controller->_max_os_priority = CLIENT_OS_MAX_PRIORITY;
  pred_controller->_min_os_priority = CLIENT_OS_MIN_PRIORITY;
  pred_controller->_os_priority_step = client_os_priority_step;
  pred_controller->_cpu_speed = cpu_speed;
  pred_controller->priority = 1;
  pred_controller->desired_period_realtime = PREDATOR_CONTROLLER_PERIOD_SEC;
  pred_controller->desired_period_cycles = seconds_to_cycles( pred_controller->desired_period_realtime, cpu_speed );
  pred_controller->next_desired_activation_time += pred_controller->desired_period_cycles;
  pred_controller->last_activation_time = 0;

  std::stringstream ss_pred_controller_timelog;
  ss_pred_controller_timelog << "tcs_pursuit_pred_controller_time_trial_" << EXPERIMENTAL_SEED ;
  std::string pred_controller_timelog_name = ss_pred_controller_timelog.str();
  pred_controller->timelog = log_c( pred_controller_timelog_name.c_str(), true );
  log_err = pred_controller->timelog.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( spstr, "(tcs/coordinator.cpp) failed to allocate pred_controller->timelog\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
  }
  pred->run_queue.push( pred_controller );

  schedulererr = scheduler_c::create( pred_controller, 3, DEFAULT_CPU, "pursuit-tcs-pred-controller", argc[argv-1], suppress_client_console_io );
  //schedulererr = scheduler_c::create( pred_controller, 3, DEFAULT_CPU, "pursuit-pred-controller", argc[argv-1], false );
  //pred_controller->block();

  sprintf( spstr, "created pred-controller: pid[%d], _os_priority[%d], _os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", pred_controller->pid, pred_controller->_os_priority, pred_controller->_os_priority_step, pred_controller->_max_os_priority, pred_controller->_min_os_priority );
  if( info ) info->write( spstr );

  // predator planner
  pred_planner = boost::shared_ptr<osthread_c>( new osthread_c( "pred_planner", pred, &select, &read_notifications, &process_notifications, &service_state_request, pinfo ) );
  pred_planner->_max_os_priority = CLIENT_OS_MAX_PRIORITY;
  pred_planner->_min_os_priority = CLIENT_OS_MIN_PRIORITY;
  pred_planner->_os_priority_step = client_os_priority_step;
  pred_planner->_cpu_speed = cpu_speed;
  pred_planner->priority = 0;
  pred_planner->desired_period_realtime = 0;
  pred_planner->desired_period_cycles = 0;
  //pred_planner->next_desired_activation_time += pred_planner->desired_period_cycles;
  //pred_controller->last_activation_time = 0;

  std::stringstream ss_pred_planner_timelog;
  ss_pred_planner_timelog << "tcs_pursuit_pred_planner_time_trial_" << EXPERIMENTAL_SEED ;
  std::string pred_planner_timelog_name = ss_pred_planner_timelog.str();
  pred_planner->timelog = log_c( pred_planner_timelog_name.c_str(), true );
  log_err = pred_planner->timelog.allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( spstr, "(tcs/coordinator.cpp) failed to allocate pred_planner->timelog\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );
  }
  pred->run_queue.push( pred_planner );
  no_plan = true;

  schedulererr = scheduler_c::create( pred_planner, 3, DEFAULT_CPU, "pursuit-tcs-pred-planner", argc[argv-1], suppress_client_console_io );
  //schedulererr = scheduler_c::create( pred_planner, 3, DEFAULT_CPU, "pursuit-pred-planner", argc[argv-1], false );
  //pred_planner->block();

  sprintf( spstr, "created pred-planner: pid[%d], _os_priority[%d], _os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", pred_planner->pid, pred_planner->_os_priority, pred_planner->_os_priority_step, pred_planner->_max_os_priority, pred_planner->_min_os_priority );
  if( info ) info->write( spstr );

  // * initialize timer *
  // set up the timer handler overhead to minimize what changes inside
  timer_note.source = notification_t::TIMER;
  timer_note.type = notification_t::WRITE;
  timer_write_fd = FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL;

  // create the timer
  if( timer.create( timer_sighandler, RTTIMER_SIGNAL ) != timer_c::ERROR_NONE ) {
    sprintf( spstr, "(coordinator.cpp) init() failed calling timer.create(timer_sighandler,RTTIMER_SIGNAL)\nExiting\n" );
    if( info ) info->write( spstr );
    printf( "%s", spstr );

    kill_wakeup;

    kill_prey_controller();
    kill_pred_controller();
    kill_pred_planner();

    msgbuffer.close( );
    close_pipes( );
    exit( 1 );
  }

  // * initialize other resources *
  serviced_read_note.source = notification_t::SERVER;
  serviced_read_note.type = notification_t::READ;      

  // lock into memory to minimize pagefaults.  do last before main loop
  mlockall( MCL_CURRENT );
} 

//-----------------------------------------------------------------------------
void shutdown( void ) {
  // unlock memory.  do before any other shutdown operations
  munlockall();

/*
  // kill the wakeup thread
  kill_wakeup( );
*/

  // kill the timer
  kill_timer();

  // kill the clients
  kill_prey_controller();
  kill_pred_controller();
  kill_pred_planner();

  // shutdown dynamics
  dynamics->shutdown();

  // close the shared buffer
  msgbuffer.close();

  // close the pipes
  close_pipes();

  // write any last statistics
  sprintf( spstr, "timer events: actual[%d], caught[%u]\n", actual_timer_events, caught_timer_events );
  if( info ) info->write( spstr );
  printf( spstr );

  sprintf( spstr, "coordinator shutdown\n" );
  if( info ) info->write( spstr );
  printf( spstr );

  // ensure the log gets written then destroyed
  if( info ) info->flush();
  if( info ) info->deallocate();
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] ) {
  // Due to realtime scheduling, et al, must have root access to run.
  if( geteuid() != 0 ) {
    sprintf( spstr, "This program requires root access.  Re-run with sudo.\nExiting\n" );
    printf( "%s", spstr );
    exit( 1 );
  }

  init( argc, argv );

  // last before main loop, arm the timer
  timer.arm( timer_c::PERIODIC, TIMER_PERIOD_NSECS );

  //while( !quit.load( std::memory_order_seq_cst ) ) {
  //while( !quit ) {
  while( !quit.load( std::memory_order_relaxed ) ) {
    scheduler_c::step_system( processor_thread, current_thread, processor->run_queue, processor->block_queue, info.get() );
  }

  shutdown();

  return 0;
}

