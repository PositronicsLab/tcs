#include <tcs/osthread.h>

#include <tcs/time.h>
#include <tcs/scheduler.h>
//#include <sstream>
#include <stdio.h>

// TODO: Add constructor that includes all the ancillary and potentially private
//       data
//-----------------------------------------------------------------------------
osthread_c::osthread_c( const char* name ) : 
  timesink_c( name ) 
{
  assert( scheduler_c::get_realtime_min_priority(_min_os_priority) == scheduler_c::ERROR_NONE );
  assert( scheduler_c::get_realtime_max_priority(_max_os_priority) == scheduler_c::ERROR_NONE );
  _os_priority_step = _max_os_priority - _min_os_priority;

  sprintf( this->name, "%s", name );
  temporal_progress = 0;
  computational_progress = 0;
  wait_time = 0;
  prev_computational_progress = 0;

  desired_period_cycles = 0;
  desired_period_realtime = 0;
  next_desired_activation_time = 0;
  last_activation_time = 0;

  invalidated = false;
  idle = false;
  needs_response = false;
}

//-----------------------------------------------------------------------------
osthread_c::osthread_c( const char* name, const timesink_p& owner ) : 
  timesink_c( name, owner, scheduler_c::PRIORITY ) 
{
  assert( scheduler_c::get_realtime_min_priority(_min_os_priority) == scheduler_c::ERROR_NONE );
  assert( scheduler_c::get_realtime_max_priority(_max_os_priority) == scheduler_c::ERROR_NONE );
  _os_priority_step = _max_os_priority - _min_os_priority;

  //sprintf( this->name, "%s", name );
  //this->owner = owner;

  temporal_progress = 0;
  computational_progress = 0;
  wait_time = 0;
  prev_computational_progress = 0;

  desired_period_cycles = 0;
  desired_period_realtime = 0;
  next_desired_activation_time = 0;
  last_activation_time = 0;

  invalidated = false;
  enqueued = false;
  idle = false;
  needs_response = false;
}

//-----------------------------------------------------------------------------
osthread_c::osthread_c( const char* name, const timesink_p& owner, select_f select, read_notifications_f read_notifications, process_notifications_f process_notifications, service_state_request_f service_state_request, log_c* info = NULL ) : 
  timesink_c( name, owner, scheduler_c::PRIORITY ) 
{
  //sprintf( this->name, "%s", name );
  this->select = select;
  this->read_notifications = read_notifications;
  this->process_notifications = process_notifications;
  this->service_state_request = service_state_request;
  _info = info;

  assert( scheduler_c::get_realtime_min_priority(_min_os_priority) == scheduler_c::ERROR_NONE );
  assert( scheduler_c::get_realtime_max_priority(_max_os_priority) == scheduler_c::ERROR_NONE );
  _os_priority_step = _max_os_priority - _min_os_priority;

//  this->owner = owner;

  temporal_progress = 0;
  computational_progress = 0;
  wait_time = 0;
  prev_computational_progress = 0;

  desired_period_cycles = 0;
  desired_period_realtime = 0;
  next_desired_activation_time = 0;
  last_activation_time = 0;

  invalidated = false;
  enqueued = false;
  idle = false;
  needs_response = false;
}

//-----------------------------------------------------------------------------
osthread_c::~osthread_c( void ) {

}

//-----------------------------------------------------------------------------
cycle_t osthread_c::blocktill( void ) {
  if( desired_period_cycles > 0 ) {
    //assert( temporal_progress < next_desired_activation_time );
    return next_desired_activation_time;
  } else {
    return computational_progress;
  }
}

//-----------------------------------------------------------------------------
void osthread_c::dispatch( thread_p& current_thread ) {
  char spstr[512];
  osthread_p current_osthread, this_osthread;
  thread_p this_thread;
  timestamp_t ts_before, ts_after; 
  timestamp_t dts;

  // get a reference to the thread about to be dispatched
  this_thread = shared_from_this();
  // get an osthread pointer to the thread about to be dispatched
  this_osthread = boost::dynamic_pointer_cast<osthread_c>( this_thread );

  //--------
  // check if current thread has a valid reference (otherwise was nothing)
  if( current_thread ) {
    // sanity check
    if( current_thread->type() == thread_c::OSTHREAD ) {

      // get an osthread pointer to the thread previously dispatched
      current_osthread = boost::dynamic_pointer_cast<osthread_c>(current_thread);

/*
      if( _info != NULL ) {
        sprintf( spstr, "lowering priority on %s: pid[%d], _os_priority[%d], _os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", current_osthread->name, current_osthread->pid, current_osthread->_os_priority, current_osthread->_os_priority_step, current_osthread->_max_os_priority, current_osthread->_min_os_priority );
        _info->write( spstr );
      }
      // lower the priority of the current thread
      //current_osthread->lower_priority();
      if( _info != NULL ) {
        sprintf( spstr, "priority of %s: pid[%d], _os_priority[%d], _os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", current_osthread->name, current_osthread->pid, current_osthread->_os_priority, current_osthread->_os_priority_step, current_osthread->_max_os_priority, current_osthread->_min_os_priority );
        _info->write( spstr );
      }
*/
      // log information about the switch
      if( _info != NULL ) {
        if( current_thread != this_thread ) {
          sprintf( spstr, "switching between current[%s] and next[%s] threads\n", current_thread->name, this_thread->name );
          _info->write( spstr ); 
          //printf( "%s", spstr );
        }
      }
    }
  }

  // set the current thread as the one being dispatched
  current_thread = this_thread;

  // flush log here as the time accounting will not be attributed to clients
  if( _info != NULL ) {
    if( _info->size() >= _info->capacity() / 2 ) 
      _info->flush();
  }

  if( _info != NULL ) {
    sprintf( spstr, "raising priority on %s: pid[%d], _os_priority[%d], _os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", this_osthread->name, this_osthread->pid, this_osthread->_os_priority, this_osthread->_os_priority_step, this_osthread->_max_os_priority, this_osthread->_min_os_priority );
    _info->write( spstr );
  }
 
  if( this_osthread->needs_response ) {
    //printf( "servicing state request\n" );
    service_state_request( this_osthread );
    this_osthread->needs_response = false;
  }

  if( this_osthread->idle ) {
    //sprintf( spstr, "activating idle thread %s: desired_period[%llu], last_activation_time[%llu], next_desired_activation_time[%llu]\n", this_osthread->name, this_osthread->desired_period, this_osthread->last_activation_time, this_osthread->next_desired_activation_time );
    //_info->write( spstr );
    //printf( "%s", spstr );

    this_osthread->next_desired_activation_time += this_osthread->desired_period_cycles;
    this_osthread->last_activation_time = this_osthread->temporal_progress;
    this_osthread->idle = false;

    //sprintf( spstr, "idle thread %s activated: desired_period[%llu], last_activation_time[%llu], next_desired_activation_time[%llu]\n", this_osthread->name, this_osthread->desired_period, this_osthread->last_activation_time, this_osthread->next_desired_activation_time );
    //_info->write( spstr );
    //printf( "%s", spstr );
  }

  // raise the priority of the thread being dispatched
  this_osthread->raise_priority();
/*
  // * TODO: Remove *
  if( _info != NULL ) {
    //sprintf( spstr, "dispatching %s: pid[%d], _os_priority[%d], _os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", this_osthread->name, this_osthread->pid, this_osthread->_os_priority, this_osthread->_os_priority_step, this_osthread->_max_os_priority, this_osthread->_min_os_priority );
    sprintf( spstr, "dispatching %s at priority %d\n", this_osthread->name, this_osthread->_os_priority );
    _info->write( spstr );
  }

  // * TODO: Remove *
  if( _info != NULL ) {
    sprintf( spstr, "selecting\n" );
    _info->write( spstr );
  }
*/
  //this_osthread->unblock();
  //ts_dispatched = generate_timestamp();
  ts_before = generate_timestamp();
  while( !select() );  // Note: select may be interrupted -> loop till success
  ts_after = generate_timestamp();

  //this_osthread->block();
  this_osthread->lower_priority();
/*
  ts_delta = ts_after - ts_dispatched;

  computational_progress += ts_delta;  // correct
*/
  //dts = ts_after - ts_dispatched;
  dts = ts_after - ts_before;

  if(this_osthread->initialized) {
    temporal_progress += dts;
    computational_progress += dts;
  }

  //temporal_progress += ts_delta;       // ?Correct?
  //temporal_progress += desired_period;    // true only if the controller finishes a computation and is not interrupted due to blocking events.

/*
  if( _info != NULL ) {
    sprintf( spstr, "%s preempted(osthread.cpp): computational_progress[%llu], temporal_progress[%llu]\n", name, computational_progress, temporal_progress );
    _info->write( spstr );
  }
*/
  //--------
  if( _info != NULL ) {
    sprintf( spstr, "reading notifications\n" );
    _info->write( spstr );
  }

  read_notifications();
}
/*
//-----------------------------------------------------------------------------
void osthread_c::terminate( void ) {
  assert( true );     // for now we don't want to allow this
  // TODO: add a notification based means to terminate
}
*/
//-----------------------------------------------------------------------------
void osthread_c::raise_priority( void ) {

  //if( _os_priority < _max_os_priority ) {
    scheduler_c::error_e error;
    error = scheduler_c::increase_realtime_priority( pid, _os_priority, _os_priority_step, _max_os_priority );
    if( error != scheduler_c::ERROR_NONE ) {
      // TODO: how to recover?
      printf( "failed to increase_priority\n" );
    }
  //}
}

//-----------------------------------------------------------------------------
void osthread_c::lower_priority( void ) {

  //if( _os_priority > _min_os_priority ) {
    scheduler_c::error_e error;
    error = scheduler_c::decrease_realtime_priority( pid, _os_priority, _os_priority_step, _min_os_priority );
    if( error != scheduler_c::ERROR_NONE ) {
      // TODO: how to recover?
      printf( "failed to decrease_priority: " );
      if( error == scheduler_c::ERROR_PARAM_QUERY ) 
        printf( "ERROR_PARAM_QUERY\n" );
      else if( error == scheduler_c::ERROR_PRIORITY_BOUNDS ) 
        printf( "ERROR_PRIORITY_BOUNDS_os_priority_step[%d], _max_os_priority[%d], _min_os_priority[%d]\n", _os_priority_step, _max_os_priority, _min_os_priority );
      else if( error == scheduler_c::ERROR_PARAM_UPDATE ) 
        printf( "ERROR_PARAM_UPDATE\n" );
      
    }
  //}
}

//-----------------------------------------------------------------------------
void osthread_c::block( void ) {
  kill( pid, SIGSTOP );
}

//-----------------------------------------------------------------------------
void osthread_c::unblock( void ) {
  kill( pid, SIGCONT );
}

//-----------------------------------------------------------------------------
