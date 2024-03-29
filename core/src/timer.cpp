#include <tcs/timer.h>

#include <assert.h>
#include <tcs/time.h>  // API time
#include <time.h>  // POSIX time

//#include <stdio.h>

//-----------------------------------------------------------------------------
/// Default contructor.
timer_c::timer_c( void ) {
  _first_arming = true;
  agg_error = 0;
  last_overrun = 0;
  ts_prev_arm = 0;
  _signum = -1;
}

//-----------------------------------------------------------------------------
/// Destructor.
timer_c::~timer_c( void ) {

}

//-----------------------------------------------------------------------------
/// Blocks the timer signal if it is necessary to suppress the timer using
/// POSIX system call.
/// @return indicator of operation success or specified error.
timer_c::error_e timer_c::block( void ) {
  if( sigprocmask( SIG_SETMASK, &_mask, NULL ) == -1 )
    return ERROR_SIGPROCMASK;
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Unblocks a blocked timer signal using POSIX system call.
/// @return indicator of operation success or specified error.
timer_c::error_e timer_c::unblock( void ) {
  if( sigprocmask( SIG_UNBLOCK, &_mask, NULL ) == -1 )
    return ERROR_SIGPROCMASK;
  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Creates a high-resolution realtime timer to monitor the controller process.
/// @param sighandler the timer function called when a timer fires.
/// @param signum the signal identifier to attach the timer to.  For realtime 
/// timers, it must be in the range define by SIGRTMIN to SIGRTMAX.
/// @return indicator of operation success or specified error.
timer_c::error_e timer_c::create( sighandler_f sighandler, int signum ) {
  struct sigevent sevt;
  struct sigaction action;

  assert( signum > 0 );

  // Establish handler for timer signal
  action.sa_flags = SA_SIGINFO;
  action.sa_sigaction = sighandler;
  sigemptyset( &action.sa_mask );
  if( sigaction( signum, &action, NULL ) == -1)
    return ERROR_SIGACTION;

  // intialize the signal mask
  sigemptyset( &_mask );
  sigaddset( &_mask, signum );

  sevt.sigev_notify = SIGEV_SIGNAL;
  sevt.sigev_signo = signum;
  sevt.sigev_value.sival_ptr = &_id;
  if( timer_create( CLOCK_MONOTONIC, &sevt, &_id ) == -1 )
    return ERROR_CREATE;

  _signum = signum;

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Destroys an already created timer.
void timer_c::destroy( void ) {
  struct sigaction action;

  assert( _signum > 0 );

  timer_delete( _id );
  // timer_delete returns -1 on failure, errno{EINVAL}

  action.sa_handler = SIG_DFL;
  action.sa_mask = _mask; 

  if( sigaction( _signum, &action, NULL ) == -1) {
    //return ERROR_SIGACTION;
  }
}

//-----------------------------------------------------------------------------
/// Arms a realtime timer.
/// @param type defines whether to arm as a one-shot or periodic timer.
/// @param period_nsec the period in nanoseconds that the timer is requested
/// to have.
/// @return indicator of operation success or specified error.
#include <stdio.h>
timer_c::error_e timer_c::arm( const type_e& type, const unsigned long long& period_nsec ) {
  struct itimerspec its;
  struct timespec tspec = nanoseconds_to_timespec( period_nsec );

  //printf( "timespec: %d, %d\n", tspec.tv_sec, tspec.tv_nsec );

  if( type == ONESHOT ) {
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = tspec.tv_sec;
    its.it_value.tv_nsec = tspec.tv_nsec;
  } else if( type == PERIODIC ) {
    its.it_interval.tv_sec = tspec.tv_sec;
    its.it_interval.tv_nsec = tspec.tv_nsec;
    its.it_value.tv_sec = tspec.tv_sec;
    its.it_value.tv_nsec = tspec.tv_nsec;
  }

  if( timer_settime( _id, 0, &its, NULL ) == -1 ) 
    return ERROR_SETTIME;

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
/// Arms the realtime timer using error correction.  This code was inherited 
/// from initial pendulum experiment.  Retesting has demonstrated that as it
/// is here, its performance is circumspec and needs deeper evaluation.  So
/// while this text is here, use at own risk.
/// @param type defines whether to arm as a one-shot or periodic timer.
/// @param period_nsec the period in nanoseconds that the timer is requested
/// to have.
/// @param ts_req the requested time for the timer to arm.
/// @param ts_arm a timestamp that is generated when the timer is armed.
/// @param cpu_hz the speed of the processor.
/// @return indicator of operation success or specified error.
timer_c::error_e timer_c::arm( const type_e& type, const unsigned long long& period_nsec, const timestamp_t& ts_req, timestamp_t& ts_arm, const cpu_speed_t& cpu_hz ) {
  struct itimerspec its;
  timestamp_t ts_now;
  unsigned long long ts_err = 0;
  unsigned long long ns_err = 0;
  unsigned long long period = period_nsec;

  ts_now = generate_timestamp();

  if( _first_arming ) {
    // cannot compute err correction
    _first_arming = false;
  } else {
    ts_err = ts_now - ts_prev_arm;
    ns_err = cycles_to_nanoseconds( ts_err, cpu_hz );
    long long error = (long long) period - (long long) ns_err;

    agg_error += error;
    period += agg_error;
  }

  struct timespec tspec = nanoseconds_to_timespec( period );
  //printf( "period[%llu]: secs[%d], nsecs[%d]\n", period, tspec.tv_sec, tspec.tv_nsec );

  if( type == ONESHOT ) {
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = tspec.tv_sec;
    its.it_value.tv_nsec = tspec.tv_nsec;
  } else if( type == PERIODIC ) {
    its.it_interval.tv_sec = tspec.tv_sec;
    its.it_interval.tv_nsec = tspec.tv_nsec;
    its.it_value.tv_sec = tspec.tv_sec;
    its.it_value.tv_nsec = tspec.tv_nsec;
  }

  ts_arm = ts_req;
  ts_prev_arm = ts_now;

  if( timer_settime( _id, 0, &its, NULL ) == -1 ) {
    //printf( "dts: %lld, dns: %lld, nsec: %d\n", dts, dns, its.it_value.tv_nsec );
    return ERROR_SETTIME;
  }

  return ERROR_NONE;
}

/*
//-----------------------------------------------------------------------------
/// Arms a realtime timer.
/// @param type defines whether to arm as a one-shot or periodic timer.
/// @param period_sec the period in seconds (double) that the timer is requested
/// to have.
/// @return indicator of operation success or specified error.
timer_c::error_e timer_c::arm( const type_e& type, realtime_t period_sec ) {
  struct itimerspec its;
  struct timespec tspec = realtime_to_timespec( period_sec );

  printf( "timespec: %d, %d\n", tspec.tv_sec, tspec.tv_nsec );

  if( type == ONESHOT ) {
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = tspec.tv_sec;
    its.it_value.tv_nsec = tspec.tv_nsec;
  } else if( type == PERIODIC ) {
    its.it_interval.tv_sec = tspec.tv_sec;
    its.it_interval.tv_nsec = tspec.tv_nsec;
    its.it_value.tv_sec = tspec.tv_sec;
    its.it_value.tv_nsec = tspec.tv_nsec;
  }

  if( timer_settime( _id, 0, &its, NULL ) == -1 ) 
    return ERROR_SETTIME;

  return ERROR_NONE;
}
*/
//-----------------------------------------------------------------------------
