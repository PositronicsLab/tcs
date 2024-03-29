/*-----------------------------------------------------------------------------
timer.h encapsulates all POSIX realtime timer functionality 

author: James Taylor : jrt@gwu.edu
-----------------------------------------------------------------------------*/

#ifndef _TIMER_H_
#define _TIMER_H_

//-----------------------------------------------------------------------------

#include <tcs/time.h>
#include <tcs/cpu.h>

#include <signal.h>

//-----------------------------------------------------------------------------

// The signal identifier for the controller's real-time timer
#define RTTIMER_SIGNAL                          SIGRTMIN + 4

//-----------------------------------------------------------------------------

/// Signalhandler function pointer type for use with timers.
typedef void (*sighandler_f)( int signum, siginfo_t *si, void *data );

//-----------------------------------------------------------------------------

class timer_c {
private:
  sigset_t            _mask;        // POSIX signal mask
  int                 _signum;
  timer_t             _id;          // timer identifier
  bool                _first_arming;        // has this timer armed before

public:
  long long agg_error;                ///< Aggregate error of the timer
  unsigned long long last_overrun;    ///< The amount of the last overrun
  timestamp_t ts_prev_arm;            ///< The time of last arming

  /// The set of timer operation errors.
  enum error_e {
    ERROR_NONE = 0,          ///< Operation completed successfully.
    ERROR_SIGACTION,         ///< A failure to set the signal action.
    ERROR_SIGPROCMASK,       ///< A failure to set the signal mask.
    ERROR_CREATE,            ///< A failure to create the timer.
    ERROR_SETTIME,           ///< A failure to set the time.
    ERROR_GETCLOCKID         ///< A failure to get the clock id.
  };

  enum type_e {
    ONESHOT,
    PERIODIC
  };

  timer_c( void );
  virtual ~timer_c( void ); 

  error_e block( void );
  error_e unblock( void );

  error_e create( sighandler_f sighandler, int signum );
  void destroy( void );

  error_e arm( const type_e& type, const unsigned long long& period_nsec );
  error_e arm( const type_e& type, const unsigned long long& period_nsec, const timestamp_t& ts_req, timestamp_t& ts_arm, const cpu_speed_t& cpu_hz );
//  error_e arm( const type_e& type, realtime_t period_nsec );

};

//-----------------------------------------------------------------------------


#endif // _TIMER_H_
