#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include <tcs/timesink.h>
#include <tcs/time.h>
#include <tcs/dynamics_data.h>
#include <tcs/dynamics_plugin.h>
#include <tcs/log.h>

//-----------------------------------------------------------------------------
class dynamics_c : public timesink_c {
private:
  dynamics_plugin_c _plugin;
  std::string _filename;

  realtime_t _step_size_realtime;
  cycle_t _step_size_cycles;

  log_c log;

  bool _seeded;
  bool _simulated;
public:

  enum error_e {
    ERROR_NONE,
    ERROR_READ,
    ERROR_INIT
  };

  dynamics_c( const char* name, const timesink_p& owner, const cpu_speed_t& cpu_speed, const char* plugin, int log_index );
  virtual ~dynamics_c( void );

  virtual type_e type( void ) { return DYNAMICS; }

  virtual void dispatch( thread_p& current_thread );
  virtual void terminate( void );

  void step( void );
  void step( const cycle_t& dt );

  // Need to maintain a state/command history here.
  dynamics_data_t buffer;

  void flush_log( void );

  // Encapsulate the plugin here and all interfaces with
  error_e init( int argc, char* argv[] );
  void shutdown( void );

  cycle_t step_size_cycles( void );
  realtime_t step_size_realtime( void );

private:
  char spstr[512];
  void log_buffer( void );

};

//-----------------------------------------------------------------------------

#endif // _DYANMICS_H_
