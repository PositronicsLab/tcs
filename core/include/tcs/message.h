#ifndef _MESSAGE_H_
#define _MESSAGE_H_

//-----------------------------------------------------------------------------
#include <tcs/time.h>
#include <tcs/tcs.h>
//-----------------------------------------------------------------------------
#define CLIENT_MESSAGE_BUFFER_NAME "client_message_buffer"
#define CLIENT_MESSAGE_BUFFER_MUTEX_NAME "client_message_buffer_mutex"

//-----------------------------------------------------------------------------

struct client_message_header_t {
  timestamp_t ts;
  realtime_t t;
  realtime_t dt; 
};

//-----------------------------------------------------------------------------
struct state_t {
  timestamp_t ts;
  double q[7];
  double dq[6]; 
};

//-----------------------------------------------------------------------------
struct control_t {
  timestamp_t ts;
  double u[6];
  double duration;
};

//-----------------------------------------------------------------------------
struct plan_t {
  timestamp_t ts;
  bool failed;
  bool updated;
  unsigned count;
  control_t control[MAX_PLANNER_CONTROLS];
};

//-----------------------------------------------------------------------------
class client_message_t {
public:
  //client_message_c( void );
  //virtual ~client_message_c( void );

  client_message_header_t header;
  state_t prey_state;
  state_t pred_state;
  control_t control;
  plan_t plan;
};

//-----------------------------------------------------------------------------
#include <tcs/os.h>
#include <string>

class client_message_buffer_c {
private:
  bool                  _create;
  bool                  _defined;
  bool                  _open;

  std::string           _buffer_name;
  int                   _fd_buffer;
  client_message_t*     _buffer;  

  mutex_c               _mutex;

public:
  //---------------------------------------------------------------------------
  enum buffer_error_e {
    ERROR_NONE = 0,
    ERROR_MAPPING,
    ERROR_UNMAPPING,
    ERROR_OPENING,
    ERROR_TRUNCATING,
    ERROR_UNLINKING,
    ERROR_SYNCING,
    ERROR_MUTEX
  };

  //---------------------------------------------------------------------------
  client_message_buffer_c( void );
  client_message_buffer_c( const char* buffer_name, const char* mutex_name, const bool& create );

  //---------------------------------------------------------------------------
  virtual ~client_message_buffer_c( void );

  //---------------------------------------------------------------------------
private:
  buffer_error_e init( void );
  buffer_error_e destroy( void );

  //---------------------------------------------------------------------------
public:
  buffer_error_e open( void );
  void close( void );
  buffer_error_e write( const client_message_t& m, bool write_plan = false );
  buffer_error_e read( client_message_t& m, bool read_plan = false );

};

//-----------------------------------------------------------------------------

#endif // _MESSAGE_H_
