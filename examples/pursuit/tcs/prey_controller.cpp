#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <atomic>

#include <unistd.h>
#include <signal.h>

#include <tcs/log.h>
#include <tcs/time.h>
#include <tcs/os.h>
#include <tcs/notification.h>
#include <tcs/message.h>

#include <tcs/channels.h>

#include <tcs/osthread.h>

#include "../experiment.h"
#include "../moby_xml.h"
#include "../moby_space.h"
#include "../moby_ship.h"

#include <errno.h>

//-----------------------------------------------------------------------------
log_p info;
char spstr[512];

//-----------------------------------------------------------------------------
// Local helpers
static osthread_p     mythread;
//std::atomic<int>      quit;
static bool           quit;

//-----------------------------------------------------------------------------
// Notification Overhead
static int            write_fd;
static int            read_fd;

notification_t        command_note;
notification_t        state_note;
notification_t        yield_note;
notification_t        server_note;

//-----------------------------------------------------------------------------

realtime_t t;
realtime_t dt;

//-----------------------------------------------------------------------------

client_message_t msg;
client_message_buffer_c msgbuffer;

//-----------------------------------------------------------------------------

moby_space_p space;
ship_p prey;
ship_p predator;

std::vector<double> prey_state;
std::vector<double> pred_state;
std::vector<double> prey_control;

//-----------------------------------------------------------------------------
// read( blocks process ) the notification sent back from the coordinator
void read( notification_t& note ) {
  if( __read( read_fd, &note, sizeof(notification_t) ) == -1 ) {
    // ERROR
    //return false;
  }
}

//-----------------------------------------------------------------------------
void write( notification_t& note ) {
  char eno[16];

  // update the timestamp on the note
  note.ts = generate_timestamp();
  ssize_t bytes_written;

  os_error_e result;
  result = __write( write_fd, &note, sizeof(notification_t), bytes_written );
  if( result != OS_ERROR_NONE ) {
///*
    if( result == OS_ERROR_PIPE )
      sprintf( eno, "EPIPE" );
    else if( result == OS_ERROR_AGAIN )
      sprintf( eno, "EWOULDBLOCK" );
    else if( result == OS_ERROR_BADF )
      sprintf( eno, "EBADF" );
    else if( result == OS_ERROR_DESTADDRREQ )
      sprintf( eno, "EDESTADDRREQ" );
    else if( result == OS_ERROR_DQUOT )
      sprintf( eno, "EDQUOT" );
    else if( result == OS_ERROR_FAULT )
      sprintf( eno, "EFAULT" );
    else if( result == OS_ERROR_FBIG )
      sprintf( eno, "EFBIG" );
    else if( result == OS_ERROR_INTR )
      sprintf( eno, "EINTR" );
    else if( result == OS_ERROR_INVAL )
      sprintf( eno, "EINVAL" );
    else if( result == OS_ERROR_IO )
      sprintf( eno, "EIO" );
    else if( result == OS_ERROR_NOSPC )
      sprintf( eno, "ENOSPC" );
    printf( "ERROR: (%s) failed making system call write(...) errno[%s]\n", mythread->name, eno );
//*/
  }
}

//-----------------------------------------------------------------------------
void term_sighandler( int signum ) {
  quit = true;
  //quit.store( 1, std::memory_order_seq_cst );
}

//-----------------------------------------------------------------------------
void compute_command( void ) {

  printf( "prey: time[%f], dtime[%f]\n", t, dt );

  ship_c::compute_prey_command( pred_state, prey_state, prey_control, t, dt, space.get() );

}

//-----------------------------------------------------------------------------
// Request/Reply
void request_state( void ) {

  //printf( "(client-process) %s requesting state\n", mythread->name );

  // write the request information into the shared memory buffer

  // send the notification to the coordinator
  state_note.ts = generate_timestamp();
  write( state_note );

  // read( block ) the notification sent back from the coordinator
  read( server_note );

  if( server_note.source == notification_t::SERVER ) {
    if( server_note.type == notification_t::READ ) {
      // read data from the shared memory buffer
      msgbuffer.read( msg );

      //printf( "message: t[%f], dt[%f]\n", msg.header.t, msg.header.dt );

      prey->time = msg.header.t;
      prey->dtime = msg.header.dt;

      for( unsigned i = 0; i < 7; i++ )
        prey_state[i] = msg.prey_state.q[i];
      for( unsigned i = 0; i < 6; i++ )
        prey_state[i+7] = msg.prey_state.dq[i];

      for( unsigned i = 0; i < 7; i++ )
        pred_state[i] = msg.pred_state.q[i];
      for( unsigned i = 0; i < 6; i++ )
        pred_state[i+7] = msg.pred_state.dq[i];
    }
  }

  //printf( "(client-process) %s received state\n", mythread->name );

  
  //return true;
}

//-----------------------------------------------------------------------------
// Pub/Sub
bool publish_command( void ) {

  //printf( "(client-process) %s publishing command\n", mythread->name );

  // generate a timestamp
  timestamp_t ts = generate_timestamp();

  // put data in shared memory
  msg.control.ts = ts;
  for( unsigned i = 0; i < 6; i++ )
    msg.control.u[i] = prey_control[i];
  msgbuffer.write( msg );

  // send the notification to the coordinator
  command_note.ts = ts;
  write( command_note );
  
  // probably need to force a block here

  return true;
}

//-----------------------------------------------------------------------------
// May be superfluous with above
void publish_yield( void ) {

  //printf( "(client-process) %s yields\n", mythread->name );

  // send the notification to the coordinator
  yield_note.ts = generate_timestamp();
  write( yield_note );

}

//-----------------------------------------------------------------------------
int init( int argc, char* argv[] ) {
  mythread = osthread_p( new osthread_c( argv[0] ) );

  printf( "%s is initializing\n", mythread->name );

  quit = false;
  //quit.store( 0, std::memory_order_seq_cst );
  mythread->pid = getpid();

  if( argc < 2 ) {
    printf( "ERROR: client process[%s] was not given enough arguments\nExiting client process.\n", mythread->name );
    exit( 1 );
  }

  // * install SIGTERM signal handler *
  struct sigaction action;
  memset( &action, 0, sizeof(struct sigaction) );
  action.sa_handler = term_sighandler;
  sigaction( SIGTERM, &action, NULL );

  // close the coordinator side channels
  __close( FD_TIMER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_TIMER_TO_COORDINATOR_WRITE_CHANNEL );
  __close( FD_WAKEUP_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_WAKEUP_TO_COORDINATOR_WRITE_CHANNEL );
  __close( FD_PREYCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDCONTROLLER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_PREDPLANNER_TO_COORDINATOR_READ_CHANNEL );
  __close( FD_COORDINATOR_TO_PREYCONTROLLER_WRITE_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDCONTROLLER_WRITE_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDPLANNER_WRITE_CHANNEL );
  // close the other channels that should not be accessed
  __close( FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL );
  __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
  __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );

  read_fd = FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL;
  write_fd = FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL;

  //printf( "%s write_fd:%d; read_fd:%d\n", client_name.c_str(), write_fd, read_fd );

  // * initialize shared memory *
  msgbuffer = client_message_buffer_c( CLIENT_MESSAGE_BUFFER_NAME, CLIENT_MESSAGE_BUFFER_MUTEX_NAME, false );
  if( msgbuffer.open( ) != client_message_buffer_c::ERROR_NONE ) {
    sprintf( spstr, "(client-process.cpp) init() for %s failed calling client_msg_buffer_c.open(...)\n", mythread->name );
    printf( "%s", spstr );
  }

  // build the notification prototypes
  // - state notification -
  state_note.source = notification_t::CLIENT;
  state_note.type = notification_t::READ;
  state_note.pid = mythread->pid;

  // - command notification -
  command_note.source = notification_t::CLIENT;
  command_note.type = notification_t::WRITE;
  command_note.pid = mythread->pid;

  // - block notification -
  yield_note.source = notification_t::CLIENT;
  yield_note.type = notification_t::IDLE;
  yield_note.pid = mythread->pid;

/*
  if( strcmp( mythread->name, "pred-planner" ) == 0 )
    yield_note.period = 0;
  else
    yield_note.period = max_cycles;
*/

  //-------------------------
  // read the initial state from the moby configuration file
  init_moby_xml( argc, argv );

  space = moby_space_p( new moby_space_c() );
  space->read( READ_MAP );
  //space->update();
  prey = space->prey();
  predator = space->predator();

  prey_state.resize( 13 );
  pred_state.resize( 13 );
  prey_control.resize( 6 );

  for( unsigned i = 0; i < 6; i++ )
    prey_control[i] = 0;

  t = 0;
  dt = PREY_CONTROLLER_PERIOD_SEC;

  printf( "%s has initialized\n", mythread->name );

  return 0;
}

//-----------------------------------------------------------------------------
void shutdown( void ) {

  // close the shared memory
  msgbuffer.close();

  // close the pipes
  __close( read_fd );
  __close( write_fd );

  // print a parting message
  printf( "client[%s]: shutting down\n", mythread->name );
}

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] ) {

  if( init( argc, argv ) != 0 )
    //quit.store( 1, std::memory_order_seq_cst );
    quit = true;

  while( !quit ) {
    t += dt;
  //while( !quit.load( std::memory_order_seq_cst ) ) {
    request_state();
    compute_command();
    publish_command();
    publish_yield();
  }

  shutdown();

  return 0;
}
