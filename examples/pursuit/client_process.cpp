#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <atomic>

#include <unistd.h>
#include <signal.h>
#include <errno.h>

#include <tcs/log.h>
#include <tcs/time.h>
#include <tcs/os.h>
#include <tcs/notification.h>
#include <tcs/message.h>
#include <tcs/osthread.h>

#include "channels.h"

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
// Work simulation
volatile unsigned long long  counter;

static unsigned              rand_seed;

static unsigned              rand_delta;
static unsigned              min_cycles;
static unsigned              max_cycles;

static unsigned long long    delta_workunits;
static unsigned long long    min_workunits;
static unsigned long long    max_workunits;

static unsigned long long    cycles_per_workunit;

client_message_t msg;
client_message_buffer_c msgbuffer;

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

  //printf( "(client-process) %s computing command\n", mythread->name );

  // Busy work
  counter = 0;
  //unsigned cycles_to_run = (rand() % rand_delta) + min_cycles;
  unsigned long long workunits_to_run = ((unsigned long long)rand()) % delta_workunits + min_workunits;
  //printf( "workunits_to_run: %llu\n", workunits_to_run );


  for( unsigned long long i = 0; i < workunits_to_run; i++ ) {
    counter++;
  }

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

  // read data from the shared memory buffer
  msgbuffer.read( msg );

  //printf( "(client-process) %s received state\n", mythread->name );
  
  //return true;
}

//-----------------------------------------------------------------------------
// Pub/Sub
bool publish_command( void ) {

  //printf( "(client-process) %s publishing command\n", mythread->name );

  // put data in the shared memory buffer
  msgbuffer.write( msg );

  // send the notification to the coordinator
  command_note.ts = generate_timestamp();
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

  if( argc < 4 ) {
    printf( "ERROR: client process[%s] was not given enough arguments\nExiting client process.\n", mythread->name );
    exit( 1 );
  }
  // calibrate the time it takes to do incrementation (simulated work)  
  counter = 0;
  unsigned long long increments = 100;

  timestamp_t ts_before, ts_after, ts_delta;

  ts_before = generate_timestamp();
  for( unsigned long long i = 0; i < increments; i++ ) {
    counter++;
  }
  ts_after = generate_timestamp();
  ts_delta = ts_after - ts_before;
  cycles_per_workunit = (ts_delta / increments) + 1; // since integer division, add 1 to handle any truncation

  printf( "cycles_per_workunit: %llu\n", cycles_per_workunit ); 

  // get and convert (if necessary) the parameters
  rand_seed = (unsigned) atoi( argv[1] );
  //min_cycles = (unsigned long long) atoi( argv[2] );
  //max_cycles = (unsigned long long) atoi( argv[3] );
  min_workunits = strtoull( argv[2], NULL, 10 );
  min_workunits /= cycles_per_workunit;
  max_workunits = strtoull( argv[3], NULL, 10 );
  max_cycles = max_workunits;
  max_workunits /= cycles_per_workunit;
  delta_workunits = max_workunits - min_workunits;
  // Add in little extra buffer to the delta
  delta_workunits -= min_workunits;

  //printf( "%s has started with rand_seed[%d], min_cycles[%d], max_cycles[%d].\n", client_name.c_str(), rand_seed, min_cycles, max_cycles ); 
    
  srand( rand_seed );

  rand_delta = max_cycles - min_cycles;

  // open log * only for debugging client process ipc using test_client.cpp*
  char logname[64];
  sprintf( logname, "%s.log", mythread->name );
  info = log_p( new log_c( logname, true ) );
  log_c::error_e log_err = info->allocate( LOG_CAPACITY );
  if( log_err != log_c::ERROR_NONE ) {
    sprintf( spstr, "(client-process.cpp) init() failed calling log_c::allocate(...).\nExiting\n" ); 
    printf( spstr );
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

  if( strcmp( mythread->name, "prey-controller" ) == 0 ) {
    read_fd = FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL;
    write_fd = FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL;

    // close the other channels that should not be accessed
    __close( FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL );
    __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );
  } else if( strcmp( mythread->name, "pred-controller" ) == 0 ) {
    read_fd = FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL;
    write_fd = FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL;

    // close the other channels that should not be accessed
    __close( FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL );
    __close( FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL );
  } else if( strcmp( mythread->name, "pred-planner" ) == 0 ) {
    read_fd = FD_COORDINATOR_TO_PREDPLANNER_READ_CHANNEL;
    write_fd = FD_PREDPLANNER_TO_COORDINATOR_WRITE_CHANNEL;

    // close the other channels that should not be accessed
    __close( FD_PREYCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREYCONTROLLER_READ_CHANNEL );
    __close( FD_PREDCONTROLLER_TO_COORDINATOR_WRITE_CHANNEL );
    __close( FD_COORDINATOR_TO_PREDCONTROLLER_READ_CHANNEL );
  } else {
    //printf( "ERROR: unable to reference correct program\nExiting\n" );
    return 1;
  }
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

  if( strcmp( mythread->name, "pred-planner" ) == 0 )
    yield_note.period = 0;
  else
    yield_note.period = max_cycles;

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
  //while( !quit.load( std::memory_order_seq_cst ) ) {
    request_state();
    compute_command();
    publish_command();
    publish_yield();
  }

  shutdown();

  return 0;
}
