#ifndef _DYNAMICS_PLUGIN_H_
#define _DYNAMICS_PLUGIN_H_

#include <dlfcn.h>              //POSIX

//-----------------------------------------------------------------------------

typedef double Real;

//-----------------------------------------------------------------------------

// dynamics initialization function signature
typedef bool ( *init_f )( int argv, char** argc );

// dynamics shutdown function signature
typedef void ( *shutdown_f )( void );

// dynamics step function signature
typedef void ( *step_f )( dynamics_data_t* data );

// dynamics step size query function signature
typedef Real ( *get_step_size_f )( void );

// dynamics state query function signature
typedef void ( *get_state_f )( dynamics_data_t* data );

//-----------------------------------------------------------------------------

class dynamics_plugin_c {
private:

  void* HANDLE; 	// plugin handle

  //---------------------------------------------------------------------------

public:
  enum error_e {
    ERROR_NONE = 0,
    ERROR_OPEN,
    ERROR_LINK,
    ERROR_READ
  };

  dynamics_plugin_c( void ) { HANDLE = NULL; }
  virtual ~dynamics_plugin_c( void ) { }

  init_f               init;
  shutdown_f           shutdown;
  step_f               step;
  get_step_size_f      get_step_size;
  get_state_f          get_state;

  //---------------------------------------------------------------------------
  error_e read( const char* filename ) {
    if( open( filename )  )
      // attempt to read the plugin file
      HANDLE = dlopen( filename, RTLD_LAZY );
    printf( "plugin: %s\n", filename );

    if( !HANDLE ) {
      std::cerr << " failed to read plugin from " << filename << std::endl;
      std::cerr << "  " << dlerror( ) << std::endl;
      return ERROR_OPEN;
    }

    // locate the init function
    init = (init_f) dlsym(HANDLE, "init");
    const char* dlsym_error1 = dlerror( );
    if( dlsym_error1 ) {
      std::cerr << " warning: cannot load symbol 'init' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error1 << std::endl;
      return ERROR_LINK;
    }

    shutdown = (shutdown_f) dlsym(HANDLE, "shutdown");
    const char* dlsym_error2 = dlerror( );
    if( dlsym_error2 ) {
      std::cerr << " warning: cannot load symbol 'init' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error2 << std::endl;
      return ERROR_LINK;
    }

    // locate the run function
    step = (step_f) dlsym(HANDLE, "step");
    const char* dlsym_error3 = dlerror( );
    if( dlsym_error3 ) {
      std::cerr << " warning: cannot load symbol 'step' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error3 << std::endl;
      return ERROR_LINK;
    }

    // locate the get step size function
    get_step_size = (get_step_size_f) dlsym(HANDLE, "get_step_size");
    const char* dlsym_error4 = dlerror( );
    if( dlsym_error4 ) {
      std::cerr << " warning: cannot load symbol 'get_step_size' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error4 << std::endl;
      return ERROR_LINK;
    }

    // locate the get state function
    get_state = (get_state_f) dlsym(HANDLE, "get_state");
    const char* dlsym_error5 = dlerror( );
    if( dlsym_error5 ) {
      std::cerr << " warning: cannot load symbol 'get_state' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error5 << std::endl;
      return ERROR_LINK;
    }

    return ERROR_NONE;
  }
  //---------------------------------------------------------------------------
  void close( void ) {
    if( HANDLE != NULL )
      dlclose( HANDLE );
  }
  //---------------------------------------------------------------------------

private:
  //---------------------------------------------------------------------------
  error_e open( const char* filename ) {

    HANDLE = dlopen( filename, RTLD_LAZY );
    if( !HANDLE ) {
      //if( VERBOSE ) std::cerr << " failed to open plugin: " << filename << std::endl;
      //if( VERBOSE ) std::cerr << "  " << dlerror( ) << std::endl;
      return ERROR_OPEN;
    }
    return ERROR_NONE;
  }

  //---------------------------------------------------------------------------

};

//-----------------------------------------------------------------------------

#endif // _DYNAMICS_PLUGIN_H_
