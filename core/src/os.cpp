#include <tcs/os.h>

#include <stdio.h>
#include <vector>
#include <assert.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>  // POSIX

#include <tcs/log.h>

//-----------------------------------------------------------------------------
extern log_c info;
//-----------------------------------------------------------------------------
//#include <stdio.h>
void __select( const std::vector<int>& fds, fd_set& channels ) {
  int max_fd = 0;

  assert( fds.size() > 0 );

  FD_ZERO( &channels );
  for( int i = 0; i < fds.size(); i++ ) {
    max_fd = std::max( max_fd, fds[i] );
    FD_SET( fds[i], &channels );
    //printf( "fd:%d, ", fds[i] );
  }
  //printf( "max_fd:%d\n", max_fd );
  if( select( max_fd + 1, &channels, NULL, NULL, NULL ) == -1 ) {
/*
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

    printf( "ERROR in select:%s\n", buf );
*/
  }
}

//-----------------------------------------------------------------------------
bool __isset( const int& fd, fd_set& channels ) {
  if( FD_ISSET( fd, &channels ) == 0 ) return false;
  return true;
}

//-----------------------------------------------------------------------------
int __open( const char* path, int oflags ) {
  return open( path, oflags );

/*if( open( path, oflags, mode ) == -1 ) {
    errno{EACCES,EDQUOT,EEXIST,EFAULT,EFBIG,EINTR,EISDIR,ELOOP,EMFILE,ENAMETOOLONG,ENFILE,ENODEV,ENOENT,ENOMEM,ENOSPC,ENOTDIR,ENXIO,EOVERFLOW,EPERM,EROFS,ETXTBSY,EWOULDBLOCK}
  }*/
}

//-----------------------------------------------------------------------------
int __open( const char* path, int oflags, const mode_t& mode ) {
  return open( path, oflags, mode );

/*if( open( path, oflags, mode ) == -1 ) {
    errno{EACCES,EDQUOT,EEXIST,EFAULT,EFBIG,EINTR,EISDIR,ELOOP,EMFILE,ENAMETOOLONG,ENFILE,ENODEV,ENOENT,ENOMEM,ENOSPC,ENOTDIR,ENXIO,EOVERFLOW,EPERM,EROFS,ETXTBSY,EWOULDBLOCK}
  }*/
}

//-----------------------------------------------------------------------------
void __close( const int& fd ) {
  close( fd );
}

//-----------------------------------------------------------------------------
ssize_t __read( int fd, void* buffer, size_t bytes ) {
  return read( fd, buffer, bytes );

/*if( read( fd, buffer, bytes ) == -1 ) {
    errno{EAGAIN, EBADF, EFAULT, EINVAL, EINTR, EIO, EISDIR}
  }*/
}

//-----------------------------------------------------------------------------
os_error_e __write( int fd, void* buffer, size_t bytes_to_write, ssize_t& bytes_written ) {
  char buf[16];
  char spstr[256];
  bytes_written = write( fd, buffer, bytes_to_write );
  if( bytes_written == -1 ) {
    //errno{EAGAIN, EBADF, EDESTADDRREQ, EDQUOT, EFAULT, EFBIG, EINVAL, EINTR, EIO, ENOSPC, EPIPE
    bytes_written = 0;
    if( errno == EAGAIN ) {
      return OS_ERROR_AGAIN;
    } else if( errno == EWOULDBLOCK ) {
      return OS_ERROR_AGAIN;
    } else if( errno == EBADF ) {
      return OS_ERROR_BADF;
    } else if( errno == EDESTADDRREQ ) {
      return OS_ERROR_DESTADDRREQ;
    } else if( errno == EDQUOT ) {
      return OS_ERROR_DQUOT;
    } else if( errno == EFAULT ) {
      return OS_ERROR_FAULT;
    } else if( errno == EFBIG ) {
      return OS_ERROR_FBIG;
    } else if( errno == EINVAL ) {
      return OS_ERROR_INVAL;
    } else if( errno == EINTR ) {
      return OS_ERROR_INTR;
    } else if( errno == EIO ) {
      return OS_ERROR_IO;
    } else if( errno == ENOSPC ) {
      return OS_ERROR_NOSPC;
    } else if( errno == EPIPE ) {
      return OS_ERROR_PIPE;
    }
  }
  return OS_ERROR_NONE;
}

//-----------------------------------------------------------------------------
// Mutex
//-----------------------------------------------------------------------------
mutex_c::mutex_c( void ) {
  _create = false;
  _open = false;
  _defined = false;
  _mutex = NULL;
}

//-----------------------------------------------------------------------------
mutex_c::mutex_c( const char* name, const bool& create = false ) {
  assert( name != NULL );

  _name = name;
  _create = create;
  _open = false;
  _defined = true;
  _mutex = NULL;
}

//-----------------------------------------------------------------------------
mutex_c::~mutex_c( void ) {

}

//-----------------------------------------------------------------------------
mutex_c::error_e mutex_c::init( void ) {
  void*                   addr;
  pthread_mutexattr_t     attr;

  assert( _defined && !_open );

  if( _create )
    _fd = shm_open( _name.c_str( ), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR );
  else
    _fd = shm_open( _name.c_str( ), O_RDWR, S_IRUSR | S_IWUSR );

  if( _fd == -1 ) {
    perror( "shm_open()" );
    return ERROR_OPEN;
  }

  if( ftruncate( _fd, sizeof(pthread_mutex_t) ) == -1 ) {
    return ERROR_TRUNCATE;
  }

  addr = mmap( NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0 );

  if( addr == MAP_FAILED ) {
    perror( "mmap()" );
    __close( _fd );
    return ERROR_MAP;
  }

  _mutex = static_cast<pthread_mutex_t*> ( addr );
  if( _create ) {
    pthread_mutexattr_init( &attr );
    pthread_mutexattr_setpshared( &attr, PTHREAD_PROCESS_SHARED );

    if( pthread_mutex_init( _mutex, &attr ) != 0 ) {
      // what other handling should be done in response.  Close the fd?
      return ERROR_INIT;
    }
    pthread_mutexattr_destroy( &attr );

    if( msync( addr, sizeof(pthread_mutex_t), MS_SYNC | MS_INVALIDATE ) != 0 ) {
      // what other handling should be done in response.  Close the fd, release the mutex?
      perror( "msync()" );
      return ERROR_SYNC;
    }
  }
 
  _open = true;

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
mutex_c::error_e mutex_c::destroy( void ) {

  assert( _defined && _open );

  __close( _fd );

  if( _create ) {
    if( munmap( (void*)_mutex, sizeof(pthread_mutex_t) ) ) {
      perror( "munmap()" );
      return ERROR_UNMAP;
    }
    if( shm_unlink( _name.c_str( ) ) != 0 ) {
      return ERROR_UNLINK;
    }
  }

  _open = false;

  return ERROR_NONE;
}

//-----------------------------------------------------------------------------
void mutex_c::lock( void ) {

  assert( _open );

  pthread_mutex_lock( _mutex );
}

//-----------------------------------------------------------------------------
void mutex_c::unlock( void ) {

  assert( _open );

  pthread_mutex_unlock( _mutex );
}

//-----------------------------------------------------------------------------
bool mutex_c::open( void ) const {
  return _open;
}

//-----------------------------------------------------------------------------
bool mutex_c::defined( void ) const {
  return _defined;
}

//-----------------------------------------------------------------------------
bool mutex_c::creator( void ) const {
  return _create;
}

//-----------------------------------------------------------------------------
std::string mutex_c::name( void ) const {
  return _name;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
