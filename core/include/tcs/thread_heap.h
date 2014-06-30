#ifndef _THREAD_HEAP_H_
#define _THREAD_HEAP_H_

#include <tcs/types.h>
#include <tcs/thread.h>
#include <vector>
#include <algorithm>
#include <exception>

//-----------------------------------------------------------------------------

typedef bool (*comparator_f)( thread_p t1, thread_p t2 ); 

//-----------------------------------------------------------------------------
// Comparison function declarations
//-----------------------------------------------------------------------------
bool max_compare_threads_by_priority_f( thread_p t1, thread_p t2 );
bool min_compare_threads_by_priority_f( thread_p t1, thread_p t2 );
bool max_compare_threads_by_progress_f( thread_p t1, thread_p t2 );
bool min_compare_threads_by_progress_f( thread_p t1, thread_p t2 );
 
//-----------------------------------------------------------------------------
// Thread Head declaration
//-----------------------------------------------------------------------------

class thread_heap_c {
public:
  enum type_e {
    MINHEAP,
    MAXHEAP
  };
 
  enum criteria_e {
    PRIORITY,
    PROGRESS
  };
 
  enum error_e {
    ERROR_NONE,
    ERROR_MAKEHEAP,
    ERROR_PUSHHEAP,
    ERROR_POPHEAP,
    ERROR_EMPTY
  };

private:
  std::vector<thread_p>  _heap;
  bool                   _empty;
  comparator_f           _comparator;
  type_e                 _type;
  criteria_e             _criteria;

public:
  thread_heap_c( void );
  thread_heap_c( type_e type, criteria_e criteria );
  virtual ~thread_heap_c( void );

  error_e push( thread_p thread );
  error_e pop( thread_p& thread );
  error_e top( thread_p& thread );

  unsigned size( void );
  thread_p element( unsigned i );
  error_e remove( unsigned i, thread_p& thread );
  bool empty( void );

private:
  void set_comparator( void );
};

/*
//-----------------------------------------------------------------------------
// Old method was not adaptable enough.  Left for reference as an option,
// but the function pointer method should be used in lieu rather than the
// functor
struct compare_thread_p_t {
  bool operator()( const thread_p& t1, const thread_p& t2 ) const {
    if( t1->type() == thread_c::PROCESSOR && t2->type() == thread_c::PROCESSOR ) {
      // Min heap.  Want lowest progress thread at top of heap
      return t1->temporal_progress > t2->temporal_progress;
    } else if( t1->type()==thread_c::OSTHREAD && t2->type()==thread_c::OSTHREAD ) {
      // Max heap.  Want highest priority thread at top of heap
      return t1->priority < t2->priority;
    } else  {
      // Generic catch all for all other cases
      // May need additional specific cases
      return t1->temporal_progress > t2->temporal_progress; 
    }
  }
};
*/
//-----------------------------------------------------------------------------

#endif // _THREAD_HEAP_H_
