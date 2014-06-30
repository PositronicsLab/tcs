#ifndef _PURSUIT_MOBY_SPACE_H_
#define _PURSUIT_MOBY_SPACE_H_

//-----------------------------------------------------------------------------

#include "space.h"

//-----------------------------------------------------------------------------

#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <Moby/Types.h>
#include <Moby/XMLReader.h>
#include <Moby/RigidBody.h>

//-----------------------------------------------------------------------------

using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

class moby_space_c;
typedef boost::shared_ptr<moby_space_c> moby_space_p;

//-----------------------------------------------------------------------------

class moby_space_c : public space_c {
public:

  moby_space_c( void );
  virtual ~moby_space_c( void );

  virtual bool read( const std::map<std::string, Moby::BasePtr>& READ_MAP );

};

//-----------------------------------------------------------------------------

#endif // _PURSUIT_MOBY_SPACE_H_
