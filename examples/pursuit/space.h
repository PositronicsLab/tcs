#ifndef _PURSUIT_SPACE_H_
#define _PURSUIT_SPACE_H_

//-----------------------------------------------------------------------------

#include "plane.h"
#include "aabb.h"
#include "ship.h"

//-----------------------------------------------------------------------------

#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
/*
#include <Moby/Types.h>
#include <Moby/XMLReader.h>
#include <Moby/RigidBody.h>
*/
//-----------------------------------------------------------------------------
/*
using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;
*/
//-----------------------------------------------------------------------------

class space_c;
typedef boost::shared_ptr<space_c> space_p;
//class ship_c;
//typedef boost::shared_ptr<ship_c> ship_p;

//-----------------------------------------------------------------------------

class space_c : public boost::enable_shared_from_this<space_c> {
private:
  int _pred_idx;
  int _prey_idx;

public:
  plane_list_t planes;
  aabb_c bounds;
  std::vector<ship_p> ships;

  ship_p predator( void );
  ship_p prey( void );

  void update( void );

  space_c( void );
  virtual ~space_c( void );

};

//-----------------------------------------------------------------------------

#endif // _PURSUIT_SPACE_H_
