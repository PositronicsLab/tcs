#ifndef _PURSUIT_GAZEBO_SPACE_H_
#define _PURSUIT_GAZEBO_SPACE_H_

//-----------------------------------------------------------------------------

#include "../space.h"

//-----------------------------------------------------------------------------

#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

//-----------------------------------------------------------------------------

class gazebo_space_c;
typedef boost::shared_ptr<gazebo_space_c> gazebo_space_p;

//-----------------------------------------------------------------------------

class gazebo_space_c : public space_c {
public:

  gazebo_space_c( void );
  virtual ~gazebo_space_c( void );

  virtual bool read( gazebo::physics::WorldPtr world );

};

//-----------------------------------------------------------------------------

#endif // _PURSUIT_GAZEBO_SPACE_H_
