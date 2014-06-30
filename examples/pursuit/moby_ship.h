#ifndef _PURSUIT_MOBY_SHIP_H_
#define _PURSUIT_MOBY_SHIP_H_

//-----------------------------------------------------------------------------

#include "ship.h"

#include <boost/shared_ptr.hpp>

#include <Moby/Types.h>
#include <Moby/XMLReader.h>
#include <Moby/RigidBody.h>
#include <Moby/BoxPrimitive.h>

//-----------------------------------------------------------------------------

using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

class moby_ship_c;
typedef boost::shared_ptr<moby_ship_c> moby_ship_p;

//-----------------------------------------------------------------------------
class moby_ship_c : public ship_c {
public:
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  moby_ship_c( ship_c::role_e role, space_p space, RigidBodyPtr body, boost::shared_ptr<Moby::BoxPrimitive> box ); 
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~moby_ship_c( void );

  //---------------------------------------------------------------------------
  // Inherited Definitions
  //---------------------------------------------------------------------------
  virtual bool open( void );
  virtual bool read( std::vector<double>& q );
  virtual bool write( const std::vector<double>& u );
  virtual void reset( void );

  virtual bool read_ke( double& ke );
  //---------------------------------------------------------------------------
protected:
  RigidBodyPtr _body;
  boost::shared_ptr<Moby::BoxPrimitive> _box;

  aabb_c aabb( void );

private:
  Ravelin::Pose3d           _pose;   // pose
  Ravelin::SVelocityd       _dx;     // change in position and rotation
  Ravelin::VectorNd         _u;      // current control

  Ravelin::Vector3d         _linear_vel;
  Ravelin::Vector3d         _angular_vel;
};

#endif // _PURSUIT_MOBY_SHIP_H_
