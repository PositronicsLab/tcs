#ifndef _PURSUIT_GAZEBO_SHIP_H_
#define _PURSUIT_GAZEBO_SHIP_H_

//-----------------------------------------------------------------------------

#include "../ship.h"

#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

//-----------------------------------------------------------------------------

class gazebo_ship_c;
typedef boost::shared_ptr<gazebo_ship_c> gazebo_ship_p;

//-----------------------------------------------------------------------------
class gazebo_ship_c : public ship_c {
public:
  //---------------------------------------------------------------------------
  // Constructors
  //---------------------------------------------------------------------------
  gazebo_ship_c( role_e role, space_p space, gazebo::physics::ModelPtr model );
  
  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~gazebo_ship_c( void );

  //---------------------------------------------------------------------------
  // Inherited Definitions
  //---------------------------------------------------------------------------
  virtual bool open( void );
  virtual bool read( std::vector<double>& q );
  virtual bool write( const std::vector<double>& u );
  virtual void reset( void );

protected:
  // the gazebo reference to the ship's model 
  gazebo::physics::ModelPtr _model;
  // the gazebo reference to the ship's link
  gazebo::physics::LinkPtr _body;

  aabb_c aabb( void );

private:
  gazebo::math::Vector3     _pos;         // position
  gazebo::math::Quaternion  _rot;         // rotation
  gazebo::math::Vector3     _dpos;        // change in position
  gazebo::math::Vector3     _drot;        // change in rotation

  gazebo::math::Vector3     _accum_force;
  gazebo::math::Vector3     _accum_torque;

  gazebo::math::Vector3     _force;
  gazebo::math::Vector3     _torque;
};

#endif // _PURSUIT_GAZEBO_SHIP_H_
