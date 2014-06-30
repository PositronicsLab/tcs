#include "moby_ship.h"

//-----------------------------------------------------------------------------
// Constructors
//-----------------------------------------------------------------------------
moby_ship_c::moby_ship_c( ship_c::role_e role, space_p space, RigidBodyPtr body, boost::shared_ptr<Moby::BoxPrimitive> box ) : ship_c( role )
{ 
  _body = body;
  _box = box;
  _space = space;

  //_dx.set_zero( 6 );
  _u.set_zero( 6 );

  if( role == ship_c::PREY ) {
    controller_step = PREY_CONTROLLER_PERIOD_SEC;
  } else if(role == ship_c::PREDATOR ) {
    controller_step = PREDATOR_CONTROLLER_PERIOD_SEC;
  }
}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
moby_ship_c::~moby_ship_c( void ) {

}

//-----------------------------------------------------------------------------
// Base Class Implementations
//-----------------------------------------------------------------------------
bool moby_ship_c::open( void ) {
  if( !_body ) return false;
  if( !_box ) return false;
  
  _pose = Ravelin::Pose3d( *_body->get_pose().get() );
  _velocity = _body->get_velocity();
  
  _inertial = _body->get_inertia();

  ship_frame_bb = aabb_c();

  return true;
}

//-----------------------------------------------------------------------------
bool moby_ship_c::read( std::vector<double>& q ) {
  // sanity check
  assert( q.size() == 13 );
/*
  // reset accumulators
  _body->reset_accumulators();
*/
  // get the current pose and velocity
  _pose = *_body->get_pose( );
  _pose.update_relative_pose( GLOBAL );
  _dx = Ravelin::Pose3d::transform( GLOBAL, _body->get_velocity() );

  // update the state vector
  for( unsigned i = 0; i < 3; i++ )
    q[i] = _pose.x[i];
  for( unsigned i = 0; i < 4; i++ )
    q[i+3] = _pose.q[i];
  _linear_vel = _dx.get_linear();
  _angular_vel = _dx.get_angular();
  for( unsigned i = 0; i < 3; i++ ) {
    q[i+7] = _linear_vel[i];
    q[i+10] = _angular_vel[i];
  }

  return true;
}

//-----------------------------------------------------------------------------
bool moby_ship_c::read_ke( double& ke ) {
  ke = _body->calc_kinetic_energy();
  return true;
}
//-----------------------------------------------------------------------------
bool moby_ship_c::write( const std::vector<double>& u ) {
  // sanity check
  assert( u.size() ==  6 );

  // update the control data buffer
  for( unsigned i = 0; i < 6; i++ )
    _u[i] = u[i];

  // update the body reference
  _body->add_generalized_force( _u );

  return true;
}

//-----------------------------------------------------------------------------
void moby_ship_c::reset( void ) {
  // reset accumulators
  _body->reset_accumulators();
}

//------------------------------------------------------------------------------
aabb_c moby_ship_c::aabb( void ) {
  Ravelin::Vector3d bb_center( 0, 0, 0 );
  Ravelin::Vector3d bb_extens( _box->get_x_len() / 2, _box->get_y_len() / 2, _box->get_z_len() / 2 ); 

  return aabb_c( bb_center, bb_extens );
}

//------------------------------------------------------------------------------
