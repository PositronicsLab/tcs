#include "gazebo_ship.h"

//-----------------------------------------------------------------------------
// Constructors
//-----------------------------------------------------------------------------
gazebo_ship_c::gazebo_ship_c( ship_c::role_e role, space_p space, gazebo::physics::ModelPtr model ) : ship_c( role )
{
  _model = model;
  _space = space;

  if( role == ship_c::PREY ) {
    controller_step = PREY_CONTROLLER_PERIOD_SEC;
  } else if(role == ship_c::PREDATOR ) {
    controller_step = PREDATOR_CONTROLLER_PERIOD_SEC;
  }
}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
gazebo_ship_c::~gazebo_ship_c( void ) {

}

//-----------------------------------------------------------------------------
// Base Class Implementations
//-----------------------------------------------------------------------------
bool gazebo_ship_c::open( void ) {
  const unsigned X = 0, Y = 1, Z = 2;

  _body = _model->GetLink("body");
  if( !_body ) {
    gzerr << "Unable to find link: body\n";
    return false;
  }

  // get inertial properties from Gazebo
  gazebo::physics::InertialPtr gz_inertial = _body->GetInertial();

  // setup mass
  _inertial.m = gz_inertial->GetMass();

  // setup moment of inertia
  gazebo::math::Vector3 pm = gz_inertial->GetPrincipalMoments();
  _inertial.J(X,X) = pm.x;  _inertial.J(X,Y) = 0.0;  _inertial.J(X,Z) = 0.0;
  _inertial.J(Y,X) = 0.0;   _inertial.J(Y,Y) = pm.y; _inertial.J(Y,Z) = 0.0;
  _inertial.J(Z,X) = 0.0;   _inertial.J(Z,Y) = 0.0;  _inertial.J(Z,Z) = pm.z;

  // Note: center-of-mass is assumed to be (0,0,0) relative to the body frame

  // Note: aabb may not be tight fit if the ship begins unaligned to world
  // and error may propagate through future collision queries.
  ship_frame_bb = aabb();    

  return true;
}

//-----------------------------------------------------------------------------
bool gazebo_ship_c::read( std::vector<double>& q ) {
  // sanity check
  assert( q.size() == 13 );

  // reset accumulators??  <<see below>>

  // get the current pose and velocity
  _pos = _model->GetWorldPose().pos;
  _rot = _model->GetWorldPose().rot;
  _dpos = _model->GetWorldLinearVel();
  _drot = _model->GetWorldAngularVel();

  // update the state vector
  for( unsigned i = 0; i < 3; i++ )
    q[i] = _pos[i];
  q[3] = _rot.x;
  q[4] = _rot.y;
  q[5] = _rot.z;
  q[6] = _rot.w;
  for( unsigned i = 0; i < 3; i++ )
    q[i+7] = _dpos[i];
  for( unsigned i = 0; i < 3; i++ )
    q[i+10] = _drot[i];

  // do updates
  // TODO:update aabb

  return true;
}

//-----------------------------------------------------------------------------
bool gazebo_ship_c::write( const std::vector<double>& u ) {
  // sanity check
  assert( u.size() == 6 );

  // NOTE: we do this in a different place for gazebo than moby.  Rectify if 
  // necessary
/*
  // reset the force accumulators
  _accum_force = _body->GetRelativeForce();
  _body->AddRelativeForce( -_accum_force );
  // reset the torque accumulators
  _accum_torque = _body->GetRelativeTorque();
  _body->AddRelativeTorque( -_accum_torque );
*/
  // update the control force
  _force.x = u[0];
  _force.y = u[1];
  _force.z = u[2];
  _body->AddForce( _force );

  // update the control torque
  _torque.x = u[3];
  _torque.y = u[4];
  _torque.z = u[5];
  _body->AddTorque( _torque );

  return true;
}

//-----------------------------------------------------------------------------
void gazebo_ship_c::reset( void ) {
  // reset the force accumulators
  _accum_force = _body->GetRelativeForce();
  _body->AddRelativeForce( -_accum_force );
  // reset the torque accumulators
  _accum_torque = _body->GetRelativeTorque();
  _body->AddRelativeTorque( -_accum_torque );
}

//-----------------------------------------------------------------------------
/// Query the current axis aligned bounding box of the ship
aabb_c gazebo_ship_c::aabb( void ) {
  // Note: following may need to query GetCollisionBox instead
  gazebo::math::Box gzbb = _model->GetBoundingBox();
  gazebo::math::Vector3 gzc = gzbb.GetCenter();
  gazebo::math::Vector3 gze = gzbb.GetSize() / 2;

  Ravelin::Vector3d c( gzc.x, gzc.y, gzc.z );
  Ravelin::Vector3d e( gze.x, gze.y, gze.z );

  return aabb_c ( c, e );
}

//------------------------------------------------------------------------------
