#include "gazebo_space.h"

#include "gazebo_ship.h"

//-----------------------------------------------------------------------------
// Constructors
//-----------------------------------------------------------------------------
gazebo_space_c::gazebo_space_c( void ) { 

}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
gazebo_space_c::~gazebo_space_c( void ) {

}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
bool gazebo_space_c::read( gazebo::physics::WorldPtr world ) {

  gazebo::physics::ModelPtr model_prey;
  gazebo::physics::ModelPtr model_predator;

  gazebo_ship_p prey;
  gazebo_ship_p predator;

  space_p this_space = shared_from_this();

  // ** Find the Actors **
  model_prey = world->GetModel( "prey" );
  if( !model_prey ) {
    // Incomplete validation.  Bail out.
    gzerr << "Unable to find prey model\n";
    return false;
  }

  model_predator = world->GetModel( "predator" );
  if( !model_prey ) {
    // Incomplete validation.  Bail out.
    gzerr << "Unable to find predator model\n";
    return false;
  }

  prey = gazebo_ship_p( new gazebo_ship_c( ship_c::PREY, this_space, model_prey ) );
  if( !prey->open() ) return false;

  predator = gazebo_ship_p( new gazebo_ship_c( ship_c::PREDATOR, this_space, model_predator ) );
  if( !predator->open() ) return false;


  ship_p base_prey = boost::dynamic_pointer_cast<ship_c>( prey );
  ship_p base_pred = boost::dynamic_pointer_cast<ship_c>( predator );

  prey->adversary = base_pred;
  predator->adversary = base_prey;

  ships.push_back( base_prey );
  ships.push_back( base_pred );
 

  // ** Find the World Bounds **
  // get the reference to the spatial bound
  gazebo::physics::ModelPtr space = world->GetModel("pen");
  if( !space ) {
    gzerr << "Unable to find pen model\n";
    return false;
  }
  // compute & cache the spatial bound aabb

  // Space is empty inside the boundary, but the boundary is made up of 
  // thin walls not just a cube.
  gazebo::physics::Link_V space_links = space->GetLinks();
  if( space_links.size() != 6 ) {
    // very basic validation.  can still be malformed even if has correct 
    // number of planes
    gzerr << "Spatial boundary is malformed: model=pen\n";
    return false;
  }

  Ravelin::Vector3d c( 0.0, 0.0, 0.0 );
  Ravelin::Vector3d e( 0.0, 0.0, 0.0 );
//  gazebo::math::Vector3 c( 0.0, 0.0, 0.0 );
//  gazebo::math::Vector3 e( 0.0, 0.0, 0.0 );
  for( unsigned i = 0; i < space_links.size(); i++ ) {
    gazebo::physics::LinkPtr link = space_links[i];
    gazebo::math::Box gzbb = link->GetBoundingBox();
    //c += gzbb.GetCenter();
    c += Ravelin::Vector3d( gzbb.GetCenter().x, gzbb.GetCenter().y, gzbb.GetCenter().z );
    // Note:: following assumes centered at (0,0,0) and symmetric.  Dirty but works.
    //e = gazebo::math::Vector3( std::max(gzbb.GetCenter().x, e.x), std::max(gzbb.GetCenter().y, e.y),std::max(gzbb.GetCenter().z, e.z) );
    e = Ravelin::Vector3d( std::max(gzbb.GetCenter().x, e[0]), std::max(gzbb.GetCenter().y, e[1]),std::max(gzbb.GetCenter().z, e[2]) );
  }
  bounds = aabb_c( c, e );

  planes.resize(6);
  // east
  planes[0]=plane_c(Ravelin::Vector3d(c[0]+e[0],0,0),Ravelin::Vector3d(-1,0,0));
  // west
  planes[1]=plane_c(Ravelin::Vector3d(c[0]-e[0],0,0),Ravelin::Vector3d(1,0,0));
  // north
  planes[2]=plane_c(Ravelin::Vector3d(0,c[1]+e[1],0),Ravelin::Vector3d(0,-1,0));
  // south
  planes[3]=plane_c(Ravelin::Vector3d(0,c[1]-e[1],0),Ravelin::Vector3d(0,1,0));
  // up
  planes[4]=plane_c(Ravelin::Vector3d(0,0,c[2]+e[2]),Ravelin::Vector3d(0,0,-1));
  // down
  planes[5]=plane_c(Ravelin::Vector3d(0,0,c[2]-e[2]),Ravelin::Vector3d(0,0,1));


  update();

  return true;
}

//-----------------------------------------------------------------------------

