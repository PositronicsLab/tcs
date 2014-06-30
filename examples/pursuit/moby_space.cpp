#include "moby_space.h"

#include "moby_ship.h"

//-----------------------------------------------------------------------------
// Constructors
//-----------------------------------------------------------------------------
moby_space_c::moby_space_c( void ) { 

}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
moby_space_c::~moby_space_c( void ) {

}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
bool moby_space_c::read( const std::map<std::string, Moby::BasePtr>& READ_MAP ) {

  Ravelin::Pose3d pose;
  Moby::RigidBodyPtr wall_north;
  Moby::RigidBodyPtr wall_south;
  Moby::RigidBodyPtr wall_east;
  Moby::RigidBodyPtr wall_west;
  Moby::RigidBodyPtr wall_up;
  Moby::RigidBodyPtr wall_down;

  Moby::RigidBodyPtr rbpredator;
  Moby::RigidBodyPtr rbprey;
  boost::shared_ptr<Moby::BoxPrimitive> box;

  moby_ship_p prey;
  moby_ship_p predator;

  space_p this_space = shared_from_this();

  // ** Find the Actors **
  // find the prey
  if( READ_MAP.find("prey") == READ_MAP.end() ) return false;
    rbprey = dynamic_pointer_cast<Moby::RigidBody>( READ_MAP.find("prey")->second );
  if( !rbprey ) return false;

  // find the predator
  if( READ_MAP.find("predator") == READ_MAP.end() ) return false;
  rbpredator = dynamic_pointer_cast<Moby::RigidBody>( READ_MAP.find("predator")->second );
  if( !rbpredator ) return false;
  
  // find the ship (box primitive)
  if( READ_MAP.find("ship") == READ_MAP.end() ) return false;
  box = dynamic_pointer_cast<Moby::BoxPrimitive>( READ_MAP.find("ship")->second );
  if( !box ) return false;

  prey = moby_ship_p( new moby_ship_c( ship_c::PREY, this_space, rbprey, box ) );
  //if( !prey->read( READ_MAP, this_space ) )
  if( !prey->open() ) return false;

  predator = moby_ship_p( new moby_ship_c( ship_c::PREDATOR, this_space, rbpredator, box ) );
  //if( !predator->read( READ_MAP, this_space ) )
  if( !predator->open() ) return false;

  ship_p base_prey = dynamic_pointer_cast<ship_c>( prey );
  ship_p base_pred = dynamic_pointer_cast<ship_c>( predator );

  prey->adversary = base_pred;
  predator->adversary = base_prey;

  ships.push_back( base_prey );
  ships.push_back( base_pred );
 

  // ** Find the World Bounds **
  // compute aabb bounds
  Ravelin::Vector3d c( 0.0, 0.0, 0.0 );
  Ravelin::Vector3d e( 0.0, 0.0, 0.0 );

  // * Wall North *
  if( READ_MAP.find("wall_north") == READ_MAP.end() ) 
    return false;
  wall_north = dynamic_pointer_cast<Moby::RigidBody>( READ_MAP.find("wall_north")->second );
  if( !wall_north )
    return false;
  pose = *wall_north->get_pose();
  pose.update_relative_pose( GLOBAL );
  c += pose.x;
  // Note:: following assumes centered at (0,0,0) and symmetric.  Dirty but works.
  e = Ravelin::Vector3d( std::max(pose.x[0], e[0]), std::max(pose.x[1], e[1]), std::max(pose.x[2], e[2]) );
  
  // * Wall South *
  if( READ_MAP.find("wall_south") == READ_MAP.end() )
    return false;
  wall_south = dynamic_pointer_cast<Moby::RigidBody>( READ_MAP.find("wall_south")->second );
  if( !wall_south )
    return false;
  pose = *wall_south->get_pose();
  pose.update_relative_pose( GLOBAL );
  c += pose.x;
  // Note:: following assumes centered at (0,0,0) and symmetric.  Dirty but works.
  e = Ravelin::Vector3d( std::max(pose.x[0], e[0]), std::max(pose.x[1], e[1]), std::max(pose.x[2], e[2]) );

  // * Wall East *
  if( READ_MAP.find("wall_east") == READ_MAP.end() ) 
    return false;
  wall_east = dynamic_pointer_cast<Moby::RigidBody>( READ_MAP.find("wall_east")->second );
  if( !wall_east ) 
    return false; 
  pose = *wall_east->get_pose();
  pose.update_relative_pose( GLOBAL );
  c += pose.x;
  // Note:: following assumes centered at (0,0,0) and symmetric.  Dirty but works.
  e = Ravelin::Vector3d( std::max(pose.x[0], e[0]), std::max(pose.x[1], e[1]), std::max(pose.x[2], e[2]) );

  // * Wall West *
  if( READ_MAP.find("wall_west") == READ_MAP.end() ) 
    return false;
  wall_west = dynamic_pointer_cast<Moby::RigidBody>( READ_MAP.find("wall_west")->second );
  if( !wall_west )
    return false;
  pose = *wall_west->get_pose();
  pose.update_relative_pose( GLOBAL );
  c += pose.x;
  // Note:: following assumes centered at (0,0,0) and symmetric.  Dirty but works.
  e = Ravelin::Vector3d( std::max(pose.x[0], e[0]), std::max(pose.x[1], e[1]), std::max(pose.x[2], e[2]) );

  // * Wall Up *
  if( READ_MAP.find("wall_up") == READ_MAP.end() )
    return false;
  wall_up = dynamic_pointer_cast<Moby::RigidBody>( READ_MAP.find("wall_up")->second );
  if( !wall_up ) 
    return false;
  pose = *wall_up->get_pose();
  pose.update_relative_pose( GLOBAL );
  c += pose.x;
  // Note:: following assumes centered at (0,0,0) and symmetric.  Dirty but works.
  e = Ravelin::Vector3d( std::max(pose.x[0], e[0]), std::max(pose.x[1], e[1]), std::max(pose.x[2], e[2]) );

  // * Wall Down *
  if( READ_MAP.find("wall_down") == READ_MAP.end() ) 
    return false;
  wall_down = dynamic_pointer_cast<Moby::RigidBody>( READ_MAP.find("wall_down")->second );
  if( !wall_down ) 
    return false;
  pose = *wall_down->get_pose();
  pose.update_relative_pose( GLOBAL );
  c += pose.x;
  // Note:: following assumes centered at (0,0,0) and symmetric.  Dirty but works.
  e = Ravelin::Vector3d( std::max(pose.x[0], e[0]), std::max(pose.x[1], e[1]), std::max(pose.x[2], e[2]) );

  bounds = aabb_c( c, e );

  // --------------------------------------

  // compute the bounding planes
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

