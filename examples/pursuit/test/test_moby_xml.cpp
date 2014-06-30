#include "../moby_xml.h"

#include "../moby_space.h"

//-----------------------------------------------------------------------------

char spstr[512];
moby_space_p space;

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] ) {

  init_moby_xml( argc, argv );  

  boost::shared_ptr<const Ravelin::Pose3d> pose;
  Ravelin::SVelocityd dx;
  Ravelin::VectorNd u;
  
  space = moby_space_p( new moby_space_c() );

  if( !space->read( READ_MAP ) ) {
    sprintf( spstr, "failed to read spatial information\n" );
    printf( "%s", spstr );
    return 1;
  }

  // space was successfully read
  
  sprintf( spstr,"spatial aabb:[center[%f,%f,%f], extens[%f,%f,%f], x[%f,%f], y[%f,%f], z[%f,%f]]\n",space->bounds.center[0], space->bounds.center[1], space->bounds.center[2], space->bounds.extens[0], space->bounds.extens[1], space->bounds.extens[2], space->bounds.center[0]-space->bounds.extens[0], space->bounds.center[0]+space->bounds.extens[0], space->bounds.center[1]-space->bounds.extens[1], space->bounds.center[1]+space->bounds.extens[1], space->bounds.center[2]-space->bounds.extens[2], space->bounds.center[2]+space->bounds.extens[2]);
  printf( "%s", spstr );
  
  sprintf( spstr, "spatial planes: " );
  printf( "%s", spstr );
  for( unsigned i = 0; i < space->planes.size(); i++ ) {
    if( i > 0 ) { sprintf( spstr, ", " ); printf( "%s", spstr ); }
    sprintf( spstr, "plane[%d]{point[%f,%f,%f], normal[%f,%f,%f]}", i, space->planes[i].point[0], space->planes[i].point[1], space->planes[i].point[2], space->planes[i].normal[0], space->planes[i].normal[1], space->planes[i].normal[2] );
    printf( "%s", spstr );
  }
  sprintf( spstr, "\n" );
  printf( "%s", spstr );

  
  for( unsigned i = 0; i < space->ships.size(); i++ ) {
    ship_p ship = space->ships[i];
    if( ship->role == ship_c::PREY ) {
      //sprintf(spstr, "ship[%d]{prey[x[%f,%f,%f,%f,%f,%f,%f],dx[%f,%f,%f,%f,%f,%f]]}", i, ship->pose.x[0], ship->pose.x[1], ship->pose.x[2], ship->pose.q[0], ship->pose.q[1], ship->pose.q[2], ship->pose.q[3], ship->velocity[0], ship->velocity[1], ship->velocity[2], ship->velocity[3], ship->velocity[4], ship->velocity[5]);
      printf( "%s", spstr );
      if( ship->adversary->role != ship_c::PREDATOR ) {
        printf( "\n" );
        sprintf( spstr, "malformed adversary data for prey\n" );
        printf( "%s", spstr );
        return 1;
      }
      
      //sprintf(spstr, "adversary{predator[x[%f,%f,%f,%f,%f,%f,%f],dx[%f,%f,%f,%f,%f,%f]]}\n", ship->adversary->pose.x[0], ship->adversary->pose.x[1], ship->adversary->pose.x[2], ship->adversary->pose.q[0], ship->adversary->pose.q[1], ship->adversary->pose.q[2], ship->adversary->pose.q[3], ship->adversary->velocity[0], ship->adversary->velocity[1], ship->adversary->velocity[2], ship->adversary->velocity[3], ship->adversary->velocity[4], ship->adversary->velocity[5]);
      printf( "%s", spstr );      
    } else if( ship->role == ship_c::PREDATOR ) {
      //sprintf(spstr, "ship[%d]{predator[x[%f,%f,%f,%f,%f,%f,%f],dx[%f,%f,%f,%f,%f,%f]]}", i, ship->pose.x[0], ship->pose.x[1], ship->pose.x[2], ship->pose.q[0], ship->pose.q[1], ship->pose.q[2], ship->pose.q[3], ship->velocity[0], ship->velocity[1], ship->velocity[2], ship->velocity[3], ship->velocity[4], ship->velocity[5]);
      printf( "%s", spstr );
      if( ship->adversary->role != ship_c::PREY ) {
        printf( "\n" );
        sprintf( spstr, "malformed adversary data for predator\n" );
        printf( "%s", spstr );
        return 1;
      }
      
      //sprintf(spstr, "adversary{prey[x[%f,%f,%f,%f,%f,%f,%f],dx[%f,%f,%f,%f,%f,%f]]}\n", ship->adversary->pose.x[0], ship->adversary->pose.x[1], ship->adversary->pose.x[2], ship->adversary->pose.q[0], ship->adversary->pose.q[1], ship->adversary->pose.q[2], ship->adversary->pose.q[3], ship->adversary->velocity[0], ship->adversary->velocity[1], ship->adversary->velocity[2], ship->adversary->velocity[3], ship->adversary->velocity[4], ship->adversary->velocity[5]);
      printf( "%s", spstr );      
    } else {
      sprintf( spstr, "malformed ship data\n" );
      printf( "%s", spstr );
      return 1;
    }
  }


  printf( "passed\n" );

  return 0;
}
