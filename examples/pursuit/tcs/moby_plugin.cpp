/*----------------------THE GEORGE WASHINGTON UNIVERSITY-----------------------
author: James R. Taylor                                             jrt@gwu.edu

dynamics.cpp
Moby plugin
-----------------------------------------------------------------------------*/
///*
#include <stdio.h>

#include <errno.h>
#include <sys/time.h>
#include <dlfcn.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <boost/foreach.hpp>
#include <Moby/XMLReader.h>
#include <Ravelin/AAngled.h>

#ifdef USE_OSG
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Geode>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#endif

#include <Moby/Log.h>
#include <Moby/Simulator.h>
#include <Moby/RigidBody.h>
#include <Moby/EventDrivenSimulator.h>
#include <Moby/RCArticulatedBody.h>
//*/
#include <tcs/dynamics_data.h>
#include <tcs/time.h>
//#include "moby_xml.h"
/*
#include <tas/tas.h>
#include <tas/actuator.h>
#include <tas/log.h>
#include <tas/ipc.h>
*/

#include "../moby_space.h"
#include "../ship.h"

typedef double Real;

//-----------------------------------------------------------------------------

using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

//log_c dyn_error_log;
char spstr[ 512 ];

//-----------------------------------------------------------------------------
// MOBY driver
//-----------------------------------------------------------------------------

/// Handle for dynamic library loading
void* HANDLE = NULL;

/// Horizontal and vertical resolutions for offscreen-rendering
const unsigned HORZ_RES = 1024;
const unsigned VERT_RES = 768;

/// Beginning iteration for logging
unsigned LOG_START = 0;

/// Ending iteration for logging
unsigned LOG_STOP = std::numeric_limits<unsigned>::max();

/// The logging reporting level
unsigned LOG_REPORTING_LEVEL = 0;

/// Used for timing
clock_t start_time;

/// The default simulation step size
const Real DEFAULT_STEP_SIZE = .001;

/// The simulation step size
Real STEP_SIZE = DEFAULT_STEP_SIZE;

/// The time of the first simulation step
Real FIRST_STEP_TIME = -1;

/// The time of the last simulation step
Real LAST_STEP_TIME = 0;

/// The current simulation iteration
unsigned ITER = 0;

/// Interval for offscreen renders (0=offscreen renders disabled)
unsigned IMAGE_IVAL = 0;

/// Interval for 3D outputs (0=3D outputs disabled)
unsigned THREED_IVAL = 0;

/// Determines whether to do onscreen rendering (false by default)
bool ONSCREEN_RENDER = false;

/// Determines whether to output timings
bool OUTPUT_TIMINGS = false;

/// Extension/format for 3D outputs (default=Wavefront obj)
char THREED_EXT[5] = "obj";

/// Determines whether to update graphics (false by default, but certain
/// options will set to true)
bool UPDATE_GRAPHICS = false;

/// If true, outputs (image, vrml) tied to simulation time rather than actual iteration count
bool OUTPUT_TO_TIME = false;

/// The maximum number of iterations (default infinity)
unsigned MAX_ITER = std::numeric_limits<unsigned>::max();

/// The maximum time of the simulation (default infinity)
Real MAX_TIME = std::numeric_limits<Real>::max();

/// The total (CPU) clock time used by the simulation
Real TOTAL_TIME = 0.0;

/// Last 3D output iteration and time output
unsigned LAST_3D_WRITTEN = -1;
Real LAST_3D_WRITTEN_T = -std::numeric_limits<Real>::max()/2.0;

/// Last image iteration output
unsigned LAST_IMG_WRITTEN = -1;
Real LAST_IMG_WRITTEN_T = -std::numeric_limits<Real>::max()/2.0;

/// Outputs to stdout
bool OUTPUT_FRAME_RATE = false;
bool OUTPUT_ITER_NUM = false;
bool OUTPUT_SIM_RATE = false;

/// Render Contact Points
bool RENDER_CONTACT_POINTS = false;

/// The map of objects read from the simulation XML file
std::map<std::string, BasePtr> READ_MAP;

#ifdef USE_OSG
/// The OpenInventor group node for Moby
osg::Group* MOBY_GROUP;

/// The OpenInventor root group node for this application
osg::Group* MAIN_GROUP;

/// Pointer to the viewer
osgViewer::Viewer* viewer_pointer;
#endif

/**
  // Note: In this architecture, Moby doesn't have any direct relationship to
  // controllers, so the ability to load control plugins is removed

/// Pointer to the controller's initializer, called once (if any)
typedef void (*init_t)(void*, const std::map<std::string, BasePtr>&, Real);
std::list<init_t> INIT;
*/
/*
extern bool ONSCREEN_RENDER;
extern std::map<std::string, BasePtr> READ_MAP;
#ifdef USE_OSG
extern osgViewer::Viewer* viewer_pointer;
#endif
*/

//-----------------------------------------------------------------------------
// Maybe should be inside extern "C" scope
static moby_space_p space;
static ship_p predator;
static ship_p prey;

std::vector<double> prey_state;
std::vector<double> pred_state;
std::vector<double> prey_control;
std::vector<double> pred_control;

//-----------------------------------------------------------------------------

/// Checks whether was compiled with OpenSceneGraph support
bool check_osg() {
 #ifdef USE_OSG
  return true;
 #else
  return false;
 #endif
}

//-----------------------------------------------------------------------------

/// TODO : REFACTOR using TAS Standard
/// Gets the current time (as a floating-point number)
Real get_current_time() {
  const Real MICROSEC = 1.0/1000000;
  timeval t;
  gettimeofday(&t, NULL);
  return (Real) t.tv_sec + (Real) t.tv_usec * MICROSEC;
}

//-----------------------------------------------------------------------------

/// runs the simulator and updates all transforms
void step(void* arg) {
  // get the simulator pointer
  boost::shared_ptr<Simulator> sim = *(boost::shared_ptr<Simulator>*) arg;

  // get the simulator as event driven simulation
  boost::shared_ptr<EventDrivenSimulator> edsim = boost::dynamic_pointer_cast<EventDrivenSimulator>( sim );

  // see whether to activate logging
  if (ITER >= LOG_START && ITER <= LOG_STOP)
    Log<OutputToFile>::reporting_level = LOG_REPORTING_LEVEL;
  else
    Log<OutputToFile>::reporting_level = 0;

    // output the iteration #
  if (OUTPUT_ITER_NUM)
    std::cout << "iteration: " << ITER << "  simulation time: " << sim->current_time << std::endl;
  if (Log<OutputToFile>::reporting_level > 0)
    FILE_LOG(Log<OutputToFile>::reporting_level) << "iteration: " << ITER << "  simulation time: " << sim->current_time << std::endl;

  // only update the graphics if it is necessary
  if (UPDATE_GRAPHICS)
    sim->update_visualization();

  // output the image, if desired
 #ifdef USE_OSG
  if (IMAGE_IVAL > 0) {
    // determine at what iteration nearest frame would be output
    if ((OUTPUT_TO_TIME && sim->current_time - LAST_IMG_WRITTEN_T > STEP_SIZE * IMAGE_IVAL) || (!OUTPUT_TO_TIME && ITER % IMAGE_IVAL == 0)) {
      char buffer[128];
      sprintf(buffer, "driver.out.%08u.png", ++LAST_IMG_WRITTEN);
      // TODO: call offscreen renderer
      LAST_IMG_WRITTEN_T = sim->current_time;
    }
  }

  // output the 3D file, if desired
  if (THREED_IVAL > 0) {
    // determine at what iteration nearest frame would be output
    if ((OUTPUT_TO_TIME && sim->current_time - LAST_3D_WRITTEN_T > STEP_SIZE * THREED_IVAL) || (!OUTPUT_TO_TIME && ITER % THREED_IVAL == 0)) {
      // write the file (fails silently)
      char buffer[128];
      sprintf(buffer, "driver.out-%08u-%f.%s", ++LAST_3D_WRITTEN, sim->current_time, THREED_EXT);
      osgDB::writeNodeFile(*MAIN_GROUP, std::string(buffer));
      LAST_3D_WRITTEN_T = sim->current_time;
    }
  }
  #endif

  // step the simulator and update visualization
  clock_t pre_sim_t = clock();
  sim->step(STEP_SIZE);
  clock_t post_sim_t = clock();
  Real total_t = (post_sim_t - pre_sim_t) / (Real) CLOCKS_PER_SEC;
  TOTAL_TIME += total_t;

  // output the iteration / stepping rate
  if (OUTPUT_SIM_RATE)
    std::cout << "time to compute last iteration: " << total_t << " (" << TOTAL_TIME / ITER << "s/iter, " << TOTAL_TIME / sim->current_time << "s/step)" << std::endl;
/*
  // see whether to output the timings
  if (OUTPUT_TIMINGS) {
    if (!edsim)
      std::cout << ITER << " " << sim->dynamics_utime << " " << sim->dynamics_stime << std::endl;
    else
      std::cout << ITER << " " << edsim->dynamics_utime << " " << edsim->dynamics_stime << " " << edsim->coldet_utime << " " << edsim->coldet_stime << " " << edsim->event_utime << " " << edsim->event_stime << std::endl;
  }
*/
  // update the iteration #
  ITER++;

  // output the frame rate, if desired
  if (OUTPUT_FRAME_RATE) {
    Real tm = get_current_time();
    std::cout << "instantaneous frame rate: " << (1.0/(tm - LAST_STEP_TIME)) << "fps  avg. frame rate: " << (ITER / (tm - FIRST_STEP_TIME)) << "fps" << std::endl;
    LAST_STEP_TIME = tm;
  }

  // check that maximum number of iterations or maximum time not exceeded
  if (ITER >= MAX_ITER || sim->current_time > MAX_TIME) {
    clock_t end_time = clock();
    Real elapsed = (end_time - start_time) / (Real) CLOCKS_PER_SEC;
    std::cout << elapsed << " seconds elapsed" << std::endl;
    exit(0);
  }

  // if render contact points enabled, notify the Simulator
  if( RENDER_CONTACT_POINTS && edsim)
    edsim->render_contact_points = true;
}

//-----------------------------------------------------------------------------

/// Adds lights to the scene when no scene background file specified
void add_lights() {
 #ifdef USE_OSG
  // add lights
 #endif
}

//-----------------------------------------------------------------------------
///*
/// Gets the XML sub-tree rooted at the specified tag
shared_ptr<const XMLTree> find_subtree(shared_ptr<const XMLTree> root, const std::string& name) {

  // if we found the tree, return it
  if (strcasecmp(root->name.c_str(), name.c_str()) == 0)
    return root;

  // otherwise, look for it recursively
  const std::list<XMLTreePtr>& children = root->children;
  for (std::list<XMLTreePtr>::const_iterator i = children.begin(); i != children.end(); i++) {
    shared_ptr<const XMLTree> node = find_subtree(*i, name);
    if (node)
      return node;
  }

  // return NULL if we are here
  return shared_ptr<const XMLTree>();
}
//*/
//-----------------------------------------------------------------------------
///*
// finds and processes given XML tags
void process_tag(const std::string& tag, shared_ptr<const XMLTree> root, void (*fn)(shared_ptr<const XMLTree>)) {
  // if this node is of the given type, process it 
  if (strcasecmp(root->name.c_str(), tag.c_str()) == 0) {
    fn(root);
  } else {
    const std::list<XMLTreePtr>& child_nodes = root->children;
    for (std::list<XMLTreePtr>::const_iterator i = child_nodes.begin(); i != child_nodes.end(); i++)
      process_tag(tag, *i, fn);
  }
}
//*/
//-----------------------------------------------------------------------------
///*
/// processes the 'camera' tag
void process_camera_tag(boost::shared_ptr<const XMLTree> node) {
  if (!ONSCREEN_RENDER)
    return;

  // read all attributes
  XMLAttrib* target_attr = node->get_attrib("target");
  XMLAttrib* position_attr = node->get_attrib("position");
  XMLAttrib* up_attr = node->get_attrib("up");
  if (!target_attr || !position_attr || !up_attr)
    return;

  // get the actual values
  Ravelin::Vector3d target, position, up;
  target_attr->get_vector_value(target);
  position_attr->get_vector_value(position);
  up_attr->get_vector_value(up);

  // setup osg vectors
 #ifdef USE_OSG
  osg::Vec3d position_osg(position[0], position[1], position[2]);
  osg::Vec3d target_osg(target[0], target[1], target[2]);
  osg::Vec3d up_osg(up[0], up[1], up[2]);

  // set the camera view
  if (viewer_pointer && viewer_pointer->getCameraManipulator()) {
    osg::Camera* camera = viewer_pointer->getCamera();
    camera->setViewMatrixAsLookAt(position_osg, target_osg, up_osg);

    // setup the manipulator using the camera, if necessary
    viewer_pointer->getCameraManipulator()->setHomePosition(position_osg, target_osg, up_osg);
  }
 #endif
}
//*/
//-----------------------------------------------------------------------------
///*
/// processes the 'window' tag
void process_window_tag(boost::shared_ptr<const XMLTree> node) {
  // don't process if not onscreen rendering
  if (!ONSCREEN_RENDER)
    return;

  // read window location
  XMLAttrib* loc_attr = node->get_attrib("location");

  // read window size
  XMLAttrib* size_attr = node->get_attrib("size");

  // get the actual values
  Ravelin::Vector2d loc(0,0), size(640,480);
  if (loc_attr)
    loc_attr->get_vector_value(loc);
  if (size_attr)
    size_attr->get_vector_value(size);

 #ifdef USE_OSG
  // setup the window
  viewer_pointer->setUpViewInWindow(loc[0], loc[1], size[0], size[1]);
 #endif
}
//*/
//-----------------------------------------------------------------------------
///*
/// processes all 'driver' options in the XML file
void process_xml_options(const std::string& xml_fname) {
  // *************************************************************
  // going to remove any path from the argument and change to that
  // path; this is done so that all files referenced from the
  // local path of the XML file are found
  // *************************************************************

  // set the filename to use as the argument, by default
  std::string filename = xml_fname;

  // get the current pathname
  size_t BUFSIZE = 128;
  boost::shared_array<char> cwd;
  while (true) {
    cwd = boost::shared_array<char>((new char[BUFSIZE]));
    if (getcwd(cwd.get(), BUFSIZE) == cwd.get())
      break;
    if (errno != ERANGE) {
      std::cerr << "process_xml_options() - unable to allocate sufficient memory!" << std::endl;
      return;
    }
    BUFSIZE *= 2;
  }

  // separate the path from the filename
  size_t last_path_sep = xml_fname.find_last_of('/');
  if (last_path_sep != std::string::npos) {
    // get the new working path
    std::string pathname = xml_fname.substr(0,last_path_sep+1);

    // change to the new working path
    if( chdir(pathname.c_str()) == -1 ) {
      // Error
     }

    // get the new filename
    filename = xml_fname.substr(last_path_sep+1,std::string::npos);
  }

  // read the XML Tree
  //XMLTreePtr driver_tree = XMLTree::read_from_xml(filename);
  shared_ptr<const XMLTree> driver_tree = XMLTree::read_from_xml(filename);
  if (!driver_tree) {
    std::cerr << "process_xml_options() - unable to open file " << xml_fname;
    std::cerr << " for reading" << std::endl;
    chdir(cwd.get());
    return;
  }

  // find the driver tree
  driver_tree = find_subtree(driver_tree, "driver");

  // make sure that the driver node was found
  if (!driver_tree) {
    if( chdir(cwd.get()) == -1 ) {
            // Error
    }
    return;
  }

  // process tags
  process_tag("window", driver_tree, process_window_tag);
  process_tag("camera", driver_tree, process_camera_tag);

  // change back to current directory
  if( chdir(cwd.get()) == -1 ) {
    // Error
  }
}
//*/
//-----------------------------------------------------------------------------
// MOBY as a plugin.
// c_dynamics_plugin
//-----------------------------------------------------------------------------

// Plugins have to extern "C"
extern "C" {

//-----------------------------------------------------------------------------

boost::shared_ptr<Simulator> sim;
osgViewer::Viewer viewer;
const Real DYNAMICS_FREQ = 0.001;
//RCArticulatedBodyPtr pendulum;

//-----------------------------------------------------------------------------
// Dynamics plugin entry point.  
bool init( int argc, char** argv ) {
/*
  //+++++++++++++++++++++++++++++++++++++++
  dyn_error_log = log_c( FD_ERROR_LOG );
  if( dyn_error_log.open( ) != LOG_ERROR_NONE ) {
    // Note: this is not really necessary
    printf( "(dynamics.cpp) ERROR: init(argc,argv) failed calling log_c.open() on FD_ERROR_LOG\n" );
  }
  //+++++++++++++++++++++++++++++++++++++++
*/
  const unsigned ONECHAR_ARG = 3, TWOCHAR_ARG = 4;

 #ifdef USE_OSG
  viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
  viewer_pointer = &viewer;
 #endif

  // setup some default options
  std::string scene_path;
  THREED_IVAL = 0;
  IMAGE_IVAL = 0;

  // check that syntax is ok
  if (argc < 2) {
    std::cerr << "syntax: driver [OPTIONS] <xml file>" << std::endl;
    std::cerr << "        (see README for OPTIONS)" << std::endl;
    return false;
  }

  // get all options
  for (int i=1; i< argc-1; i++) {
    // get the option
    std::string option(argv[i]);

    // process options
    if (option == "-r") {
      ONSCREEN_RENDER = true;
      UPDATE_GRAPHICS = true;
      check_osg();
    } else if (option.find("-of") != std::string::npos) {
      OUTPUT_FRAME_RATE = true;
    } else if (option.find("-ot") != std::string::npos) {
      OUTPUT_TIMINGS = true;
    } else if (option.find("-oi") != std::string::npos) {
      OUTPUT_ITER_NUM = true;
    } else if (option.find("-or") != std::string::npos) {
      OUTPUT_SIM_RATE = true;
    } else if (option.find("-v=") != std::string::npos) {
      UPDATE_GRAPHICS = true;
      check_osg();
      THREED_IVAL = std::atoi(&argv[i][ONECHAR_ARG]);
      //assert(THREED_IVAL >= 0);     // Note: remarked because always true
    } else if (option.find("-i=") != std::string::npos) {
      check_osg();
      UPDATE_GRAPHICS = true;
      IMAGE_IVAL = std::atoi(&argv[i][ONECHAR_ARG]);
      //assert(IMAGE_IVAL >= 0);      // Note: remarked because always true
    } else if (option.find("-t") != std::string::npos) {
      OUTPUT_TO_TIME = true;
    } else if (option.find("-s=") != std::string::npos) {
      STEP_SIZE = std::atof(&argv[i][ONECHAR_ARG]);
      assert(STEP_SIZE >= 0.0 && STEP_SIZE < 1);
    } else if (option.find("-lf=") != std::string::npos) {
      std::string fname(&argv[i][TWOCHAR_ARG]);
      OutputToFile::stream.open(fname.c_str());
    } else if (option.find("-l=") != std::string::npos) {
      LOG_REPORTING_LEVEL = std::atoi(&argv[i][ONECHAR_ARG]);
      Log<OutputToFile>::reporting_level = LOG_REPORTING_LEVEL;
    } else if (option.find("-lt=") != std::string::npos) {
      LOG_START = std::atoi(&argv[i][TWOCHAR_ARG]);
    } else if (option.find("-lp=") != std::string::npos) {
      LOG_STOP = std::atoi(&argv[i][TWOCHAR_ARG]);
    } else if (option.find("-mi=") != std::string::npos) {
      MAX_ITER = std::atoi(&argv[i][TWOCHAR_ARG]);
      assert(MAX_ITER > 0);
    } else if (option.find("-mt=") != std::string::npos) {
      MAX_TIME = std::atof(&argv[i][TWOCHAR_ARG]);
      assert(MAX_TIME > 0);
    } else if (option.find("-x=") != std::string::npos) {
      check_osg();
      scene_path = std::string(&argv[i][ONECHAR_ARG]);
    } else if (option.find("-p=") != std::string::npos) {
      /// read_plugin(&argv[i][ONECHAR_ARG]);
    } else if (option.find("-y=") != std::string::npos) {
      strcpy(THREED_EXT, &argv[i][ONECHAR_ARG]);
    } else if (option.find("-vcp") != std::string::npos) {
      RENDER_CONTACT_POINTS = true;
    }
  }

  // setup the simulation
  READ_MAP = XMLReader::read(std::string(argv[argc-1]));

  // setup the offscreen renderer if necessary
 #ifdef USE_OSG
  if (IMAGE_IVAL > 0) {
    // TODO: setup offscreen renderer here
  }
 #endif

  // get the (only) simulation object

  for (std::map<std::string, BasePtr>::const_iterator i = READ_MAP.begin(); i != READ_MAP.end(); i++) {
    sim = boost::dynamic_pointer_cast<Simulator>(i->second);
    if (sim)
      break;
  }

  // make sure that a simulator was found
  if (!sim) {
    std::cerr << "driver: no simulator found in " << argv[argc-1] << std::endl;
    //return -1;
    return false;
  }

  // setup osg window if desired
 #ifdef USE_OSG
  if (ONSCREEN_RENDER) {
    // setup any necessary handlers here
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);
    viewer.addEventHandler(new osgViewer::StatsHandler);
  }

  // init the main group
  MAIN_GROUP = new osg::Group;
 #endif

  // look for a scene description file
 #ifdef USE_OSG
  if (scene_path != "") {
    std::ifstream in(scene_path.c_str());
    if (in.fail()) {
      std::cerr << "driver: unable to find scene description from " << scene_path << std::endl;
      add_lights();
    } else {
      in.close();
      osg::Node* node = osgDB::readNodeFile(scene_path);
      if (!node) {
        std::cerr << "driver: unable to open scene description file!" << std::endl;
        add_lights();
    } else
      MAIN_GROUP->addChild(node);
    }
  } else {
    add_lights();
  }
 #endif

  // process XML options
  process_xml_options(std::string(argv[argc-1]));

  // get the simulator visualization
 #ifdef USE_OSG
  MAIN_GROUP->addChild(sim->get_persistent_vdata());
  MAIN_GROUP->addChild(sim->get_transient_vdata());
 #endif

  // setup the timers
  FIRST_STEP_TIME = get_current_time();
  LAST_STEP_TIME = FIRST_STEP_TIME;

  // begin timing
  start_time = clock();

  // prepare to render
 #ifdef USE_OSG
  if (ONSCREEN_RENDER) {
    viewer.setSceneData(MAIN_GROUP);
    viewer.realize();
  }
 #endif

  // custom implementation follows

  space = moby_space_p( new moby_space_c() );
  if( !space->read( READ_MAP ) ) printf( "ERROR: failed to read dynamics\n" );
  prey = space->prey();
  predator = space->predator();

  prey_state.resize( 13 );
  pred_state.resize( 13 );
  prey_control.resize( 6 );
  pred_control.resize( 6 );

  for( unsigned i = 0; i < 13; i++ ) {
    prey_state[i] = 0;
    pred_state[i] = 0;
  }
  prey_state[6] = 1;  // set quaternion w coordinate
  pred_state[6] = 1;  // set quaternion w coordinate

  for( unsigned i = 0; i < 6; i++ ) {
    prey_control[i] = 0;
    pred_control[i] = 0;
  }
  

  printf( "(dynamics::initialized)\n" );

  return true;
}

//-----------------------------------------------------------------------------

void shutdown( void ) {

}

//-----------------------------------------------------------------------------
void get_state( dynamics_data_t* data ) {
  timestamp_t ts;

  prey->read( prey_state );
  predator->read( pred_state );

  for( unsigned i = 0; i < 7; i++ ) {
    data->prey_state.q[i] = prey_state[i];
    data->pred_state.q[i] = pred_state[i];
  } 
  for( unsigned i = 0; i < 6; i++ ) {
    data->prey_state.dq[i] = prey_state[i+7];
    data->pred_state.dq[i] = pred_state[i+7];
  } 
  data->prey_state.ts = ts;
  data->pred_state.ts = ts;
 
}
//-----------------------------------------------------------------------------

// Step the dynamics forward
void step( dynamics_data_t* data ) {
  timestamp_t ts;

 #ifdef USE_OSG
  if( ONSCREEN_RENDER ) {
    if( viewer.done( ) ) return;
      viewer.frame( );
  }
 #endif

  // reset accumulators
  prey->reset();
  predator->reset();

  // map the controls in 
  for( unsigned i = 0; i < 6; i++ ) {
    prey_control[i] = data->prey_control.u[i];
    pred_control[i] = data->pred_control.u[i];
  }
  prey->write( prey_control );
  predator->write( pred_control );

  // step dynamics
  step( (void*) &sim );

  // update time data
  data->t = sim->current_time;
  data->dt = STEP_SIZE;
  ts = generate_timestamp(); 

  // map the state out
  prey->read( prey_state );
  predator->read( pred_state );

  for( unsigned i = 0; i < 7; i++ ) {
    data->prey_state.q[i] = prey_state[i];
    data->pred_state.q[i] = pred_state[i];
  } 
  for( unsigned i = 0; i < 6; i++ ) {
    data->prey_state.dq[i] = prey_state[i+7];
    data->pred_state.dq[i] = pred_state[i+7];
  } 
  data->prey_state.ts = ts;
  data->pred_state.ts = ts;

  prey->read_ke( data->prey_ke );
  predator->read_ke( data->pred_ke );
}

//-----------------------------------------------------------------------------
Real get_step_size( void ) {
  return STEP_SIZE;
}

//-----------------------------------------------------------------------------

} // extern "C"

//-----------------------------------------------------------------------------
