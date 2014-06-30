#ifndef _PURSUIT_MOBY_XML_H_
#define _PURSUIT_MOBY_XML_H_

//-----------------------------------------------------------------------------

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <fstream>

#include <Moby/XMLReader.h>
#include <Moby/RigidBody.h>

//-----------------------------------------------------------------------------

using namespace Moby;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

//-----------------------------------------------------------------------------

std::map<std::string, BasePtr> READ_MAP;

//-----------------------------------------------------------------------------
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

//-----------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------

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

  // change back to current directory
  if( chdir(cwd.get()) == -1 ) {
    // Error
  }
}

//-----------------------------------------------------------------------------
int init_moby_xml( int argc, char* argv[] ) {
  std::string scene_path;

  // check that syntax is ok
  if (argc < 2) {
    std::cerr << "syntax: driver [OPTIONS] <xml file>" << std::endl;
    std::cerr << "        (see README for OPTIONS)" << std::endl;
    return 1;
  }

  // setup the simulation
  READ_MAP = XMLReader::read(std::string(argv[argc-1]));

  // process XML options
  process_xml_options(std::string(argv[argc-1]));
}


//-----------------------------------------------------------------------------

#endif // _PURSUIT_MOBY_XML_H_
