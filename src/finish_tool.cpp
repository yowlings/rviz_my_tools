/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "finish_tool.h"
#include <iostream>
#include <fstream>
#include <cv.h>
#include <highgui.h>
#include <pthread.h> 
using namespace cv;
using namespace std;


namespace rviz_my_tools
{

// BEGIN_TUTORIAL
// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
FinishTool::FinishTool()  
{
  shortcut_key_ = 'f';
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
FinishTool::~FinishTool()
{
  
}



// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving flag node to be visible, then we create an
// rviz::VectorProperty to show the user the position of the flag.
// Unlike rviz::Display, rviz::Tool is not a subclass of
// rviz::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_flag_property_ to be read-only, but
// if it were writable the flag should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.
void* reloadmapf(void* args)
{
  char *q=getenv("USER");
  string username(q);
  string rootpath="/home/";
  rootpath+=username;
  rootpath+="/cafe_robot_single/";
  string filepath4=rootpath+"src/nav_staff/map/edited_map.yaml";
  string command="rosrun map_server map_server "+filepath4;
  char* c5;
  int len = command.length();
  c5 =new char[len+1];
  strcpy(c5,command.c_str());
  system(c5);

}

void FinishTool::activate()
{
  Mat image;
  char *q=getenv("USER");
  string username(q);
  string rootpath="/home/";
  rootpath+=username;
  rootpath+="/cafe_robot_single/";
  string filepath1=rootpath+"src/nav_staff/map/office_map_manual.pgm";
//  char* c1;
//  int len = filepath1.length();
//  c1 =new char[len+1];
//  strcpy(c1,filepath1.c_str());

  image = imread(filepath1);
  Mat outimage;
  cvtColor( image, outimage, CV_BGR2GRAY );
  int nRows = outimage.rows;
  int nCols = outimage.cols;
  

  float kind,i,j;
  uchar* p;

  
  string filepath2=rootpath+"obstacles.txt";  
  char* c2;
  int len = filepath2.length();
  c2 =new char[len+1];
  strcpy(c2,filepath2.c_str());
  fstream file;
  file.open(c2,ios::in);//example.txt是你要输出的文件的名字


  while(!file.eof())
  {
    file>>kind;
    file>>i;
    file>>j;
    for( int u = nRows-20*j-3; u < nRows-20*j+4; ++u)
    {
        p = outimage.ptr<uchar>(u);
        for ( int v = 20*i-3; v < 20*i+4; ++v)
        {
            p[v] = 0;
        }
    }
  }
  file.close();

  string command="rm ";
  command+=filepath2;
  char* c4;
  len = command.length();
  c4 =new char[len+1];
  strcpy(c4,command.c_str());
  system(c4);
    
  namedWindow( "zly", CV_WINDOW_AUTOSIZE );
  namedWindow("outimage",CV_WINDOW_AUTOSIZE);
  string filepath3=rootpath+"src/nav_staff/map/edited_map.pgm";
//  char* c3;
//  len = filepath3.length();
//  c3 =new char[len+1];
//  strcpy(c3,filepath3.c_str());
  imwrite(filepath3,outimage);
  imshow( "zly", image );
  imshow("outimage",outimage);
  // waitKey(0);
  system("rosnode kill map_server");
  // system("rosrun rqt_graph rqt_graph");
  // sleep(10000);
  pthread_t reloadmapt;
  pthread_create(&reloadmapt,NULL,reloadmapf,NULL);
  

  
  
}



// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current flag
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of flags when
// we switch to another tool.
void FinishTool::deactivate()
{
  
}



// End of .cpp file
// ^^^^^^^^^^^^^^^^
//
// At the end of every plugin class implementation, we end the
// namespace and then tell pluginlib about the class.  It is important
// to do this in global scope, outside our package's namespace.

} // end namespace rviz_my_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_my_tools::FinishTool,rviz::Tool )
// END_TUTORIAL
