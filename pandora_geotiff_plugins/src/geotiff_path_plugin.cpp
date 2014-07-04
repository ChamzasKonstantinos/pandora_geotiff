//=================================================================================================
// Copyright (c) 2012, Gregor Gebhardt, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <pandora_geotiff/map_writer_interface.h>
#include <pandora_geotiff/map_writer_plugin_interface.h>

#include <ros/ros.h>

#include <pluginlib/class_loader.h>
#include <fstream>

namespace pandora_geotiff_plugins
{

using namespace pandora_geotiff;

class PathWriter : public MapWriterPluginInterface
{
public:
  PathWriter();
  virtual ~PathWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface);
  virtual void getGeotiffData();

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient service_client_;

  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  
  
private:

  int *pathx;
  int *pathy;
  int pathSize;

};

PathWriter::PathWriter()
    : initialized_(false)
{}

PathWriter::~PathWriter()
{}

void PathWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string service_name_;

  plugin_nh.param("service_name", service_name_, std::string("Path"));

  service_client_ = nh_.serviceClient<pandora_nav_msgs::GetRobotPath>(service_name_);

  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff MapWriter plugin %s.", name_.c_str());
}

PathWriter::getGeotiffData():
{
 pandora_nav_msgs::GetRobotPath path_srv;
    if (!service_client_.call(path_srv)) {
      ROS_ERROR_NAMED(name_, "Cannot draw Map, service %s failed", service_client_.getService().c_str());
      return;
    }
}
  
void PathWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;

    QColor pathColor;
    pathColor.setRgb(120,0,140);
    QPen pathPen(pathColor);
    pathPen.setWidth(2);
    mapPainter.setPen(pathPen);

for(int i=0; i<pathSize-1; i++){
  mapPainter.drawLine(pathx[i], ysize-1-pathy[i], pathx[i+1], ysize-1-pathy[i+1]);
  }
}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::PathWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, PathWriter, pandora_geotiff_plugins::PathWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
