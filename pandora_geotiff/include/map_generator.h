#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/String.h>

#include <Eigen/Geometry>

#include <QtGui/QApplication>

#include "pandora_geotiff/SaveMission.h"
#include "GeotiffCreator.h"
#include <map_writer_plugin_interface.h>


class MapGenerator
{
private:  

  GeotiffCreator * geotiffCreator;
  
  
  std::string p_plugin_list_;
  ros::NodeHandle pn_;
  ros::ServiceServer save_mission_service;
  std::vector<boost::shared_ptr<pandora_geotiff::MapWriterPluginInterface> > plugin_vector_;
  pluginlib::ClassLoader<pandora_geotiff::MapWriterPluginInterface>* plugin_loader_;
  
public:
  MapGenerator();
  ~MapGenerator();
  
  void writeGeotiff();
  bool saveGeotiff(pandora_geotiff::SaveMission::Request& req ,
    pandora_geotiff::SaveMission::Response& res );
};

  
#endif
