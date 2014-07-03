#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>


#include <Eigen/Geometry>

#include <QtGui/QApplication>


#include "GeotiffCreator.h"
#include <map_writer_plugin_interface.h>


class MapGenerator
{
private:  

  GeotiffCreator * geotiffCreator;
  
  
  std::string p_plugin_list_;
  ros::NodeHandle pn_;
  
  
  std::vector<boost::shared_ptr<pandora_geotiff::MapWriterPluginInterface> > plugin_vector_;
  pluginlib::ClassLoader<pandora_geotiff::MapWriterPluginInterface>* plugin_loader_;
  
public:
  MapGenerator();
  ~MapGenerator();
  
  void writeGeotiff();
  void timerSaveGeotiffCallback(const ros::TimerEvent& e);
};

  
#endif
