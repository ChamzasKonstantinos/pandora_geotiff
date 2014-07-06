#include <map_writer_interface.h>
#include <map_writer_plugin_interface.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <fstream>

namespace pandora_geotiff_plugins
{

using namespace pandora_geotiff;

class MapCoverageWriter : public MapWriterPluginInterface
{
public:
  MapCoverageWriter();
  virtual ~MapCoverageWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface);
  void getGeotiffData(nav_msgs::OccupancyGrid map);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber map_sub;
  ros::Subscriber map_coverage_sub;

  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  
  
private:
   nav_msgs::OccupancyGrid map;

   bool gotData;
};

MapCoverageWriter::MapCoverageWriter()
    : initialized_(false)
{}

MapCoverageWriter::~MapCoverageWriter()
{}
void MapCoverageWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string map_topic_name_;
  std::string map_coverage_topic_name_;

  plugin_nh.param("/published_topic_names/map", map_topic_name_, std::string("/slam/occupancyGridMap"));
  plugin_nh.param("/published_topic_names/coverage_map", map_coverage_topic_name_, std::string("/data_fusion/sensor_coverage/coverage_map"));
  

  map_sub = plugin_nh.subscribe(map_topic_name_, 1000,  &MapCoverageWriter::getGeotiffData,this);
  //~ map_coverage_sub = plugin_nh.subscribe(map_coverage_topic_name_, 1000,  &MapCoverageWriter::getGeotiffData,this);
  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff MapCoverageWriter plugin %s.", name_.c_str());
}

void MapCoverageWriter::getGeotiffData(nav_msgs::OccupancyGrid map)
{
  this->map = map;

}
  
void MapCoverageWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;

    ROS_ERROR("Drawing the awesome map");
    
    interface->drawMap(&map);
}

} 
// namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::MapCoverageWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, MapCoverageWriter, pandora_geotiff_plugins::MapCoverageWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
