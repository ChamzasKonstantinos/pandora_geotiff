#include <pandora_geotiff/map_writer_interface.h>
#include <pandora_geotiff/map_writer_plugin_interface.h>

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
  virtual void getGeotiffData();

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient service_client_;

  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  
  
private:

};

MapCoverageWriter::MapCoverageWriter()
    : initialized_(false)
{}

MapCoverageWriter::~MapCoverageWriter()
{}

void MapCoverageWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string service_name_;

  plugin_nh.param("service_name", service_name_, std::string("NAME_OF_SERVICE_WANTED"));

  service_client_ = nh_.serviceClient<pandora_nav_msgs::GetRobotPath>(service_name_);

  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff MapWriter plugin %s.", name_.c_str());
}

MapCoverageWriter::getGeotiffData():
{
 pandora_nav_msgs::GetRobotPath map_coverage_srv;
    if (!service_client_.call(map_coverage_srv)) {
      ROS_ERROR_NAMED(name_, "Cannot draw Map, service %s failed", service_client_.getService().c_str());
      return;
    }
}
  
  
}
void MapCoverageWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;


}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::MapCoverageWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, MapCoverageWriter, pandora_geotiff_plugins::MapCoverageWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
