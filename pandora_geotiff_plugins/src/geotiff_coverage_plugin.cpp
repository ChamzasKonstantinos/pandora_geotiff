#include <map_creator_interface.h>
#include <map_writer_plugin_interface.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

namespace pandora_geotiff_plugins
{

using namespace pandora_geotiff;

class CoverageWriter : public MapWriterPluginInterface
{
public:
  CoverageWriter();
  virtual ~CoverageWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface);
  void getGeotiffDataCoverage(nav_msgs::OccupancyGrid map);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_coverage;

  bool initialized_;
  std::string name_;  
  
private:

   nav_msgs::OccupancyGrid coverage;

   bool gotData;
   std::string COVERAGE_COLOR;
   int COV_BOT_THRES;
   int COV_TOP_THRES;
};

CoverageWriter::CoverageWriter()
    : initialized_(false)
{}

CoverageWriter::~CoverageWriter()
{}
void CoverageWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string coverage_topic_name_;

  plugin_nh.param("/published_topic_names/coverage_map", coverage_topic_name_,
    std::string("/data_fusion/sensor_coverage/coverage_map"));
  plugin_nh.param("COVERAGE_COLOR",COVERAGE_COLOR, std::string("LIGHT_GREEN_MAX"));
  plugin_nh.param("COV_BOT_THRES",COV_BOT_THRES,-5);
  plugin_nh.param("COV_TOP_THRES",COV_TOP_THRES,100);


  sub_coverage = plugin_nh.subscribe(coverage_topic_name_,
       1000,  &CoverageWriter::getGeotiffDataCoverage,this);
       
  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff CoverageWriter plugin %s.", name_.c_str());
}


void CoverageWriter::getGeotiffDataCoverage(nav_msgs::OccupancyGrid coverage)
{
  this->coverage = coverage;
  gotData = true;
}

void CoverageWriter::draw(MapWriterInterface *interface)
{
       if(!initialized_||!gotData)
    {
      ROS_WARN_NAMED("CoverageWriter","plugin not initilized or no data has been received /n ABORTING DRAWING..");
      return;
      }

    interface->drawMap(coverage,COVERAGE_COLOR,COV_BOT_THRES,COV_TOP_THRES);
}

} 
// namespace pandora_geotiff

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::CoverageWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, CoverageWriter, pandora_geotiff_plugins::CoverageWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
