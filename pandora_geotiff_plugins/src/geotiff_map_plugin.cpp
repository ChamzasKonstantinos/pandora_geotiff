#include <map_creator_interface.h>
#include <map_writer_plugin_interface.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>


namespace pandora_geotiff_plugins
{

using namespace pandora_geotiff;
  
  class MapWriter : public MapWriterPluginInterface
  {
  public:
    MapWriter();
    virtual ~MapWriter();
  
    virtual void initialize(const std::string& name);
    virtual void draw(MapWriterInterface *interface);
    void getGeotiffDataMap(nav_msgs::OccupancyGrid map);
  
  protected:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub;
  
    bool initialized_;
    std::string name_;
    
    
  private:

     nav_msgs::OccupancyGrid map;
     bool gotData;
     std::string MAP_COLOR;
     int MAP_BOT_THRES;
     int MAP_TOP_THRES;
     std::string WALL_COLOR;
     int WALL_BOT_THRES;
     int WALL_TOP_THRES;
  };
  
  MapWriter::MapWriter()
      : initialized_(false)
  {}
  
  MapWriter::~MapWriter()
  {}
  void MapWriter::initialize(const std::string& name)
  {
    ros::NodeHandle plugin_nh("~/" + name);
    std::string map_topic_name_;
  
    plugin_nh.param("/published_topic_names/map", map_topic_name_, std::string("/slam/map"));
    plugin_nh.param("MAP_BOT_THRES",MAP_BOT_THRES,0);
    plugin_nh.param("MAP_TOP_THRES",MAP_TOP_THRES,30);
    plugin_nh.param("MAP_COLOR",MAP_COLOR, std::string("WHITE_MAX"));
    plugin_nh.param("WALL_BOT_THRES",WALL_BOT_THRES,60);
    plugin_nh.param("WALL_TOP_THRES",WALL_TOP_THRES,250);
    plugin_nh.param("WALL_COLOR",WALL_COLOR, std::string("SOLID BLUE"));
      
    map_sub = plugin_nh.subscribe(map_topic_name_,
         1000,  &MapWriter::getGeotiffDataMap,this);
         
    initialized_ = true;
    this->name_ = name;
    ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff MapWriter plugin %s.", name_.c_str());
  }
  
  void MapWriter::getGeotiffDataMap(nav_msgs::OccupancyGrid map)
  {
    this->map = map;
    gotData = true;
  }

  void MapWriter::draw(MapWriterInterface *interface)
  {
       if(!initialized_||!gotData)
    {
      ROS_WARN_NAMED("MapWriter","plugin not initialized or no data has been received /n ABORTING DRAWING..");
      return;
      }

      ROS_ERROR("Drawing the awesome map");
      interface->drawMap(map, MAP_COLOR, MAP_BOT_THRES, MAP_TOP_THRES,1);
      interface->drawMap(map, WALL_COLOR, WALL_BOT_THRES, WALL_TOP_THRES, 0);
  }

}
// namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::MapWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, MapWriter, pandora_geotiff_plugins::MapWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
