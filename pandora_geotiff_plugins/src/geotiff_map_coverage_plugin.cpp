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

  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  
  
private:
   std::vector<int> *map; 

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
  std::string topic_name_;

  plugin_nh.param("/slam/occupancyGridMap", topic_name_, std::string("/slam/occupancyGridMap"));
  

  map_sub = plugin_nh.subscribe(topic_name_, 1000,  &MapCoverageWriter::getGeotiffData,this);

  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully inzxzxxitialized pandora_geotiff MapCoverageWriter plugin %s.", name_.c_str());
}

void MapCoverageWriter::getGeotiffData(nav_msgs::OccupancyGrid map)
{

  int xsize = map.info.width;
  int ysize = map.info.height;
  
  for ( int i =0 ; i < ysize*xsize; i++)
  {
    this->map->push_back(map.data[i]);
  }
  //~ this->map = new std::vector<int8_t>[xsize*ysize];
  
  
  //~ coverage = new uchar[xsize*ysize];
//~ 
  //~ for(int i=0; i<xsize; i++){
    //~ for(int j=0; j<ysize; j++){
      //~ map[i + (j*xsize)] = map_coverage_srv.response.map[j + (i*ysize)];
      //~ coverage[i + (j*xsize)] = map_coverage_srv.response.coverage[j + (i*ysize)];
  //~ }
}
  
  

void MapCoverageWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;
    
    
    ROS_ERROR("AAAAAAAAAAA");
    

// *************draw map & coverage
  //~ QImage mapIm(xsize, ysize, QImage::Format_ARGB32);
  //~ QPainter mapPainter;
  //~ mapPainter.begin(&mapIm);
//~ 
//~ 
//~ 
  //~ mapPainter.setCompositionMode(QPainter::CompositionMode_Source);
  //~ mapPainter.fillRect(mapIm.rect(), Qt::transparent);
  //~ mapPainter.setCompositionMode(QPainter::CompositionMode_SourceOver);
//~ 
//~ 
  //~ QColor wallsObstaclesColor;
  //~ wallsObstaclesColor.setRgb(0, 40, 120);
//~ 
  //~ for(int i=0; i<xsize; i++){
    //~ for(int j=0; j<ysize; j++){
      //~ if(map[j*xsize + i] < 127 && map[j*xsize + i] > 0){
//~ 
        //~ QPen wallsObstaclesPen(wallsObstaclesColor);
        //~ wallsObstaclesPen.setWidth(3);
        //~ mapPainter.setPen(wallsObstaclesPen);
        //~ mapPainter.drawPoint(i,ysize-1-j);
      //~ }
      //~ else if(map[j*xsize + i] > 170 ){
       //~ QColor searchedAreaColor;
       //~ searchedAreaColor.setRgb(map[j*xsize + i], map[j*xsize + i], map[j*xsize + i]);
       //~ mapPainter.setPen(searchedAreaColor);
       //~ mapPainter.drawPoint(i,ysize-1-j);
      //~ }
//~ 
      //~ QColor coverageColor;
      //~ if(coverage[j*xsize + i]){
        //~ if(map[j*xsize + i] < 170 && map[j*xsize + i] >= 127)
          //~ continue;
        //~ if(coverage[j*xsize + i] <= 85){
          //~ int colorTmp = (coverage[j*xsize + i]*17) / 255;
          //~ coverageColor.setRgb(163+colorTmp, 230, 163+colorTmp);
          //~ mapPainter.setPen(coverageColor);
//~ 
          //~ mapPainter.drawPoint(i,ysize-1-j);
        //~ }
        //~ else if(coverage[j*xsize + i] <= 170){
          //~ int colorTmp = (coverage[j*xsize + i]*17) / 170;
          //~ coverageColor.setRgb(146+colorTmp, 230, 146+colorTmp);
          //~ mapPainter.setPen(coverageColor);
//~ 
          //~ mapPainter.drawPoint(i,ysize-1-j);
        //~ } 
        //~ else {
         //~ int colorTmp = (coverage[j*xsize + i]*16) / 85;
         //~ coverageColor.setRgb(130+colorTmp, 230, 130+colorTmp);
         //~ mapPainter.setPen(coverageColor);
//~ 
         //~ mapPainter.drawPoint(i,ysize-1-j);
        //~ }
      //~ }
    //~ }
  //~ }
//~ 
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
