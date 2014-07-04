#include <pandora_geotiff/map_writer_interface.h>
#include <pandora_geotiff/map_writer_plugin_interface.h>

#include <ros/ros.h>

#include <pluginlib/class_loader.h>
#include <fstream>

namespace pandora_geotiff_plugins
{

using namespace pandora_geotiff;

class QrWriter : public MapWriterPluginInterface
{
public:
  QrWriter();
  virtual ~QrWriter();

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
  int *qrx;
  int *qry;
  int qrSize;
  time_t *qrTime;
  std::string *qrType;
  qrWorldX = 0;
  qrWorldY = 0;



};

QrWriter::QrWriter()
    : initialized_(false)
{}

QrWriter::~QrWriter()
{}

void QrWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string service_name_;

  plugin_nh.param("service_name", service_name_, std::string("NAME_OF_SERVICE_WANTED"));

  service_client_ = nh_.serviceClient<pandora_nav_msgs::GetRobotPath>(service_name_);

  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff MapWriter plugin %s.", name_.c_str());
}

QrWriter::getGeotiffData():
{
 pandora_nav_msgs::GetRobotPath qr_srv;

    if (!service_client_.call(qr_srv)) {
      ROS_ERROR_NAMED(name_, "Cannot draw Map, service %s failed", service_client_.getService().c_str());
      return;
    }
    
    if(qrx!=0) delete[] qrx;
    if(qry!=0) delete[] qry;
    if(qrType!=0) delete[] qrType;	
    if(qrTime!=0) delete[] qrTime;
    if(qrWorldX!=0) delete[] qrWorldX;
    if(qrWorldY!=0) delete[] qrWorldY;
    
    
    qrSize = dataFusionSrv.response.qrx.size();
    qrx = new int[qrSize];
    qry = new int[qrSize];
    qrWorldX = new float[qrSize];
    qrWorldY = new float[qrSize];
    qrType = new std::string[qrSize];
    qrTime = new time_t[qrSize];

    for(int i=0; i<qrSize; i++){
      qrx[i] = dataFusionSrv.response.qrx[i];
      qry[i] = dataFusionSrv.response.qry[i];
      qrWorldX[i] = dataFusionSrv.response.qrworldx[i];
      qrWorldY[i] = dataFusionSrv.response.qrworldy[i];
      ROS_ERROR("qrWorldX = %f",qrWorldX[i] );
      qrType[i] = dataFusionSrv.response.qrcontent[i];
      qrTime[i] = dataFusionSrv.response.qrtimestamp[i].sec;
    }
void QrWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;
    
    QColor qrsColor;
    qrsColor.setRgb(10,10,240);
    QRgb qrRgb = qrsColor.rgb();
    
    
        //draw qrs
    for(int i=0; i<qrSize; i++){

      QImage qr(40, 40, QImage::Format_ARGB32);
      QPainter victPainter;
      victPainter.begin(&qr);
      victPainter.setCompositionMode(QPainter::CompositionMode_Source);
      victPainter.fillRect(qr.rect(), Qt::transparent);
      victPainter.setCompositionMode(QPainter::CompositionMode_SourceOver);


      victPainter.setPen(qrsColor);
      victPainter.setBrush(qrsColor);

      QPoint circleCenter(20, 20);
      victPainter.drawEllipse(circleCenter, 10, 10);
      QPen penWhite(Qt::white);
      penWhite.setWidth(2);
      victPainter.setPen(penWhite);
      int victId = i+1;
      QString victIdStr = QString::number(victId);
      if(i+1>=10)
        victPainter.drawText(13, 25, victIdStr);
      else
        victPainter.drawText(16, 25, victIdStr);
        victPainter.end();

      QTransform transform90DegTmp;
      transform90DegTmp.rotate(90);
      QTransform transform90Deg = qr.trueMatrix(transform90DegTmp, 40, 40);

      qr = qr.transformed(transform90Deg);

      //flags indicating occupancy of a particular area by other qrs or hazmats
      int flag0 = 0;
      int flag1 = 0;
      int exitloop = 0;

      QPen blackPen(Qt::black);


      //check area "0"
      for(int k=qrx[i]-14.14-15; k<qrx[i]-14.14+15; k++){
        for(int l=ysize-1-qry[i]-14.14-15; l<ysize-1-qry[i]-14.14+15; l++){
          if( (mapIm.pixel(k,l) == victRgb) || (mapIm.pixel(k,l) == hazRgb) || (mapIm.pixel(k,l) == qrRgb)){
            flag0=1;
            exitloop = 1;
            break;
          }
        }
       if(exitloop == 1){
         exitloop = 0;
         break;
        }
      }
      if(!flag0){
        QPoint vicP(qrx[i]-14.14, ysize-1-qry[i]-14.14);
        mapPainter.begin(&mapIm);
        mapPainter.drawImage(vicP,qr); 
        mapPainter.end();
     }
     else {
       for(int k=qrx[i]-14.14-45; k<qrx[i]-14.14-15; k++){
         for(int l=ysize-1-qry[i]-14.14-15; l<ysize-1-qry[i]-14.14+15; l++){
           if( (mapIm.pixel(k,l) == victRgb) 
               || (mapIm.pixel(k,l) == hazRgb) 
               || (mapIm.pixel(k,l) == qrRgb)){
                 flag1=1;
                 exitloop = 1;
                 break;
               }
          }
         if(exitloop == 1){
           exitloop = 0;
           break;
         }
       }
     if(!flag1){

       QPoint vicPnorm(qrx[i]-14.14-30, ysize-1-qry[i]-14.14);
       QPoint vicP(qrx[i]-20, ysize-1-qry[i]+3);
       mapPainter.begin(&mapIm);
       mapPainter.drawImage(vicPnorm,qr);
       blackPen.setWidth(1);
       mapPainter.setPen(blackPen);
       QPoint smallCircleCenter(qrx[i], ysize-1-qry[i]-4);
       mapPainter.drawEllipse(smallCircleCenter,2,2);
       mapPainter.drawLine(smallCircleCenter, vicP);
       mapPainter.end();
     }
     else{	
        flag1 = 0;	

        QPoint vicPnorm(qrx[i]-14.14+30, ysize-1-qry[i]-14.14);
        QPoint vicP(qrx[i]+28, ysize-1-qry[i]+3);
        mapPainter.begin(&mapIm);
        mapPainter.drawImage(vicPnorm,qr);
        blackPen.setWidth(1);
        mapPainter.setPen(blackPen);
        QPoint smallCircleCenter(qrx[i], ysize-1-qry[i]-4);
        mapPainter.drawEllipse(smallCircleCenter,2,2);
        mapPainter.drawLine(smallCircleCenter, vicP);
        mapPainter.end();
     }
    }
  }

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::QrWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, QrWriter, pandora_geotiff_plugins::QrWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
