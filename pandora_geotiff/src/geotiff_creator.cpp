/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Chamzas Konstantinos <chamzask@gmail.com>
 *********************************************************************/

#include "geotiff_creator.h"
//~ std::string homeFolderString("/home/konstantinos/Desktop/image.jpg");
namespace pandora_geotiff{
  
  GeotiffCreator::GeotiffCreator()
  {

    fake_argc_ = 0;
    //Create a QApplication cause otherwise drawing text will crash
    app = new QApplication(fake_argc_, fake_argv_, false);
  
    geotiffBackgroundIm_ = NULL;
    geotiffMapIm_= new QImage(200,200,QImage::Format_ARGB32);
    geotiffFinalIm_ = NULL;
    missionName_ = QString("TestMission");
    missionNamePrefix_ = QString("/RRL_2015_PANDORA_");
  // This parameters should be moved in a yaml file
    CHECKER_SIZE = 50;
    MAP_OFFSET = 4*CHECKER_SIZE;
    CHECKER_COLOR_LIGHT = "LIGHT_GREY";
    CHECKER_COLOR_DARK =  "DARK_GREY";
    MISSION_NAME_COORDS =  Eigen::Vector2i(CHECKER_SIZE/2 ,CHECKER_SIZE/2);
    MISSION_NAME_COLOR = "DARK_BLUE_F";
    MISSION_NAME_WIDTH = 2;
    MAP_SCALE_COORDS = Eigen::Vector2i(CHECKER_SIZE/2, CHECKER_SIZE*2);
    MAP_SCALE_COLOR = "DARK_BLUE_M";
    MAP_SCALE_WIDTH = 2;
    MAP_ORIENTATION_COORDS =Eigen::Vector2i(CHECKER_SIZE*2, CHECKER_SIZE*2);
    MAP_ORIENTATION_COLOR = "DARK_BLUE_M"; 
    MAP_ORIENTATION_WIDTH = 2 ;
    MAP_ORIENTATION_LENGTH = CHECKER_SIZE;
    
    colorMap["DARK_BLUE_F"]     = QColor(0, 44, 207);
    colorMap["DARK_BLUE_M"]     = QColor(0, 50, 120);
    colorMap["DARK_BLUE_W"]     = QColor(0, 40, 120);
    colorMap["LIGHT_GREY"]      = QColor(226, 226, 227);
    colorMap["DARK_GREY"]       = QColor(237, 237, 238);
    colorMap["BLACK"]           = QColor(190,190,191);
    colorMap["YELLOW"]          = QColor(255, 200, 0);
    colorMap["WHITE_MAX"]       = QColor(255, 255, 255);
    colorMap["WHITE_MIN"]       = QColor(128, 128, 128);
    colorMap["LIGHT_GREEN_MAX"] = QColor(180, 230, 180);
    colorMap["LIGHT_GREEN_MIN"] = QColor(130, 230, 130);
    colorMap["SOLID_RED"]       = QColor(240, 10, 10);
    colorMap["YELLOW"]          = QColor(255, 200, 0);
    colorMap["SOLID_ORANGE"]    = QColor(255, 100, 30);
    colorMap["SOLID BLUE"]      = QColor(10, 10, 240);
    colorMap["MAGENTA"]         = QColor(120, 0, 140);
    
  }
  //THIS FUNCTION MUST ONLY BE CALLED AFTER geotiffMapIm is initialized!
  void GeotiffCreator::createBackgroundIm()
  {
  
    ROS_INFO("Creating BackGroundIm...");
    
    geotiffBackgroundIm_= new QImage(geotiffMapIm_->height()+MAP_OFFSET,geotiffMapIm_->width()+MAP_OFFSET, QImage::Format_RGB32);
    QPainter geotiffPainter;
    geotiffPainter.begin(geotiffBackgroundIm_);
  
    drawCheckers(CHECKER_SIZE, CHECKER_COLOR_LIGHT, CHECKER_COLOR_DARK, &geotiffPainter);
    drawMissionName(MISSION_NAME_COORDS, MISSION_NAME_COLOR, MISSION_NAME_WIDTH, &geotiffPainter);
    drawMapScale(MAP_SCALE_COORDS, MAP_SCALE_COLOR, MAP_SCALE_WIDTH, &geotiffPainter);
    drawMapOrientation(MAP_ORIENTATION_COORDS, MAP_ORIENTATION_COLOR, MAP_ORIENTATION_WIDTH, &geotiffPainter);

    QPoint point(2*CHECKER_SIZE, 2*CHECKER_SIZE); 
    geotiffPainter.drawImage(point, *geotiffMapIm_);
  
    ROS_INFO("BackGroundIm was created succesfully...");
  
    }
  
  void GeotiffCreator::saveGeotiff(std::string homeFolderString)
  {
    ROS_INFO("Saving Geotiff...");
    (*geotiffBackgroundIm_).save(homeFolderString.c_str());
    ROS_INFO("Geotiff... was saved succesfully");
    
  }
  void GeotiffCreator::setMissionName(std::string missionName)
  {
    missionName_= QString(missionName.c_str());
  }
  void GeotiffCreator::drawCheckers (const int& checkerSize,const std::string& colorD,const std::string& colorL, QPainter* geotiffPainter)
  {
    ROS_INFO("Drawing Checkers...");
    QBrush gridBrush(colorMap[colorD]);
    bool dark = false;
  
      //draw (checkerboard) grid
    for (int i = 0; i < geotiffBackgroundIm_->width()/CHECKER_SIZE ; i++) {
      dark = !dark;
      for (int j = 0; j < geotiffBackgroundIm_->height()/CHECKER_SIZE ; j++) {
        dark = !dark;
        if (dark) {
           geotiffPainter->setPen(colorMap[colorD]);
           gridBrush.setColor(colorMap[colorD]);
           }
        else{
          geotiffPainter->setPen(colorMap[colorL]);
          gridBrush.setColor(colorMap[colorL]);
          }
        geotiffPainter->drawRect(i*CHECKER_SIZE, j*CHECKER_SIZE, CHECKER_SIZE, CHECKER_SIZE);
        geotiffPainter->fillRect(i*CHECKER_SIZE, j*CHECKER_SIZE, CHECKER_SIZE, CHECKER_SIZE, gridBrush);
        }
      }
    ROS_INFO("Checkers were drawed Succesfully!");
    }
  
  void GeotiffCreator::drawMissionName(const Eigen::Vector2i& coords,const std::string& color, const int& width, QPainter* geotiffPainter)
  {
    ROS_INFO("Drawing MissionName...");
    QPen Pen = QPen(colorMap[color]);
    Pen.setWidth(width);
    geotiffPainter->setPen(Pen);
    
    QPointF filenamePoint(coords.x() ,coords.y());
    
    QString filenameString(missionNamePrefix_);
    filenameString.append(missionName_);
    filenameString.append(".tiff");

    geotiffPainter->drawText(filenamePoint, filenameString);
    ROS_INFO("MissionName was drawed succesfully");
  }
  
void GeotiffCreator::drawMapScale(const Eigen::Vector2i& coords,const std::string& color, const int& width, QPainter* geotiffPainter)
  {
     ROS_INFO("Drawing MapScale...");
     QPen Pen = QPen(colorMap[color]);
     Pen.setWidth(width);
     geotiffPainter->setPen(Pen);
    // Drawing the main length unit side
     QPoint Point1(coords.x(), coords.y());
     QPoint Point2(coords.x(), coords.y() - CHECKER_SIZE);
  
     geotiffPainter->drawLine(Point1, Point2);
     //Drawing the  length unit up side (1/12 of the checker size)
     QPoint Point1Up(Point1.x() - CHECKER_SIZE/12, Point1.y());
     QPoint Point2Up(Point1.x() + CHECKER_SIZE/12, Point1.y());
     geotiffPainter->drawLine(Point1Up, Point2Up);

     //Drawing the  length unit down side (1/12 of the checker size)
     QPoint Point1Down(Point2.x() - CHECKER_SIZE/12, Point2.y());
     QPoint Point2Down(Point2.x() + CHECKER_SIZE/12, Point2.y());
     geotiffPainter->drawLine(Point1Down, Point2Down);

     //Draw the length unit point 1m 
     QPointF TextPoint(Point1.x() + CHECKER_SIZE*3/50, Point1.y() - (CHECKER_SIZE*3)/5);
     QString Text("1m");
     geotiffPainter->drawText(TextPoint, Text);

     ROS_INFO("MapScale was drawed succesfully!");
   }
  void GeotiffCreator::drawMapOrientation(const Eigen::Vector2i& coords,const std::string& color,
    const int& width, QPainter* geotiffPainter)
  {
     ROS_INFO("Drawing MapOrientation...");
     //Drawing the lines
     QPen Pen = QPen(colorMap[color]);
     Pen.setWidth(width);
     geotiffPainter->setPen(Pen);
     
     QPoint pointY(coords.x() - CHECKER_SIZE, coords.y());
     QPoint pointC(coords.x(),coords.y());
     QPoint pointX(coords.x(),coords.y() - CHECKER_SIZE);
  
     geotiffPainter->drawLine(pointY, pointC);
     geotiffPainter->drawLine(pointC, pointX);
     //Drawing the Y arrow
     QPoint pointYup(pointY.x() + CHECKER_SIZE/10, pointY.y() - CHECKER_SIZE/10);
     QPoint pointYdown(pointY.x() + CHECKER_SIZE/10, pointY.y() + CHECKER_SIZE/10);
     geotiffPainter->drawLine(pointY, pointYup);
     geotiffPainter->drawLine(pointY, pointYdown);

     //Drawing The X arrow
     QPoint pointXup(pointX.x() - CHECKER_SIZE/10, pointX.y() + CHECKER_SIZE/10);
     QPoint pointXdown(pointX.x() + CHECKER_SIZE/10, pointX.y() + CHECKER_SIZE/10);
     geotiffPainter->drawLine(pointX, pointXup);
     geotiffPainter->drawLine(pointX, pointXdown);

     //Drawing Y
     QPoint pointYText(pointY.x() + CHECKER_SIZE/5, pointY.y() - CHECKER_SIZE/6);
     QString YString("y");
     geotiffPainter->drawText(pointYText, YString);



     //Drawing X
     QPoint pointXText(pointX.x() + CHECKER_SIZE/5, pointX.y() + CHECKER_SIZE/6);
     QString XString("X");
     geotiffPainter->drawText(pointXText, XString);

     ROS_INFO("MapOrientation drawed succesfully");
   }
  
  void GeotiffCreator::drawMap(const nav_msgs::OccupancyGrid& map,const std::string& color,const int& grid_space)
  {
    
    int xRsize = int(((map.info.height+CHECKER_SIZE)/(CHECKER_SIZE))*CHECKER_SIZE)+500;
    int yRsize = int(((map.info.width+CHECKER_SIZE)/(CHECKER_SIZE))*CHECKER_SIZE)+500;

    int xsize = (map.info.height);
    int ysize = (map.info.width);


    geotiffMapIm_ = new QImage(xsize,ysize, QImage::Format_ARGB32);
    QPainter* mapPainter = new QPainter();
    mapPainter->begin(geotiffMapIm_);
  
    mapPainter->setCompositionMode(QPainter::CompositionMode_Source);
    mapPainter->fillRect(geotiffMapIm_->rect(), Qt::transparent);
    mapPainter->setCompositionMode(QPainter::CompositionMode_SourceOver);

    QPen Pen = QPen(colorMap[color]);
    mapPainter->setPen(Pen);

    for(int i=0; i<xsize; i++){
      
      for(int j=0; j<ysize; j++){
        
        if(map.data[j*ysize + i] >20 ){

          mapPainter->drawPoint(i,ysize-j-1);
        }
      }
    }
    mapPainter->end();
}
  
  void GeotiffCreator::drawPath( const std::vector<Eigen::Vector2i>& points, const std::string& color, const int& width)
  {

    QPainter* pathPainter= new QPainter();
    pathPainter->begin(geotiffMapIm_);

    QPen Pen = QPen(colorMap[color]);
    Pen.setWidth(width);
    pathPainter->setPen(Pen);
  
  
    for(int i = 0; i < points.size()-1; i++){
      
      pathPainter->drawLine(points[i].x(), points[i].y(), points[i+1].x(), points[i+1].y());
    }

    pathPainter->end();
   
  }
  
  void GeotiffCreator::drawObjectOfInterest(const Eigen::Vector2i& coords, const std::string& color, const std::string& txtcolor,
     const std::string& shape,const std::string& txt , const int& size)
  {
    
    QPainter* objectPainter = new QPainter();
    objectPainter->begin(geotiffMapIm_);

    
    objectPainter->setPen(colorMap[color]);
    objectPainter->setBrush(colorMap[color]);

    QPen Pen(colorMap[txtcolor]);
    QFont font;
    font.setPixelSize(size/2);

    if (shape == "DIAMOND"){


      
      QPoint points[4];
      points[0] = QPoint(coords.x(), coords.y()+size/2);
      points[1] = QPoint(coords.x()+size/2, coords.y());
      points[2] = QPoint(coords.x(), coords.y()-size/2);
      points[3] = QPoint(coords.x()-size/2, coords.y());
      objectPainter->drawPolygon(points, 4);

      objectPainter->setPen(Pen);
      objectPainter->setFont(font);
      objectPainter->drawText(coords.x()-size/2,coords.y()-size/2,size,size,
        Qt::AlignCenter,QString::fromStdString(txt));
    }

    else if( shape =="CIRCLE"){
      
      objectPainter->drawEllipse(QPoint(coords.x(),coords.y()), size/2, size/2);

      objectPainter->setFont(font);
      objectPainter->setPen(Pen);
      objectPainter->drawText(coords.x()-size/2,coords.y()-size/2,size,size,
        Qt::AlignCenter,QString::fromStdString(txt));
    }

    else if (shape =="ARROW")
    {
      QPoint points[4];
      points[0] = QPoint(coords.x()+3*size,coords.y());
      points[1] = QPoint(coords.x()-2*size, coords.y() -size);
      points[2] = QPoint(coords.x()-size, coords.y());
      points[3] = QPoint(coords.x()-2*size ,coords.y()+size);
      objectPainter->drawPolygon(points, 4);
    }

    else
    {

      ROS_INFO("THIS SHAPE is not supported");
    }

    objectPainter->end();

      
    //~ }

    //~ QPen penWhite(Qt::white);
    //~ penWhite.setWidth(2);
    //~ objectPainter.setPen(penWhite);
    //~ objectPainter.drawText(x-4, y+5, QString::fromStdString(txt));
    //~ objectPainter.end();
//~ 
    //~ QPoint ObjectPoint(coords.x(), coords.y());
    //~ mapPainter.drawImage(ObjectPoint, objectIm);
//~ 
    //~ int robotx = points[0].x();
    //~ int roboty = points[0].y();
    //~ QColor initPoseColor;
    //~ initPoseColor.setRgb(255, 200, 0);
    //~ QPen initPosePen(initPoseColor);
    //~ mapPainter.setPen(initPosePen);
    //~ QBrush initPoseBrush(initPoseColor);
    //~ mapPainter.setBrush(initPoseBrush);
    //~ QPoint *initPosePoints = new QPoint[5];
    //~ initPosePoints[0].setX(robotx+15);
    //~ initPosePoints[0].setY(roboty);
    //~ initPosePoints[1].setX(robotx-10);
    //~ initPosePoints[1].setY(roboty-5);
    //~ initPosePoints[2].setX(robotx-5);
    //~ initPosePoints[2].setY(roboty);
    //~ initPosePoints[3].setX(robotx-10);
    //~ initPosePoints[3].setY(roboty+5);
    //~ mapPainter.drawPolygon(initPosePoints, 4);
  //~ 
    //~ QapPainter.setPen(pathPen);
  }
}//namespace pandora_geotiff












