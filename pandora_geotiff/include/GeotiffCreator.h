/*
* Copyright (C) 2011 by Pandora Robotics Team, Aristotle Univeristy of Thessaloniki, Greece
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#ifndef GeotiffCreator_H
#define GeotiffCreator_H


#include "map_writer_interface.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <QtGui>
#include <QAction>

class GeotiffCreator : public MapWriterInterface
{

    //~ Q_OBJECT

private:


    //~ void generateQrCsv();

    ros::NodeHandle _handle;
    QString missionName;
    void geotiffTimerCb(const ros::TimerEvent& event);
    void drawCheckers ( int xsize ,int ysize , QPainter* geotiffPainter); 
    void drawFileName( int xsize ,int ysize , QPainter* geotiffPainter);
    void drawMapScale( int xsize ,int ysize ,QPainter* geotiffPainter);
    void drawMapOrientation( int xsize ,int ysize , QPainter* geotiffPainter);
    void drawExploredArea(int xsize , int ysize , QPainter* geotiffPainter, QImage* geotiff);
    int finalSizeX ;
    int finalSizeY ;

public:
    ///GeotiffCreator constructor
    GeotiffCreator();
    ///GeotiffCreator destructor
    ~GeotiffCreator() {  };

public Q_SLOTS:

    /** A Qt SLOT-function which is called when the createGeotiffPushButton button is pressed
*
* it calls the necessary services to obtain the map related information, paints the map
* on a qimage and stores it in a file on the disk.
*
*/
    void onCreateGeotiffClicked();

    /** A Qt SLOT-function which is called when the geotiffMissionNamePlainTextEdit's text is changed
*
* it stores the desired mission name on a local variable in order to be used for
* the geotif's filename
*
* \param[in] textInserted the mission name typed by the operator
*/
    //~ void saveMissionName(const QString & textInserted);
};


#endif
