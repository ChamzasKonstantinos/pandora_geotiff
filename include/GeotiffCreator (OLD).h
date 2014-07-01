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


#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "navigation_communications/NavigationGeotiffSrv.h"
#include "data_fusion_communications/DatafusionGeotiffSrv.h"
#include <QtGui>
#include <QAction>


/**
* \brief Geotiff creator
*
* creates a geotiff-formatted image conforming to the rules and the standards
* of the Robocup - Rescue Robot League Competition.
*
* \author Dimitrios Vitsios
* \author Electical and Computer Engineer
* \author Department of Electrical and Computer Engineering
* \author Aristotle University of Thessaloniki, Greece
*
*/
class GeotiffCreator : public QWidget
{

    Q_OBJECT

private:
    QMutex _geoGuard;

    void generateQrCsv();

    ros::NodeHandle _handle;

    ros::ServiceClient _navigationGeotiffServiceClient;
    ros::ServiceClient _dataFusionGeotiffServiceClient;

    ros::Timer _geotiffTimer;

    navigation_communications::NavigationGeotiffSrv navigationSrv;
    data_fusion_communications::DatafusionGeotiffSrv dataFusionSrv;

    QString missionName;

    uchar *map;
    uchar *coverage;
    int xsize;
    int ysize;
    int *pathx;
    int *pathy;
    int *victimsx;
    int *victimsy;
    int *hazmatx;
    int *hazmaty;
    int *hazmatType;
    int *qrx;
    int *qry;

    float *qrWorldX;
    float *qrWorldY;

    time_t *qrTime;
    std::string *qrType;

    int pathSize;
    int victimsSize;
    int hazmatSize;
    int qrSize;

    bool gotData;


    void getGeotiffData();

    void geotiffTimerCb(const ros::TimerEvent& event);

public:
    ///GeotiffCreator constructor
    GeotiffCreator();

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
    void saveMissionName(const QString & textInserted);
};


#endif
