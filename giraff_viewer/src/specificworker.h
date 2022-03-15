/*
 *    Copyright (C) 2022 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>

#include <QGraphicsPolygonItem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <grid2d/grid.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include "HumanCameraBody.h"
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>

#define MAX_SPEED 300

class SpecificWorker : public GenericWorker
{
Q_OBJECT
    struct Target
    {
        bool active = false;
        QPointF dest;
    };

public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
    void compute();
    int startup_check();
    void initialize(int period);
    void new_target_slot(QPointF);

private:
    bool startup_check_flag;
    AbstractGraphicViewer *viewer;

    //robot
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsEllipseItem *laser_in_robot_polygon;
    QPointF last_point;
    std::vector<QGraphicsLineItem *> lines;

    map<string, list<int>> mapaArticulaciones;


    void draw_laser(const RoboCompLaser::TLaserData &ldata);

    void setRobotSpeed(float speed, float rot);

    // grid
    int TILE_SIZE = 100;
    QRectF dimensions;
    Grid grid;

    //target
    int state;
    Target target;
    float dist;
    float beta;
    void world_to_robot(Eigen::Vector2f robot_eigen, Eigen::Vector2f target_eigen, RoboCompFullPoseEstimation::FullPoseEuler bState);

    void draw_skeletons(cv::Mat &image, const RoboCompHumanCameraBody::PeopleData &people_data);
    std::pair<float, float> calcularTarget(cv::Mat &image, const RoboCompHumanCameraBody::PeopleData &people_data);
    void cameraSetUp(const RoboCompHumanCameraBody::PeopleData &people_data);
    std::pair<float, float> printPoint(const RoboCompHumanCameraBody::PeopleData &people_data);
    float speed_multiplier(float rot, float dist);

};

#endif
