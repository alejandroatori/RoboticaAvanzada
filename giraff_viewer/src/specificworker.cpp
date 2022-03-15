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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }
try {
    auto left_x = std::stod(params.at("left_x").value);
    auto top_y = std::stod(params.at("top_y").value);
    auto width = std::stod(params.at("width").value);
    auto height = std::stod(params.at("height").value);
    auto tile = std::stod(params.at("tile").value);
    qInfo() << __FUNCTION__ << " Read parameters: " << left_x << top_y << width << height << tile;
    this->dimensions = QRectF(left_x, top_y, width, height);
    TILE_SIZE = tile;
}
catch(const std::exception &e) { std::cout << "Error reading config params" << std::endl; }

    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;

    dimensions = QRectF(-5000, -2500, 10000, 5000);
    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
    this->resize(900,450);
    auto [rp, lp] = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH);
    laser_in_robot_polygon = lp;
    robot_polygon = rp;
    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        setRobotSpeed(0,0);
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);


//    ifstream human_body_parts_file("/home/alumno/RoboticaAvanzada/giraff_viewer/src/human_pose.json");
//    Json::Reader reader;
//    Json::Value objeto_json;
//
//    reader.parse(human_body_parts_file, objeto_json);
//    const Json::Value& cadena_clave = objeto_json["keypoints"];
//    const Json::Value& cadena_valor = objeto_json["skeleton"];
////    if (human_body_parts_file.is_open()){
////        cout << "Fichero abierto bien" << endl;
////    }
////
////    cout << "Inicio - size " << cadena_clave.size() << endl;
////
//    for (int i=0; i<cadena_valor.size(); i++)
//    {
//        cout << cadena_clave[cadena_valor[i][0].asInt()].asString() << " - " << cadena_clave[cadena_valor[i][1].asInt()].asString() << endl;
//    }
//
//    exit(-1);

    // grid
    //grid.initialize(dimensions, TILE_SIZE, &viewer->scene, false);

    this->Period = period;;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);
}


void SpecificWorker::compute()
{
    auto people = humancamerabody_proxy->newPeopleData();
    std:pair<float, float> objetivo;

    cameraSetUp(people);
    objetivo = printPoint(people);

    Eigen::Vector2f robot_eigen, target_eigen;
    RoboCompFullPoseEstimation::FullPoseEuler bState;

    try
    {

        bState = fullposeestimation_proxy->getFullPoseEuler();
        //differentialrobot_proxy->getBaseState(bState);
        //qInfo()  << bState.x << bState.y << bState.rz;


        robot_polygon->setRotation(bState.rz*180/M_PI);
        robot_polygon->setPos(bState.x, bState.y);

        //robot_eigen = Eigen::Vector2f(bState.x, bState.y);

        //speed_lcd->display(fullposeestimation_proxy->)

        pos_x->display(bState.x);
        pos_z->display(bState.y);

    }
    catch(const Ice::Exception &e){ std::cout << e.what() << " POSE ERROR" << std::endl;}

    if(target.active)
    {
        Eigen::Vector2f target_eigen(target.dest.x(), target.dest.y());
        Eigen::Vector2f robot_eigen(bState.x, bState.y);
        world_to_robot(robot_eigen, target_eigen, bState);
        try
        {
            cout << "dist: " << this->dist << "\t" << "beta: " << beta << endl;
            if(dist < 100)
            {
                cout << "reached target" << endl;
                target.active = false;
                setRobotSpeed(0,0);
                return;
            }
            else
            {
                float speed = MAX_SPEED * speed_multiplier(beta, dist);
                this->beta = atan2(target.dest.x(), target.dest.y());
                setRobotSpeed(0, beta*0.6);
                std::cout << "Beta: " << beta << endl;
            }
        }
        catch(const Ice::Exception &e)
        {
            std::cout << e.what() <<std::endl;
        };
    }

}
void SpecificWorker::world_to_robot(Eigen::Vector2f robot_eigen, Eigen::Vector2f target_eigen, RoboCompFullPoseEstimation::FullPoseEuler bState)
{
    Eigen::Matrix2f rot;
    rot << cos(bState.rz), -sin(bState.rz), sin(bState.rz), cos(bState.rz);
    auto tr = rot.transpose() * (target_eigen - robot_eigen);
    this->beta = atan2(tr(0), tr(1));
    this->dist = tr.norm();
}
float SpecificWorker::speed_multiplier(float rot, float dist)
{
    float rot_factor, dist_factor;
    if(rot > 1)
        rot = 1;
    if(rot < -1)
        rot = -1;
    rot_factor = exp(pow(-rot, 2));
    if(dist > 1000)
        dist = 1000;
    dist_factor = dist/1000;
    return (rot_factor * dist_factor);
}
std::pair<float, float> SpecificWorker::printPoint(const RoboCompHumanCameraBody::PeopleData &people_data)
{
    static QGraphicsItem *target_elipse = nullptr;
    if(target_elipse != nullptr)
        viewer->scene.removeItem(target_elipse);

    QColor color("Magenta");
    double calculo = 0;
    double mejor = 10000;
    int idPersona = 1;
    std::pair<float, float> target;

    for( const auto &person : people_data.peoplelist)
    {
        if(person.joints.contains(std::to_string(17)) && !people_data.peoplelist.empty())
        {
            auto cd = person.joints.at("17");
            std::cout << cd.x << "  " << cd.y << std::endl;
            auto x = cd.x * 1000;
            auto y = cd.y * 1000;
            auto target_r =laser_in_robot_polygon->mapToScene(QPointF(x,y));
            target_elipse = viewer->scene.addEllipse(target_r.x()-100, target_r.y()-100, 200, 200, QPen(color, 30), QBrush(color));
            target_elipse->setZValue(3);

            this->target.dest = QPointF(x, y);
            this->target.active = true;


//            calculo = pow((cd.x - cd.xw), 2);
//            calculo = calculo + pow((cd.y - cd.yw), 2);
//            calculo = sqrt(calculo);
//            if (mejor > calculo) {
//                mejor = calculo;
//                idPersona = person.id;
//                target.first = cd.x;
//                target.second  = cd.y;
//            }
        }
        else
        {
            this->target.active = false;
        }
    }

    //people_data.peoplelist[idPersona].joints

//    QPolygonF poly;
//    poly << QPointF(0,0);
//    poly << QPointF (people_data.peoplelist[idPersona].joints.at("17").x, people_data.peoplelist[idPersona].joints.at("17").y);
//    poly.pop_back();
//    viewer->scene.addEllipse()
//    QRectF persona(people_data.peoplelist[idPersona].joints.at("17").x - 50, people_data.peoplelist[idPersona].joints.at("17").y - 50, 100, 100);
//
//    auto lP = viewer->scene.addEllipse(persona, QPen(QColor("DarkBlue"), 30), QBrush("LightBlue"));
//    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));

    return target;
}
void SpecificWorker::cameraSetUp(const RoboCompHumanCameraBody::PeopleData &people_data){
    //Coordenadas x, y
    // camera-ojo
    try
    {
        cv::Mat top_img_uncomp;
        QImage top_qimg;
        auto top_img = camerargbdsimple_proxy->getImage("");

        if(not top_img.image.empty())
        {
            if (!top_img.compressed)
            {
                top_img_uncomp = cv::imdecode(top_img.image, -1);
                cv::cvtColor(top_img_uncomp, top_img_uncomp, cv::COLOR_BGR2RGB);
                draw_skeletons(top_img_uncomp, people_data);
                top_qimg = QImage(top_img_uncomp.data, top_img.width, top_img.height, QImage::Format_RGB888).scaled(
                        bottom_camera_label->width(), bottom_camera_label->height(), Qt::KeepAspectRatioByExpanding);;
            } else {
                top_img_uncomp = cv::Mat(top_img.width, top_img.height, CV_8UC3, const_cast<std::vector<uint8_t>&>(top_img.image).data());
                cv::cvtColor(top_img_uncomp, top_img_uncomp, cv::COLOR_BGR2RGB);
                draw_skeletons(top_img_uncomp, people_data);
                top_qimg = QImage(top_img_uncomp.data, top_img.height, top_img.width, QImage::Format_RGB888).scaled(
                        bottom_camera_label->width(), bottom_camera_label->height(), Qt::KeepAspectRatioByExpanding);;
            }
            auto pix = QPixmap::fromImage(top_qimg);
            bottom_camera_label->setPixmap(pix);
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << "CAMERA EYE ERROR" << std::endl;}

    //camera
//    try
//    {
//        cv::Mat top_img_uncomp2;
//        QImage top_qimg2;
//        auto top_img2 = camerasimple_proxy->getImage();
//        if(not top_img2.image.empty())
//        {
//            if (top_img2.compressed)
//            {
//                top_img_uncomp2 = cv::imdecode(top_img2.image, -1);
//                cv::cvtColor(top_img_uncomp2, top_img_uncomp2, cv::COLOR_BGR2RGB);
//                top_qimg2 = QImage(top_img_uncomp2.data, top_img2.width, top_img2.height, QImage::Format_RGB888).scaled(
//                        top_camera_label->width(), top_camera_label->height(), Qt::KeepAspectRatioByExpanding);
//            } else
//                top_qimg2 = QImage(&top_img2.image[0], top_img2.width, top_img2.height, QImage::Format_RGB888).scaled(
//                        top_camera_label->width(), top_camera_label->height(), Qt::KeepAspectRatioByExpanding);
//            auto pix = QPixmap::fromImage(top_qimg2);
//            top_camera_label->setPixmap(pix);
//        }
//    }
//    catch(const Ice::Exception &e){ std::cout << e.what() << "CAMERA ERROR" << std::endl;}
}

///////////////////////////////////////////////////////////////7777
void SpecificWorker::setRobotSpeed(float speed, float rot)
{
    differentialrobot_proxy->setSpeedBase(speed, rot);
    this->speed->display(speed);
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer->scene.removeItem(laser_polygon);

    QPolygonF poly;
    poly << QPointF(0,0);
    for(auto &&l : ldata)
        poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
    poly.pop_back();

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}
void SpecificWorker::draw_skeletons(cv::Mat &image, const RoboCompHumanCameraBody::PeopleData &people_data) {
    calcularTarget(image, people_data);

    ifstream human_body_parts_file("/home/alumno/RoboticaAvanzada/giraff_viewer/src/human_pose.json");
    Json::Reader reader;
    Json::Value objeto_json;

    reader.parse(human_body_parts_file, objeto_json);
    const Json::Value &cadena_clave = objeto_json["keypoints"];
    const Json::Value &cadena_valor = objeto_json["skeleton"];
    std::pair<int, int> lista_articulaciones[cadena_valor.size()];

    for (int i = 0; i < cadena_valor.size(); i++) {
            lista_articulaciones[i].first = cadena_valor[i][0].asInt();
            lista_articulaciones[i].second = cadena_valor[i][1].asInt();
    }

    for( const auto &person : people_data.peoplelist)
    {
        for(const auto &[name1, name2] :  lista_articulaciones)
        {
            if(person.joints.contains(std::to_string(name1)) and person.joints.contains(std::to_string(name2)))
            {
                //cout << "draw skeletons" <<endl;
                auto joint1 = person.joints.at(std::to_string(name1));
                auto joint2 = person.joints.at(std::to_string(name2));
                cv::line(image, cv::Point(joint1.i, joint1.j), cv::Point(joint2.i, joint2.j), cv::Scalar(0, 255, 0), 2);
            }
        }
    }

//    for (const auto &person: people_data.peoplelist)
//    {
//        for (int i = 0; i < cadena_valor.size(); i++) {
//            lista_articulaciones[i].first = cadena_clave[cadena_valor[i][0].asInt()].asString();
//            lista_articulaciones[i].second = cadena_clave[cadena_valor[i][1].asInt()].asString();
//
//            auto joint1 = person.joints.at(lista_articulaciones[i].first);
//            auto joint2 = person.joints.at(cadena_clave[cadena_valor[i][1].asInt()].asString());
//            cout << joint1.x << " --- " << joint2.x << endl;
//            cv::line(image, cv::Point(joint1.i, joint1.j), cv::Point(joint2.i, joint2.j), cv::Scalar(0, 255, 0), 2);
//        }
//    }
}
std::pair<float, float> SpecificWorker::calcularTarget(cv::Mat &image, const RoboCompHumanCameraBody::PeopleData &people_data)
{
    float suma_x;
    float suma_z;
    float media_x;
    float media_z;
    float suma_i;
    float suma_j;
    float media_i;
    float media_j;

    std::pair<float, float> target;
    int contador = 0;

    const auto &person = people_data.peoplelist[0];
    if (!people_data.peoplelist.empty())
    {
        if (person.joints.contains("1"))
        {
//            cout << "X: " << person.joints.at("1").x << endl
//                 << "Y: " << person.joints.at("1").y << endl
//                 << "Z: " << person.joints.at("1").z << endl;
            // exit(-1);
        }
        for (auto &[k, v]: person.joints)
        {
            auto joint = person.joints.at(k);

            suma_i += joint.i;
            suma_j += joint.j;

            // eje z
            suma_z += joint.z;

            // eje x
            suma_x += joint.x;

            contador++;
        }

        // centro esqueleto
        media_i = suma_i / contador;
        media_j = suma_j / contador;

        media_x = suma_x / contador;
        media_z = suma_z / contador;

        target.first = media_x;
        target.second = media_z;

//        cout << "PUNTO MEDIO: " << target.first << ", " << target.second << endl;

        cv::Point punto_medio((int)media_x, (int)media_z);
        cv::circle(image, punto_medio, 0, (0, 0, 0), 10);
        //
    }

    return target;
}
//////////////////////////////////////////////////////////////////7
void SpecificWorker::new_target_slot(QPointF target)
{
    qInfo() << __FUNCTION__ << " Received new target at " << target;
    qInfo() << target;
    this->target.dest = target;
    this->target.active = true;
}
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}




/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompFullPoseEstimation you can call this methods:
// this->fullposeestimation_proxy->getFullPoseEuler(...)
// this->fullposeestimation_proxy->getFullPoseMatrix(...)
// this->fullposeestimation_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPoseMatrix
// RoboCompFullPoseEstimation::FullPoseEuler

/**************************************/
// From the RoboCompJointMotorSimple you can call this methods:
// this->jointmotorsimple_proxy->getMotorParams(...)
// this->jointmotorsimple_proxy->getMotorState(...)
// this->jointmotorsimple_proxy->setPosition(...)
// this->jointmotorsimple_proxy->setVelocity(...)
// this->jointmotorsimple_proxy->setZeroPos(...)

/**************************************/
// From the RoboCompJointMotorSimple you can use this types:
// RoboCompJointMotorSimple::MotorState
// RoboCompJointMotorSimple::MotorParams
// RoboCompJointMotorSimple::MotorGoalPosition
// RoboCompJointMotorSimple::MotorGoalVelocity

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData


