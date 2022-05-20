/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
    auto left_x = std::stod(params.at("left_x").value);
    auto top_y = std::stod(params.at("top_y").value);
    auto width = std::stod(params.at("width").value);
    auto height = std::stod(params.at("height").value);
    auto tile = std::stod(params.at("tile").value);
    qInfo() << __FUNCTION__ << " Read parameters: " << left_x << top_y << width << height << tile;
    this->dimensions = QRectF(left_x, top_y, width, height);
    TILE_SIZE = tile;
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;

    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
    this->resize(900,450);
    auto [rp, lp] = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH);
    laser_in_robot_polygon = lp;
    robot_polygon = rp;
    giro = -1;
    auto motor = jointmotorsimple_proxy->getMotorState("");
    last_motor_pos = motor.pos;

    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);


    // grid
    grid.initialize(dimensions, TILE_SIZE, &viewer->scene, false);

    this->Period = period;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);


    //Archivo json esqueleto
    ifstream human_body_parts_file("/home/alumno/RoboticaAvanzada/src/human_pose.json");
    Json::Reader reader;
    Json::Value objeto_json;

    reader.parse(human_body_parts_file, objeto_json);
    const Json::Value &id_articulaciones = objeto_json["skeleton"];

    vector_articulaciones.resize(id_articulaciones.size());

    for (int i = 0; i < id_articulaciones.size(); i++) {
        vector_articulaciones[i].first = id_articulaciones[i][0].asInt();
        vector_articulaciones[i].second = id_articulaciones[i][1].asInt();
    }

    x_axis_rotation_matrix = Eigen::AngleAxisf (1.3, Eigen::Vector3f::UnitX())
                             * Eigen::AngleAxisf (0.2, Eigen::Vector3f::UnitY());

    primerTarget = false;
}

void SpecificWorker::cameraSetUp (const RoboCompHumanCameraBody::PeopleData &people_data)
{
    // camera ojo
    try
    {
        cv::Mat top_img_uncomp;
        QImage top_qimg;
        auto top_img = camerargbdsimple_proxy->getImage("");

        if(not top_img.image.empty())
        {
            //qInfo() << __FUNCTION__ << "hay imagen";

            if (!top_img.compressed)
            {
                //qInfo() << __FUNCTION__  << "hola3";
                top_img_uncomp = cv::imdecode(top_img.image, -1);
                cv::cvtColor(top_img_uncomp, top_img_uncomp, cv::COLOR_BGR2RGB);
                drawSkeleton(top_img_uncomp, people_data);
                top_qimg = QImage(top_img_uncomp.data, top_img.width, top_img.height, QImage::Format_RGB888).scaled(
                        bottom_camera_label->width(), bottom_camera_label->height(), Qt::KeepAspectRatioByExpanding);
            } else {
                top_img_uncomp = cv::Mat(top_img.width, top_img.height, CV_8UC3, const_cast<std::vector<uint8_t>&>(top_img.image).data());
                cv::cvtColor(top_img_uncomp, top_img_uncomp, cv::COLOR_BGR2RGB);
                drawSkeleton(top_img_uncomp, people_data);
                top_qimg = QImage(&top_img.image[0], top_img.height, top_img.width, QImage::Format_RGB888).scaled(
                        bottom_camera_label->width(), bottom_camera_label->height(), Qt::KeepAspectRatioByExpanding);

            }

            auto pix = QPixmap::fromImage(top_qimg);
            bottom_camera_label->setPixmap(pix);
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << "CAMERA ERROR" << std::endl;}

    //Camara normal

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

void SpecificWorker::compute()
{
    auto people = humancamerabody_proxy->newPeopleData();
    RoboCompFullPoseEstimation::FullPoseEuler bState;
    try
    {
        bState = fullposeestimation_proxy->getFullPoseEuler();
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << " POSE ERROR" << std::endl;}

    cameraSetUp(people);
    posicionRobot(bState);
    int personaElegida = eleccionPersona(people);
    if (personaElegida != -1){
        drawPeopleMap(people, bState, personaElegida);
        rotateCamera(camerargbdsimple_proxy, target.dest.x());
        //moveRobot();
    }
    else{
        //seguirUltimaPosicion();
    }
}

void SpecificWorker::seguirUltimaPosicion(){
    if(target.active){
        float mod = sqrt(pow(target.dest.x(), 2) + pow(target.dest.y(), 2));
        float beta = atan2(target.dest.x(), target.dest.y());
        float reduce_speed_if_turningv = exp(-pow(beta, 2)/0.1);
        float adv = MAX_ADV_VEL * reduce_speed_if_turningv * reduce_speed_if_close_to_target(mod);

        beta = std::clamp(beta, -1.f, 1.f);
        if (fabs(beta) < 0.05){
            beta = 0;
        }

        if (mod < 3000) {
            setRobotSpeed(adv, beta*1.2);
        }
        else{
            if (mod < 1500){
                setRobotSpeed(0, 0);
                target.active = false;
            }
        }
    }
}

int SpecificWorker::eleccionPersona(const RoboCompHumanCameraBody::PeopleData &people){
    int indiceActual = 0;
    int indiceMejorHastaAhora = 0;
    float mejorDistanciaHastaAhora = 3000;
    int personasSinJulia = 0;

    if (!people.peoplelist.empty()) {
        for (const auto &person: people.peoplelist) {
            if (person.joints.contains("17")) {
                auto cd = person.joints.at("17");
                Eigen::Vector2f points = ((x_axis_rotation_matrix * Eigen::Vector3f(cd.x, cd.y, cd.z)) * 1000.f).head(2);
                points.y() = -points.y();  // invert axis
                float dist = sqrt(pow(points.x(), 2) + pow(points.y(), 2));

                if (dist < mejorDistanciaHastaAhora){
                    mejorDistanciaHastaAhora = dist;
                    indiceMejorHastaAhora = indiceActual;
                }
            }
            else{
                personasSinJulia++;
            }
            indiceActual++;
        }
        //std::cout << "num personas sin cuello -> " << personasSinJulia << " -------- indice Actual -> " << indiceActual << std::endl;
        if (personasSinJulia == indiceActual){
            indiceMejorHastaAhora = -1;
        }
        //std::cout << "numero personas -> " << indiceActual << std::endl;
        return indiceMejorHastaAhora;
    }
    else
        return -1;
}
void SpecificWorker::drawPeopleMap (const RoboCompHumanCameraBody::PeopleData &people, RoboCompFullPoseEstimation::FullPoseEuler bState, int personaElegida){
    QColor color("Magenta");
    static QGraphicsItem *target_elipse = nullptr;

    if (target_elipse != nullptr){
        viewer->scene.removeItem(target_elipse);
    }

    //std::cout << "Persona elegida -> " << personaElegida << std::endl;
    const auto &person = people.peoplelist[personaElegida];
    if (!person.joints.contains("17")) {
        std::cout << "contains 17" << std::endl;
    }
    auto cd = person.joints.at("17");

//  std::cout << "x ->" << cd.x << " y -> " << cd.y << " z -> " << cd.z << std::endl;
    Eigen::Vector2f points = ((x_axis_rotation_matrix * Eigen::Vector3f(cd.x, cd.y, cd.z)) * 1000.f).head(2);
    points.y() = -points.y();  // invert axis
    QPointF f = world_to_robot2(points, bState);

    auto target_r = laser_in_robot_polygon->mapToScene(f);
    target_elipse = viewer->scene.addEllipse(f.x(), -f.y(), 200, 200, QPen(color, 30), QBrush(color));
    target_elipse->setZValue(3);

    this->target.dest = QPointF(points.x(), points.y());
    this->target.active = true;
}


void SpecificWorker::posicionRobot (RoboCompFullPoseEstimation::FullPoseEuler bState){
    robot_polygon->setRotation(bState.rz*180/M_PI);
    robot_polygon->setPos(bState.x, bState.y);

    pos_x->display(bState.x);
    pos_y->display(bState.y);
}

QPointF SpecificWorker::world_to_robot(RoboCompFullPoseEstimation::FullPoseEuler state)
{
    float alfa = state.rz;
    Eigen::Vector2f TW(target.dest.x(),target.dest.y()); //target
    Eigen::Vector2f RW(state.x,state.z); //robot

    Eigen::Matrix2f R(2,2);
    R(0,0) = cos(alfa);
    R(0,1) = sin(alfa);
    R(1,0) = -sin(alfa);
    R(1,1) = cos(alfa);

    auto TR = R * (TW-RW);

    return QPointF(TR.x(),TR.y());
}

QPointF SpecificWorker::robot_to_world(RoboCompFullPoseEstimation::FullPoseEuler state, Eigen::Vector2f TW)
{
    float alfa = state.rz;
    Eigen::Vector2f RW(state.x,state.z); //robot

    Eigen::Matrix2f R(2,2);
    R(0,0) = cos(alfa);
    R(0,1) = -sin(alfa);
    R(1,0) = sin(alfa);
    R(1,1) = cos(alfa);

    auto TR = R * TW + RW;

    return QPointF(TR.x(),TR.y());
}

QPointF SpecificWorker::world_to_robot2(Eigen::Vector2f point, RoboCompFullPoseEstimation::FullPoseEuler bState)
{
    float alfa = bState.rz;
    Eigen::Vector2f RW(bState.x, bState.y); //robot

    Eigen::Matrix2f R(2,2);
    R(0,0) = cos(alfa);
    R(0,1) = sin(alfa);
    R(1,0) = -sin(alfa);
    R(1,1) = cos(alfa);

    auto TR = R * (point-RW);

    return QPointF (TR.x(), TR.y());
}
QPointF SpecificWorker::robot_to_world2(Eigen::Vector2f TW, RoboCompFullPoseEstimation::FullPoseEuler bState)
{
    float alfa = bState.rz;
    Eigen::Vector2f RW(bState.x, bState.y); //robot

    Eigen::Matrix2f R(2,2);
    R(0,0) = cos(alfa);
    R(0,1) = sin(alfa);
    R(1,0) = -sin(alfa);
    R(1,1) = cos(alfa);

    auto TR = R * (TW - RW);

    return QPointF(TR.x(),TR.y());
}

void SpecificWorker::drawSkeleton (cv::Mat &image, const RoboCompHumanCameraBody::PeopleData &people_data){
    for( const auto &person : people_data.peoplelist)
    {
        //qInfo() << __FUNCTION__ << " hay personas";
        for(const auto &[name1, name2] :  vector_articulaciones)
        {
            //qInfo() << __FUNCTION__ << " hay articulaciones";
            if(person.joints.contains(std::to_string(name1)) and person.joints.contains(std::to_string(name2)))
            {
                //cout << "draw skeletons" <<endl;
                auto joint1 = person.joints.at(std::to_string(name1));
                auto joint2 = person.joints.at(std::to_string(name2));
                cv::line(image, cv::Point(joint1.i, joint1.j), cv::Point(joint2.i, joint2.j), cv::Scalar(0, 255, 0), 2);

            }
            if (person.joints.contains(std::to_string(17))){
                auto cuello = person.joints.at(std::to_string(17));
                cv::circle(image, cv::Point (cuello.i, cuello.j), 7, cv::Scalar (0, 0, 0), 2);      //cv::FILLE
            }
        }

    }
    cv::line(image, cv::Point(image.cols / 2, 0), cv::Point(image.cols/2, image.rows), cv::Scalar (0, 0, 255), 4);
}

void SpecificWorker::moveRobot(){

    if (target.active) {
        float mod = sqrt(pow(target.dest.x(), 2) + pow(target.dest.y(), 2));
        float beta = atan2(target.dest.x(), target.dest.y());
        float reduce_speed_if_turningv = exp(-pow(beta, 2)/0.1);
        float adv = MAX_ADV_VEL * reduce_speed_if_turningv * reduce_speed_if_close_to_target(mod);

        //qInfo() << __FUNCTION__  << "X -> " << target.dest.x() << "         y -> " << target.dest.y() << "       angulo -> " << beta * 1.1 << "      mod -> "  << mod << endl;

        beta = std::clamp(beta, -1.f, 1.f);
//        beta = fabs(beta)< 0.05 ? 0.f : beta;
        if (fabs(beta) < 0.05){
            beta = 0;
        }

        if (mod < 3000) {
            estaParao = false;
            setRobotSpeed(adv, beta*1.2);
            //qInfo() << __FUNCTION__ << "ADV:" << adv*1.3 << "BETA:" << beta << endl;
        }
        else{
            if (mod < 1500){
                setRobotSpeed(0, 0);
                target.active = false;
            }
        }
    }
    else{
        if (!estaParao){
            setRobotSpeed(0, 0);
            estaParao = true;
        }
    }
}


void SpecificWorker::rotateCamera(RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr camerargbdsimple_proxy, float middleXPoint) {

    auto color = camerargbdsimple_proxy->getImage("");
    auto error = middleXPoint - (color.width/2);
    auto goal = RoboCompJointMotorSimple::MotorGoalPosition();
    auto der_error = -(error );//- this->last_error
    auto error_rads = atan2((k1*error)+(k2*der_error), color.focalx);
    auto motor = jointmotorsimple_proxy->getMotorState("");
    auto goal_rad = motor.pos - error_rads;

    if((this->last_rad > (goal_rad + TOLERANCE)) ||
       (this->last_rad < (goal_rad -TOLERANCE)))
    {
        cout << "Moving cam" << endl;
        auto rad_seg = abs(((last_rad - goal_rad) / this->Period) * 1000);
        if(abs(error_rads) > 0.3)
        {
            cout << motor.pos << endl;

            goal.maxSpeed = 0;
            cout << "usando 0 velocidad" << endl;
        }
        else
        {
            if(abs(rad_seg) < MIN_SPEED)
            {
                cout << "usando minima velocidad" << endl;
                goal.maxSpeed = MIN_SPEED;
            }
            else {
                cout << "usando maxima velocidad" << endl;
                goal.maxSpeed = rad_seg;
            }
        }
        goal.position = goal_rad;

        cout <<"position objetivo: " << goal.position << endl;

        auto motor = jointmotorsimple_proxy->getMotorState("");
        cout << "Antes de moverse: " << motor.pos << endl;

        if(last_motor_pos == motor.pos)
        jointmotorsimple_proxy->setPosition("", goal);

        auto motor2 = jointmotorsimple_proxy->getMotorState("");
        while(abs(motor2.pos - goal.position) > 1.5){
            cout << "Dif: " << abs(motor2.pos - goal.position) << endl;
            motor2 = jointmotorsimple_proxy->getMotorState("");
            cout << "Despues de moverse: " << motor2.pos << endl;
        }


        auto pos_err = motor.pos - last_motor_pos;

        last_motor_pos = motor.pos;
        last_error = error;
        last_rad = goal_rad;

    }
}


/////////////////////////////////////////////////////////////////////////

float SpecificWorker::reduce_speed_if_close_to_target(float mod)
{
    int threshold = 2000;

    if (mod <= threshold)
    {
        if (mod < 1000) {
            return 0;
        }
        else
        {
            return mod/threshold;
        }
    }
    else
    {
        return 1;
    }
}

float SpecificWorker::reduce_speed_if_turning(float beta){
    if (fabs(beta) >= 0.05){
        return 0;
    }
    else{
        return 1;
    }
}

void SpecificWorker::world_to_robot5(Eigen::Vector2f robot_eigen, Eigen::Vector2f target_eigen, RoboCompFullPoseEstimation::FullPoseEuler bState)
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

void SpecificWorker::setRobotSpeed(float speed, float rot)
{
    differentialrobot_proxy->setSpeedBase(speed, rot);
    this->speed->display(speed);
//    std::cout << "velocidad :) -> " << this->speed->value() << std::endl;
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
void SpecificWorker::new_target_slot(QPointF target)
{
    qInfo() << __FUNCTION__ << " Received new target at " << target;
}
void SpecificWorker::new_tilt_value_slot(int value)
{
    try
    {
        float r_value = value * M_PI / 180;
        jointmotorsimple_proxy->setPosition( "tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{r_value, 1 });
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
}
void SpecificWorker::sweep_button_slot(bool checked)
{
    if(not checked)
    {
        grid.set_all_to_not_visited();
    }
}
void SpecificWorker::trace_button_slot(bool checked)
{
    if(not checked)
    {
        for(auto &l : lines)
            viewer->scene.removeItem(l);
        lines.clear();
    }
    else
    {
        try
        {
            RoboCompGenericBase::TBaseState bState;
            differentialrobot_proxy->getBaseState(bState);
            last_point = QPointF(bState.x, bState.z);
        }
        catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    }
}
////////////////////////////////////////////////////////////////////////////
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

/**************************************/
// From the RoboCompHumanCameraBody you can use this types:
// RoboCompHumanCameraBody::TImage
// RoboCompHumanCameraBody::TGroundTruth
// RoboCompHumanCameraBody::KeyPoint
// RoboCompHumanCameraBody::Person
// RoboCompHumanCameraBody::PeopleData



