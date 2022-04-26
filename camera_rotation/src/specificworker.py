#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copyy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
import time

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
import cv2
import json
import time
from PySide2.QtGui import QImage, QPixmap


sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

MIN_VELOCITY = 0.05
TOLERANCE = 0.01


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.motor = self.jointmotorsimple_proxy.getMotorState("")
        print(self.motor)

        self.Period = 2000
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        # computeCODE
        # try:
        #   self.differentialrobot_proxy.setSpeedBase(100, 0)
        # except Ice.Exception as e:
        #   traceback.print_exc()
        #   print(e)

        # The API of python-innermodel is not exactly the same as the C++ version
        # self.innermodel.updateTransformValues('head_rot_tilt_pose', 0, 0, 0, 1.3, 0, 0)
        # z = librobocomp_qmat.QVec(3,0)
        # r = self.innermodel.transform('rgbd', z, 'laser')
        # r.printvector('d')
        # print(r[0], r[1], r[2])

        color = self.obtencion_datos()
        image = np.frombuffer(color.image, np.uint8).reshape(color.width, color.height, color.depth)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        self.tracker_camera(color, 50)
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


    def obtencion_datos(self):
        try:
            self.motor = self.jointmotorsimple_proxy.getMotorState("")
        except:
            print("Error")

        try:
            color = self.camerargbdsimple_proxy.getImage("")
        except:
            print("Error2")
        return color

    k1 = 1.1
    k2 = 0.8
    rad_old = 0
    Period_camera = 40

    def tracker_camera(self, color, puntoMedioX):
        error = puntoMedioX - color.width / 2
        goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        error_rads = np.arctan2(self.k1 * error + self.k2 * error, color.focalx)
        goal_rad = self.motor.pos - error_rads

        # Se mueve el sujeto?
        print("camera moving")
        rad_seg = abs(((self.rad_old - goal_rad) / self.Period_camera) * 1000)  # rad/s

        if abs(rad_seg) < MIN_VELOCITY:
            goal.maxSpeed = MIN_VELOCITY
        else:
            goal.maxSpeed = rad_seg

        goal.position = goal_rad
        # mandamos al motor el objetivo
        self.jointmotorsimple_proxy.setPosition("", goal)
        time.sleep(2)




    # #####################################Para pintar en la UI ################################
    # def refesco_ventana(self, color, image):
    #     qt_image = QImage(image, color.height, color.width, QImage.Format_RGB888)
    #     pix = QPixmap.fromImage(qt_image).scaled(self.ui.label_image.width(), self.ui.label_image.height())
    #     self.ui.label_image.setPixmap(pix)
    #     # image = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)
    #
    #     self.ui.lcdNumber_pos.display(self.motor.pos)
    #     self.ui.lcdNumber_speed.display(self.motor.vel)
    #     self.ui.lcdNumber_temp.display(self.motor.temperature)
    #     self.ui.lcdNumber_max_speed.display(self.current_max_speed)
    #     if self.motor.isMoving:
    #         self.ui.radioButton_moving.setChecked(True)
    #     else:
    #         self.ui.radioButton_moving.setChecked(False)

        # self.graph_tick()

    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple_proxy.getAll(...)
    # self.camerargbdsimple_proxy.getDepth(...)
    # self.camerargbdsimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompDifferentialRobot you can call this methods:
    # self.differentialrobot_proxy.correctOdometer(...)
    # self.differentialrobot_proxy.getBasePose(...)
    # self.differentialrobot_proxy.getBaseState(...)
    # self.differentialrobot_proxy.resetOdometer(...)
    # self.differentialrobot_proxy.setOdometer(...)
    # self.differentialrobot_proxy.setOdometerPose(...)
    # self.differentialrobot_proxy.setSpeedBase(...)
    # self.differentialrobot_proxy.stopBase(...)

    ######################
    # From the RoboCompDifferentialRobot you can use this types:
    # RoboCompDifferentialRobot.TMechParams

    ######################
    # From the RoboCompHumanCameraBody you can call this methods:
    # self.humancamerabody_proxy.newPeopleData(...)

    ######################
    # From the RoboCompHumanCameraBody you can use this types:
    # RoboCompHumanCameraBody.TImage
    # RoboCompHumanCameraBody.TGroundTruth
    # RoboCompHumanCameraBody.KeyPoint
    # RoboCompHumanCameraBody.Person
    # RoboCompHumanCameraBody.PeopleData

    ######################
    # From the RoboCompJointMotorSimple you can call this methods:
    # self.jointmotorsimple_proxy.getMotorParams(...)
    # self.jointmotorsimple_proxy.getMotorState(...)
    # self.jointmotorsimple_proxy.setPosition(...)
    # self.jointmotorsimple_proxy.setVelocity(...)
    # self.jointmotorsimple_proxy.setZeroPos(...)

    ######################
    # From the RoboCompJointMotorSimple you can use this types:
    # RoboCompJointMotorSimple.MotorState
    # RoboCompJointMotorSimple.MotorParams
    # RoboCompJointMotorSimple.MotorGoalPosition
    # RoboCompJointMotorSimple.MotorGoalVelocity


