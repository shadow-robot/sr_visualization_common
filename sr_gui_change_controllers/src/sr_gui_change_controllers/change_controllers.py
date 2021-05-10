#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.
# Disabling E1002 check since it complains about super for no reason - inheriting from QObject
# pylint: disable=E1002

from __future__ import absolute_import
import os
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from QtWidgets import QWidget
import rospy
import rospkg
from sr_robot_msgs.srv import RobotTeachMode, RobotTeachModeRequest, RobotTeachModeResponse
from sr_utilities.hand_finder import HandFinder


class SrGuiChangeControllers(Plugin):

    """
    A rosgui plugin for loading the different controllers
    """

    def __init__(self, context):
        super(SrGuiChangeControllers, self).__init__(context)
        self.setObjectName('SrGuiTeachMode')

        hand_finder = HandFinder()
        hand_e = hand_finder.hand_e_available()
        # hand_h = hand_finder.hand_h_available()
        self.modes_str = ["TRAJECTORY_MODE", "TEACH_MODE", "POSITION_MODE", "GRASP_MODE"]

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_change_controllers'), 'uis', 'SrChangeControllers.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrChangeControllersUI')
        context.add_widget(self._widget)

        self._rh_teach_buttons = []
        self._lh_teach_buttons = []
        self._ra_teach_buttons = []
        self._la_teach_buttons = []

        # rh group
        self._rh_teach_buttons.append(self._widget.radioButton_1)
        self._rh_teach_buttons.append(self._widget.radioButton_2)
        self._rh_teach_buttons.append(self._widget.radioButton_3)
        self._rh_teach_buttons.append(self._widget.radioButton_13)
        self._widget.radioButton_1.toggled.connect(
            self.teach_mode_button_toggled_rh)
        self._widget.radioButton_2.toggled.connect(
            self.teach_mode_button_toggled_rh)
        self._widget.radioButton_3.toggled.connect(
            self.teach_mode_button_toggled_rh)
        self._widget.radioButton_13.toggled.connect(
            self.teach_mode_button_toggled_rh)
        if hand_e:
            self._widget.radioButton_13.hide()

        # lh group
        self._lh_teach_buttons.append(self._widget.radioButton_4)
        self._lh_teach_buttons.append(self._widget.radioButton_5)
        self._lh_teach_buttons.append(self._widget.radioButton_6)
        self._lh_teach_buttons.append(self._widget.radioButton_14)
        self._widget.radioButton_4.toggled.connect(
            self.teach_mode_button_toggled_lh)
        self._widget.radioButton_5.toggled.connect(
            self.teach_mode_button_toggled_lh)
        self._widget.radioButton_6.toggled.connect(
            self.teach_mode_button_toggled_lh)
        self._widget.radioButton_14.toggled.connect(
            self.teach_mode_button_toggled_lh)
        if hand_e:
            self._widget.radioButton_14.hide()

        # ra group
        self._ra_teach_buttons.append(self._widget.radioButton_7)
        self._ra_teach_buttons.append(self._widget.radioButton_8)
        self._ra_teach_buttons.append(self._widget.radioButton_9)
        self._widget.radioButton_7.toggled.connect(
            self.teach_mode_button_toggled_ra)
        self._widget.radioButton_8.toggled.connect(
            self.teach_mode_button_toggled_ra)
        self._widget.radioButton_9.toggled.connect(
            self.teach_mode_button_toggled_ra)

        # la group
        self._la_teach_buttons.append(self._widget.radioButton_10)
        self._la_teach_buttons.append(self._widget.radioButton_11)
        self._la_teach_buttons.append(self._widget.radioButton_12)
        self._widget.radioButton_10.toggled.connect(
            self.teach_mode_button_toggled_la)
        self._widget.radioButton_11.toggled.connect(
            self.teach_mode_button_toggled_la)
        self._widget.radioButton_12.toggled.connect(
            self.teach_mode_button_toggled_la)

    def teach_mode_button_toggled_rh(self, checked):
        self.teach_mode_button_toggled(
            checked, "right_hand", self._rh_teach_buttons)

    def teach_mode_button_toggled_lh(self, checked):
        self.teach_mode_button_toggled(
            checked, "left_hand", self._lh_teach_buttons)

    def teach_mode_button_toggled_ra(self, checked):
        self.teach_mode_button_toggled(
            checked, "right_arm", self._ra_teach_buttons)

    def teach_mode_button_toggled_la(self, checked):
        self.teach_mode_button_toggled(
            checked, "left_arm", self._la_teach_buttons)

    def teach_mode_button_toggled(self, checked, robot, buttons):
        if checked:
            if robot == "right_hand" or robot == "left_hand":
                mode = self._check_hand_mode(robot, buttons)
            elif robot == "right_arm" or robot == "left_arm":
                mode = self._check_arm_mode(robot, buttons)
            else:
                rospy.logerr("Invalid input for robot %s", robot)
                return
            rospy.loginfo("Changing robot {} to mode {}".format(robot, self.modes_str[mode]))
            self.change_teach_mode(mode, robot)

    def _check_arm_mode(self, robot, buttons):
        if buttons[0].isChecked():
            mode = RobotTeachModeRequest.TRAJECTORY_MODE
        elif buttons[1].isChecked():
            mode = RobotTeachModeRequest.TEACH_MODE
        elif buttons[2].isChecked():
            mode = RobotTeachModeRequest.POSITION_MODE
        else:
            rospy.logerr("None of the buttons checked for robot %s", robot)
            return
        return mode

    def _check_hand_mode(self, robot, buttons):
        if buttons[0].isChecked():
            mode = RobotTeachModeRequest.TRAJECTORY_MODE
        elif buttons[1].isChecked():
            mode = RobotTeachModeRequest.TEACH_MODE
        elif buttons[2].isChecked():
            mode = RobotTeachModeRequest.POSITION_MODE
        elif buttons[3].isChecked():
            mode = RobotTeachModeRequest.GRASP_MODE
        else:
            rospy.logerr("None of the buttons checked for robot %s", robot)
            return
        return mode

    @staticmethod
    def change_teach_mode(mode, robot):

        teach_mode_client = rospy.ServiceProxy('/teach_mode', RobotTeachMode)

        req = RobotTeachModeRequest()
        req.teach_mode = mode
        req.robot = robot
        modes_str = ["TRAJECTORY_MODE", "TEACH_MODE", "POSITION_MODE", "GRASP_MODE"]
        try:
            resp = teach_mode_client(req)
            if resp.result == RobotTeachModeResponse.ERROR:
                rospy.logerr(
                    "Failed to change robot {} to mode {}".format(robot, modes_str[mode]))
            else:
                rospy.loginfo(
                    "Changed robot {} to mode {} Result = {}".format(robot, modes_str[mode], resp.result))
        except rospy.ServiceException:
            rospy.logerr("Failed to call service teach_mode")
