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
from QtGui import QIcon
import rospy
import rospkg
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from sr_robot_msgs.srv import RobotTeachMode, RobotTeachModeRequest, RobotTeachModeResponse
from sr_utilities.hand_finder import HandFinder


class SrGuiChangeControllers(Plugin):

    """
    A rosgui plugin for loading the different controllers
    """
    ICON_DIR = os.path.join(
        rospkg.RosPack().get_path('sr_visualization_icons'), 'icons')
    CONTROLLER_ON_ICON = QIcon(os.path.join(ICON_DIR, 'green.png'))
    CONTROLLER_OFF_ICON = QIcon(os.path.join(ICON_DIR, 'red.png'))

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


        avaliable_groups = self.find_avaliable_groups()
        self._widget.rh_group.setDisabled(True)
        self._widget.lh_group.setDisabled(True)
        self._widget.ra_group.setDisabled(True)
        self._widget.lh_group.setDisabled(True)

        self._rh_teach_buttons = []
        self._lh_teach_buttons = []
        self._ra_teach_buttons = []
        self._la_teach_buttons = []

        if 'rh_' in avaliable_groups:
            rospy.logerr("YES")
            self._widget.rh_group.setDisabled(False)

            self._widget.rh_traj.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.rh_traj.toggled.connect(
                self.teach_mode_button_toggled_rh)
            self._rh_teach_buttons.append(self._widget.rh_traj)
            
            self._widget.rh_pos.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.rh_pos.toggled.connect(
                self.teach_mode_button_toggled_rh)
            self._rh_teach_buttons.append(self._widget.rh_pos)

            self._widget.rh_teach.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.rh_teach.toggled.connect(
                self.teach_mode_button_toggled_rh)
            self._rh_teach_buttons.append(self._widget.rh_teach)
            ## hide teach mode if arm...
            if 'ra_' or 'la_' in avaliable_groups:
                self._widget.rh_teach.hide()

        elif 'lh_' in avaliable_groups:
            self._widget.lh_group.setDisabled(False)

            self._widget.lh_traj.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.lh_traj.toggled.connect(
                self.teach_mode_button_toggled_lh)
            self._lh_teach_buttons.append(self._widget.lh_traj)
            
            self._widget.lh_pos.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.lh_pos.toggled.connect(
                self.teach_mode_button_toggled_lh)
            self._lh_teach_buttons.append(self._widget.lh_pos)

            self._widget.lh_teach.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.lh_teach.toggled.connect(
                self.teach_mode_button_toggled_lh)
            self._lh_teach_buttons.append(self._widget.lh_teach)
            ## hide teach mode if arm...
            if 'ra_' or 'la_' in avaliable_groups:
                self._widget.lh_teach.hide()

        elif 'ra_' in avaliable_groups:
            self._widget.ra_group.setDisabled(False)

            self._widget.ra_traj.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.ra_traj.toggled.connect(
                self.teach_mode_button_toggled_ra)
            self._ra_teach_buttons.append(self._widget.ra_traj)

            self._widget.ra_pos.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.ra_pos.toggled.connect(
                self.teach_mode_button_toggled_ra)
            self._ra_teach_buttons.append(self._widget.ra_pos)

        elif 'la_' in avaliable_groups:
            self._widget.la_group.setDisabled(False)

            self._widget.la_traj.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.la_traj.toggled.connect(
                self.teach_mode_button_toggled_la)
            self._la_teach_buttons.append(self._widget.la_traj)

            self._widget.la_pos.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.la_pos.toggled.connect(
                self.teach_mode_button_toggled_la)
            self._la_teach_buttons.append(self._widget.la_pos)

        self.confirm_current_control()

    def find_avaliable_groups(self):
        joint_states = rospy.wait_for_message('/joint_states', JointState)
        robot_names = ["rh_", "lh_", "ra_", "la_"]
        avaliable_robot_names = []
        for robot_name in robot_names:
         for joint in joint_states.name:
            if joint.startswith(robot_name):
                if robot_name not in avaliable_robot_names:
                    avaliable_robot_names.append(robot_name)
        rospy.logerr("names: " + str(avaliable_robot_names))
        return avaliable_robot_names

    def confirm_current_control(self):
        """
        @return: list of current controllers with associated data
        """
        rospy.logerr("HERE ")
        success = True
        list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers', ListControllers)
        try:
            resp1 = list_controllers()
        except rospy.ServiceException:
            success = False

        if success:
            running_controllers = [c for c in resp1.controller if c.state == "running"]
        else:
            rospy.loginfo(
                "Couldn't get list of controllers from controller_manager/list_controllers service")
            return

        # rospy.logerr("controllers: " + str(running_controllers))
        running_traj_controllers = []
        running_pos_controllers = []
        running_teach_controllers = []
        for controller in running_controllers:
            if "position_controller" in controller.name:
                running_pos_controllers.append(controller.name)
            elif "trajectory_controller" in controller.name:
                running_traj_controllers.append(controller.name)
            elif "effort_controller" in controller.name:
                running_teach_controllers.append(controller.name)

        robot_names = ["rh_", "lh_", "ra_", "la_"]
        for robot_name in robot_names:
            for pos_ctrl in running_pos_controllers:
                if robot_name in pos_ctrl:
                    for traj_ctrl in running_traj_controllers:
                        if robot_name in traj_ctrl:
                            self.update_current_controller_field(0, robot_name)
                        else:
                            self.update_current_controller_field(1, robot_name)
            for teach_ctrl in running_teach_controllers:
                if robot_name in teach_ctrl:
                    self.update_current_controller_field(2, robot_name)

    def update_current_controller_field(self, ctrl_type, robot_name):
        if "rh_" == robot_name:
            self._rh_teach_buttons[ctrl_type].setIcon(self.CONTROLLER_ON_ICON)
        elif "lh_" == robot_name:
            self._lh_teach_buttons[ctrl_type].setIcon(self.CONTROLLER_ON_ICON)
        elif "ra_" == robot_name and ctrl_type is not 2:
            self._ra_teach_buttons[ctrl_type].setIcon(self.CONTROLLER_ON_ICON)
        elif "la_" == robot_name and ctrl_type is not 2:
            self._la_teach_buttons[ctrl_type].setIcon(self.CONTROLLER_ON_ICON)

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
            mode = RobotTeachModeRequest.POSITION_MODE
        else:
            rospy.logerr("None of the buttons checked for robot %s", robot)
            return
        return mode

    def _check_hand_mode(self, robot, buttons):
        rospy.logerr(buttons)
        if buttons[0].isChecked():
            mode = RobotTeachModeRequest.TRAJECTORY_MODE
        elif buttons[1].isChecked():
            mode = RobotTeachModeRequest.POSITION_MODE
        elif buttons[2].isChecked():
            mode = RobotTeachModeRequest.TEACH_MODE
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
