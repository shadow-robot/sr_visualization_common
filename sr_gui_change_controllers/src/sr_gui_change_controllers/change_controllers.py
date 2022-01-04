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
from QtWidgets import QWidget, QMessageBox
from QtGui import QIcon
import rospy
import rospkg
from controller_manager_msgs.srv import (ListControllers, SwitchController,
                                         SwitchControllerRequest)
from sensor_msgs.msg import JointState
from sr_robot_msgs.srv import (RobotTeachMode, RobotTeachModeRequest,
                               RobotTeachModeResponse, ChangeControlType)
from sr_robot_msgs.msg import ControlType
from sr_utilities.hand_finder import HandFinder


class SrGuiChangeControllers(Plugin):

    """
    A rosgui plugin for loading the different controllers
    """
    ICON_DIR = os.path.join(
        rospkg.RosPack().get_path('sr_visualization_common_icons'), 'icons')
    CONTROLLER_ON_ICON = QIcon(os.path.join(ICON_DIR, 'green.png'))
    CONTROLLER_OFF_ICON = QIcon(os.path.join(ICON_DIR, 'red.png'))

    def __init__(self, context):
        super(SrGuiChangeControllers, self).__init__(context)
        self.setObjectName('SrGuiTeachMode')

        self.modes_str = ["TRAJECTORY_MODE", "TEACH_MODE", "POSITION_MODE", "DIRECT_PWM_MODE"]

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_change_controllers'), 'uis', 'SrChangeControllers.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrChangeControllersUI')
        context.add_widget(self._widget)

        self.robot_names = ["rh_", "lh_", "ra_", "la_"]
        self._controller_groups = self.find_active_control_groups()

        self._rh_control_buttons = []
        self._lh_control_buttons = []
        self._ra_control_buttons = []
        self._la_control_buttons = []

        if 'rh_' not in self._controller_groups:
            self._widget.rh_group.setDisabled(True)
        else:
            self._widget.rh_traj.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.rh_traj.toggled.connect(
                self.teach_mode_button_toggled_rh)
            self._rh_control_buttons.append(self._widget.rh_traj)

            self._widget.rh_pos.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.rh_pos.toggled.connect(
                self.teach_mode_button_toggled_rh)
            self._rh_control_buttons.append(self._widget.rh_pos)

            self._widget.rh_teach.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.rh_teach.toggled.connect(
                self.teach_mode_button_toggled_rh)
            self._rh_control_buttons.append(self._widget.rh_teach)

            self._widget.rh_pwm.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.rh_pwm.toggled.connect(
                self.teach_mode_button_toggled_rh)
            self._rh_control_buttons.append(self._widget.rh_pwm)

            self._widget.rh_stop.clicked.connect(
                lambda: self.stop_all_running_controllers("rh_", "right hand"))

            # hide teach mode if arm...
            if 'ra_' in self._controller_groups:
                self._widget.rh_teach.hide()
                self._widget.rh_pwm.hide()

        if 'lh_' not in self._controller_groups:
            self._widget.lh_group.setDisabled(True)
        else:
            self._widget.lh_traj.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.lh_traj.toggled.connect(
                self.teach_mode_button_toggled_lh)
            self._lh_control_buttons.append(self._widget.lh_traj)

            self._widget.lh_pos.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.lh_pos.toggled.connect(
                self.teach_mode_button_toggled_lh)
            self._lh_control_buttons.append(self._widget.lh_pos)

            self._widget.lh_teach.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.lh_teach.toggled.connect(
                self.teach_mode_button_toggled_lh)
            self._lh_control_buttons.append(self._widget.lh_teach)

            self._widget.lh_pwm.setIcon(self.CONTROLLER_OFF_ICON)
            self._widget.lh_pwm.toggled.connect(
                self.teach_mode_button_toggled_lh)
            self._lh_control_buttons.append(self._widget.lh_pwm)

            self._widget.lh_stop.clicked.connect(
                lambda: self.stop_all_running_controllers("lh_", "left hand"))

            # hide teach mode if arm...
            if 'la_' in self._controller_groups:
                self._widget.lh_teach.hide()
                self._widget.lh_pwm.hide()

        # TODO: add buttons when more controller modes are available for arms.
        self._widget.ra_group.hide()
        self._widget.la_group.hide()

        self._widget.information_box.clicked.connect(self.display_information)
        self.confirm_current_control()

    def find_active_control_groups(self):
        joint_states = rospy.wait_for_message('/joint_states', JointState)
        _robot_names = []
        for robot_name in self.robot_names:
            for joint in joint_states.name:
                if joint.startswith(robot_name):
                    if robot_name not in _robot_names:
                        _robot_names.append(robot_name)
        return _robot_names

    def get_running_controllers(self):
        list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers', ListControllers)
        try:
            resp1 = list_controllers()
        except rospy.ServiceException as err:
            error = "Couldn't get list of controllers from controller_manager/list_controllers service"
            QMessageBox.warning(self._widget, "No Controllers Found", error)
            rospy.logerr(error + ". Error: " + err)
            return

        # Confirm all running controllers for the controls in this GUI
        running_controllers = [c for c in resp1.controller
                               if c.state == "running" and
                               "tactile_sensor" not in c.name]

        return running_controllers

    def confirm_current_control(self):
        running_controllers = self.get_running_controllers()

        current_robot_control = {
            'rh_': None,
            'lh_': None,
            'ra_': None,
            'la_': None
        }
        for robot_name in self.robot_names:
            for controller in running_controllers:
                if robot_name in controller.name:
                    if "position_controller" in controller.name:
                        current_robot_control[robot_name] = 1
                        break
                    elif "trajectory_controller" in controller.name:
                        current_robot_control[robot_name] = 0
                        break
                    elif "effort_controller" in controller.name:
                        if "h_" in robot_name:
                            if all(hand in self._controller_groups for hand in ["rh_", "lh_"]):
                                hand_id = 'sr_bimanual_hands_robot'
                            else:
                                hand_id = 'sr_hand_robot'
                            srv_path = '/{}/{}/change_control_type'.format(hand_id, robot_name[:2])
                            query_control_type = rospy.ServiceProxy(srv_path, ChangeControlType)
                            try:
                                change_type_msg = ChangeControlType()
                                change_type_msg.control_type = ControlType.QUERY
                                resp1 = query_control_type(change_type_msg)

                                if resp1.result.control_type == 0:
                                    current_robot_control[robot_name] = 3
                                elif resp1.result.control_type == 1:
                                    current_robot_control[robot_name] = 2
                            except rospy.ServiceException as err:
                                error = "Couldn't get control type from {} service".format(srv_path)
                                QMessageBox.warning(self._widget, "No Control Type Found", error)
                                rospy.logerr(error + ". Error: " + err)
                                return
                        else:
                            current_robot_control[robot_name] = 2
                        break

        for robot_name, control in current_robot_control.items():
            # if control is not None:
            self.update_current_controller_field(control, robot_name)

    def update_current_controller_field(self, ctrl_type, robot_name):
        if "rh_" == robot_name:
            for button in range(len(self._rh_control_buttons)):
                if button == ctrl_type:
                    self._rh_control_buttons[button].setIcon(self.CONTROLLER_ON_ICON)
                else:
                    self._rh_control_buttons[button].setIcon(self.CONTROLLER_OFF_ICON)
        elif "lh_" == robot_name:
            for button in range(len(self._lh_control_buttons)):
                if button == ctrl_type:
                    self._lh_control_buttons[button].setIcon(self.CONTROLLER_ON_ICON)
                else:
                    self._lh_control_buttons[button].setIcon(self.CONTROLLER_OFF_ICON)
        # TODO: confirm control types when more controller modes are available for arms.
        elif "ra_" == robot_name:
            for button in range(len(self._ra_control_buttons)):
                if button == ctrl_type:
                    self._ra_control_buttons[button].setIcon(self.CONTROLLER_ON_ICON)
                else:
                    self._ra_control_buttons[button].setIcon(self.CONTROLLER_OFF_ICON)
        elif "la_" == robot_name:
            for button in range(len(self._la_control_buttons)):
                if button == ctrl_type:
                    self._la_control_buttons[button].setIcon(self.CONTROLLER_ON_ICON)
                else:
                    self._la_control_buttons[button].setIcon(self.CONTROLLER_OFF_ICON)
        else:
            rospy.logerr("Unknown robot name: {}".format(robot_name))
            return

    def teach_mode_button_toggled_rh(self, checked):
        self.teach_mode_button_toggled(
            checked, "right_hand", self._rh_control_buttons)

    def teach_mode_button_toggled_lh(self, checked):
        self.teach_mode_button_toggled(
            checked, "left_hand", self._lh_control_buttons)

    def teach_mode_button_toggled_ra(self, checked):
        self.teach_mode_button_toggled(
            checked, "right_arm", self._ra_control_buttons)

    def teach_mode_button_toggled_la(self, checked):
        self.teach_mode_button_toggled(
            checked, "left_arm", self._la_control_buttons)

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
            changed_control = self.change_teach_mode(mode, robot)

            if changed_control:
                self.confirm_current_control()
            else:
                error = "Could not change {} control.".format(robot)
                QMessageBox.warning(self._widget, "Control Not Changed!", error)
                rospy.logerr(error)

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
        if buttons[0].isChecked():
            mode = RobotTeachModeRequest.TRAJECTORY_MODE
        elif buttons[1].isChecked():
            mode = RobotTeachModeRequest.POSITION_MODE
        elif buttons[2].isChecked():
            mode = RobotTeachModeRequest.TEACH_MODE
        elif buttons[3].isChecked():
            mode = RobotTeachModeRequest.DIRECT_PWM_MODE
        else:
            rospy.logerr("None of the buttons checked for robot %s", robot)
            return
        return mode

    def display_information(self, message):
        message = "Use this plugin to load one of the " + \
                  "different types of controllers set by default.\n" + \
                  "Simply click on a controller type, " + \
                  "and it will call a service from the controller_manager " + \
                  "to unload the currently running controller if necessary, " + \
                  "and load the one you’ve selected.\n" + \
                  "Trajectory Control: This controller allows the user to define " + \
                  "a joint space trajectory, that is a series of waypoints " + \
                  "consisting of joint positions.\n" + \
                  "Position Control: This uses a PID position controller. " + \
                  "The output of the host side PID controller is sent to the motor " + \
                  "as a PWM demand. No effort controller is used for position control.\n" + \
                  "Teach Mode: No control is implemented on the host. The Effort " + \
                  "demand is sent to the motor which implements it using a 5kHz control loop.\n" + \
                  "Direct PWM Commands: This is used for basic position control, " + \
                  "and is used by default on a new hand. The PWM demand value is sent " + \
                  "straight to the motor, unless there is a safety cutout.\n" + \
                  "NOTE: Please allow some time between control changes!"
        msg = QMessageBox()
        msg.setWindowTitle("Information")
        msg.setIcon(QMessageBox().Information)
        msg.setText(message)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def stop_all_running_controllers(self, robot_prefix, robot):
        running_controllers = self.get_running_controllers()

        switch_controllers = rospy.ServiceProxy(
            'controller_manager/switch_controller', SwitchController)

        controllers_to_start = []
        controllers_to_stop = []
        for controller in running_controllers:
            if robot_prefix in controller.name:
                controllers_to_stop.append(controller.name)

        try:
            resp1 = switch_controllers(
                controllers_to_start, controllers_to_stop, SwitchControllerRequest.BEST_EFFORT, False, 0.0)
        except rospy.ServiceException as err:
            error = "Failed to unload all controllers for {}.".format(robot) + \
                    "from '/controller_manager/unload_controller' service"
            QMessageBox.warning(self._widget, "Couldn't Unload Controllers", error)
            rospy.logerr(error + ". Error: " + err)
            return

        info_msg = "All controllers stopped for {}.".format(robot)
        QMessageBox.information(self._widget, "Stop Controllers", info_msg)
        rospy.loginfo(info_msg)
        self.confirm_current_control()

    @staticmethod
    def change_teach_mode(mode, robot):

        teach_mode_client = rospy.ServiceProxy('/teach_mode', RobotTeachMode)

        req = RobotTeachModeRequest()
        req.teach_mode = mode
        req.robot = robot
        modes_str = ["TRAJECTORY_MODE", "TEACH_MODE", "POSITION_MODE", "DIRECT_PWM_MODE"]
        try:
            resp = teach_mode_client(req)
            if resp.result == RobotTeachModeResponse.ERROR:
                rospy.logerr(
                    "Failed to change robot {} to mode {}".format(robot, modes_str[mode]))
                return False
            else:
                rospy.loginfo(
                    "Changed robot {} to mode {} Result = {}".format(robot, modes_str[mode], resp.result))
                return True
        except rospy.ServiceException as err:
            rospy.logerr("Failed to call service teach_mode: {}".format(err))
            return False
