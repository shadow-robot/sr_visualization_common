#!/usr/bin/env python3
#
# Copyright 2012, 2022 Shadow Robot Company Ltd.
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

# pylint: disable=W0621


from __future__ import absolute_import
from builtins import round
import os
from math import radians, degrees
from sr_robot_msgs.msg import sendupdate, joint  # pylint: disable=W0611
import rospy

from python_qt_binding import loadUi

from PyQt5 import QtCore, Qt
from QtWidgets import QFrame
from std_msgs.msg import Float64


class JointController():

    """
    Contains the min and the max and the command and state topics for the joint controller.
    """

    def __init__(self, name, controller_type, controller_state_type,
                 controller_category, subscribe_status_cb_list=None,
                 cmd_publisher=None, traj_target=None):
        self.name = name
        self.controller_type = controller_type
        self.controller_state_type = controller_state_type
        self.controller_category = controller_category
        self.subscribe_status_cb_list = subscribe_status_cb_list
        self.cmd_publisher = cmd_publisher
        self.traj_target = traj_target


class Joint():

    """
    Contains the name, and a controllers list for the joint.
    """

    def __init__(self, name, min_value, max_value, vel, joint_controller):
        self.name = name
        self.controller = joint_controller
        # Here we manipulate the limits to be in degrees (if they have to do with angles)
        # And also use vel to set min and max if the controller for this joint
        # is a velocity controller
        self.min_value = round(degrees(min_value), 1)
        self.max_value = round(degrees(max_value), 1)
        self.vel = round(degrees(vel), 1)
        if self.controller.controller_category == "velocity":
            self.min_value = -self.vel
            self.max_value = self.vel
        elif self.controller.controller_category == "effort":
            # Happily hardcoded value. It is difficult to establish a relation
            # between these units and Newtons. 900 is enough
            self.min_value = -900
            self.max_value = 900


class ExtendedSlider(QFrame):

    """
    This slider displays the current position and the target as well.
    """

    def __init__(self, joint, ui_file, plugin_parent, parent=None):
        QFrame.__init__(self, parent)
        ui_file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), ui_file)
        loadUi(ui_file, self)

        self.state = None

        self.plugin_parent = plugin_parent
        self.joint = joint
        self.current_controller_index = -1
        self.label.setText(joint.name)
        self.current_value = 0

        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider.setTracking(True)
        self.pos_slider_tracking_behaviour = True
        self.is_selected = False
        self.first_update_done = False

        self.current_controller_index = 0
        self.slider.setMinimum(joint.min_value)
        self.slider.setMaximum(joint.max_value)
        self.min_label.setText(str(joint.min_value))
        self.max_label.setText(str(joint.max_value))

        self.selected.stateChanged.connect(self.checkbox_click)
        self.slider.valueChanged.connect(self.change_value)

        self.timer = Qt.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(200)

    def change_value(self, value):
        self.target.setText(f"Tgt: {value:.1f}")
        self.sendupdate(value)

    def sendupdate(self, value):
        raise NotImplementedError("Virtual method, please implement.")

    def update(self):
        raise NotImplementedError("Virtual method, please implement.")

    def refresh(self):
        raise NotImplementedError("Virtual method, please implement.")

    def checkbox_click(self, value):
        self.is_selected = value

    def set_slider_behaviour(self):
        raise NotImplementedError("Virtual method, please implement.")

    def set_new_slider_behaviour(self, tracking):
        if tracking:
            self.pos_slider_tracking_behaviour = True
        else:
            self.pos_slider_tracking_behaviour = False
        self.set_slider_behaviour()


class EtherCATHandSlider(ExtendedSlider):

    """
    Slider for the EtherCAT Hand.
    """

    def __init__(self, joint, ui_file, plugin_parent, parent=None):
        ExtendedSlider.__init__(self, joint, ui_file, plugin_parent, parent)

        self.initialize_controller()

    def initialize_controller(self):
        self.slider.setMinimum(self.joint.min_value)
        self.slider.setMaximum(self.joint.max_value)
        self.min_label.setText(str(self.joint.min_value))
        self.max_label.setText(str(self.joint.max_value))

        self.pub = rospy.Publisher(
            self.joint.controller.name + "/command",
            Float64,
            queue_size=1,
            latch=True
        )
        self.set_slider_behaviour()
        self.state_sub = rospy.Subscriber(
            self.joint.controller.name + "/state", self.joint.controller.controller_state_type, self._state_cb)

    def _state_cb(self, msg):
        self.state = msg

    def sendupdate(self, value):
        if self.joint.controller.controller_category == "position" \
                or self.joint.controller.controller_category == "velocity":
            self.pub.publish(radians(float(value)))
        else:
            self.pub.publish(float(value))

    def update(self):
        try:
            if self.joint.controller.controller_category == "position" \
                    or self.joint.controller.controller_category == "velocity":
                self.current_value = round(
                    degrees(self.state.process_value), 1)
            elif self.joint.controller.controller_category == "effort":
                self.current_value = round(self.state.process_value, 1)
            self.value.setText("Val: " + str(self.current_value))

            if not self.first_update_done:
                if self.joint.controller.controller_category == "position":
                    self.slider.setSliderPosition(self.current_value)
                    self.slider.setValue(self.current_value)
                    self.target.setText(f"Tgt: {self.current_value:.1f}")
                else:
                    self.target.setText("Tgt: 0.0")
                self.first_update_done = True
        except Exception:
            pass

    def refresh(self):
        """
        Refresh the current position of the slider with index = self.current_controller_index
        """
        if self.joint.controller.controller_category == "position":
            self.slider.setSliderPosition(self.current_value)
            self.slider.setValue(self.current_value)
            self.target.setText(f"Tgt: {self.current_value:.1f}")

    def set_slider_behaviour(self):
        """
        Set the behaviour of the slider according to controller type
        """
        if self.joint.controller.controller_category == "position":
            if self.pos_slider_tracking_behaviour:
                self.slider.setTracking(True)
            else:
                self.slider.setTracking(False)
        elif self.joint.controller.controller_category == "velocity":
            self.slider.setTracking(True)
            self.slider.sliderReleased.connect(self.on_slider_released)
        elif self.joint.controller.controller_category == "effort":
            self.slider.setTracking(True)
            self.slider.sliderReleased.connect(self.on_slider_released)

    def on_slider_released(self):
        if self.joint.controller.controller_category == "effort"\
                or self.joint.controller.controller_category == "velocity":
            self.slider.setSliderPosition(0)
            self.change_value(0)


class EtherCATHandTrajectorySlider(ExtendedSlider):

    """
    Slider for one EtherCAT Hand joint, that uses the trajectory controller interface.
    """

    def __init__(self, joint, ui_file, plugin_parent, parent=None):
        ExtendedSlider.__init__(self, joint, ui_file, plugin_parent, parent)

        self.initialize_controller()

    def initialize_controller(self):
        self.slider.setMinimum(self.joint.min_value)
        self.slider.setMaximum(self.joint.max_value)
        self.min_label.setText(str(self.joint.min_value))
        self.max_label.setText(str(self.joint.max_value))

        self.pub = self.joint.controller.cmd_publisher
        self.set_slider_behaviour()

        self.joint.controller.subscribe_status_cb_list.append(self._state_cb)

    def _state_cb(self, msg):
        self.state = msg.actual.positions[
            msg.joint_names.index(self.joint.name)]

    def sendupdate(self, value):
        if self.joint.controller.traj_target.joint_names \
                and self.joint.controller.traj_target.points:
            self.joint.controller.traj_target.points[0].positions[
                self.joint.controller.traj_target.joint_names.index(self.joint.name)] = radians(float(value))
            self.pub.publish(self.joint.controller.traj_target)

    def update(self):
        try:
            self.current_value = round(degrees(self.state), 1)
            self.value.setText("Val: " + str(self.current_value))

            if not self.first_update_done:
                self.slider.setSliderPosition(self.current_value)
                self.slider.setValue(self.current_value)
                self.target.setText(f"Tgt: {self.current_value:.1f}")

                self.first_update_done = True
        except Exception:
            pass

    def refresh(self):
        """
        Refresh the current position of the slider with index = self.current_controller_index
        """
        self.slider.setSliderPosition(self.current_value)
        self.slider.setValue(self.current_value)
        self.target.setText(f"Tgt: {self.current_value:.1f}")

    def set_slider_behaviour(self):
        """
        Set the behaviour of the slider according to controller type
        """
        if self.joint.controller.controller_category == "position_trajectory":
            if self.pos_slider_tracking_behaviour:
                self.slider.setTracking(True)
            else:
                self.slider.setTracking(False)

    def on_slider_released(self):
        pass


class SelectionSlider(QFrame):

    """
    This slider allows the user to move the selected sliders.
    """

    def __init__(self, name, min_value, max_value, ui_file, plugin_parent, parent=None):
        QFrame.__init__(self, parent)
        ui_file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), ui_file)
        loadUi(ui_file, self)

        self.plugin_parent = plugin_parent
        self.label.setText(name)

        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider.setTracking(False)
        self.is_selected = False

        self.slider.setMinimum(min_value)
        self.slider.setMaximum(max_value)
        self.min_label.setText(str(min_value))
        self.max_label.setText(str(max_value))

        self.slider.valueChanged.connect(self.change_value)
        self.selected.stateChanged.connect(self.checkbox_click)

    def change_value(self, value):
        raise NotImplementedError("Virtual method, please implement.")

    def checkbox_click(self, value):
        self.is_selected = value
        for slider in self.plugin_parent.sliders:
            if slider.is_selected != value:
                slider.is_selected = value
                slider.selected.setChecked(value)


class EtherCATSelectionSlider(SelectionSlider):

    """
    This slider allows the user to move the selected sliders for an etherCAT hand.
    """

    def __init__(self, name, min_value, max_value, ui_file, plugin_parent, parent=None):
        SelectionSlider.__init__(self, name, min_value, max_value, ui_file, plugin_parent, parent)
        self.set_slider_behaviour()
        self.current_value = 0

    def set_slider_behaviour(self):
        """
        Set the behaviour of the slider according to controller type
        Currently we set the tracking to true for all the slide types
        If any of the controllers is an effort or velocity controller
        we will activate the slider released signal detection
        And set the slider halfway (50) as that is the position of the 0
        for effort and velocity controllers
        """
        self.slider.setTracking(True)
        for slider in self.plugin_parent.sliders:
            if (slider.joint.controller.controller_category == "effort")\
                    or (slider.joint.controller.controller_category == "velocity"):
                self.slider.sliderReleased.connect(self.on_slider_released)
                self.current_value = 50
                self.slider.setSliderPosition(self.current_value)
                self.target.setText(f"Tgt: {self.current_value:.1f}%")
                break

    def change_value(self, value):
        """
        modify the values from the selected sliders.
        """
        for slider in self.plugin_parent.sliders:
            if slider.is_selected:
                temp_value = ((slider.slider.maximum() - slider.slider.minimum()) * float(
                    value) / 100.0) + slider.slider.minimum()
                slider.slider.setSliderPosition(temp_value)
                slider.change_value(temp_value)

        self.current_value = value
        self.target.setText(f"Tgt: {self.current_value:.1f}")

    def on_slider_released(self):
        for slider in self.plugin_parent.sliders:
            if slider.is_selected:
                if (slider.joint.controller.controller_category == "effort")\
                        or (slider.joint.controller.controller_category == "velocity"):
                    slider.slider.setSliderPosition(0)
                    slider.change_value(0)
        self.current_value = 50
        self.slider.setSliderPosition(self.current_value)
        self.target.setText(f"Tgt: {self.current_value:.1f}%")
