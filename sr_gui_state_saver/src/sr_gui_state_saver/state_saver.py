#!/usr/bin/env python3

# Copyright 2016, 2022 Shadow Robot Company Ltd.
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

from __future__ import absolute_import
import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtWidgets import QWidget, QMessageBox

from moveit_msgs.srv import CheckIfRobotStateExistsInWarehouse as HasState
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from sr_robot_commander.sr_robot_state_saver import SrStateSaverUnsafe


class SrGuiStateSaver(Plugin):

    """
    A gui plugin for resetting motors on the shadow hand.
    """

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('SrGuiStateSaver')

        self._publisher = None
        self._widget = QWidget()

        ui_file = os.path.join(
            rospkg.RosPack().get_path('sr_gui_state_saver'), 'uis',
            'SrGuiStateSaver.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrStateSaverUi')
        context.add_widget(self._widget)

        self._has_state = rospy.ServiceProxy("/has_robot_state", HasState)
        self._get_state = rospy.ServiceProxy('/get_robot_state', GetState)

        self._widget.information_box.clicked.connect(self.display_information)
        self._widget.button_save.clicked.connect(self._button_pressed)

    def display_information(self, message):
        message = "To save a state you must first be connected to the warehouse. " + \
                  "After launching the hand, click the green Connect button in the ‘Context’ tab " + \
                  "in the 'Motion Planning' tab of RViz.\n" + \
                  "Next, go to the ‘Stored States’ tab in ‘Motion Planning’. " + \
                  "Here you have full control over the saved states in the warehouse."
        msg = QMessageBox()
        msg.setWindowTitle("Information")
        msg.setIcon(QMessageBox().Information)
        msg.setText(message)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def _button_pressed(self):
        name = self._widget.edit_name.text()

        if name == "":
            QMessageBox.warning(self._widget, "No name entered!", "You must enter a name to save the state.")
            return

        which = ""
        if self._widget.radio_hand.isChecked():
            which = "hand"
        elif self._widget.radio_arm.isChecked():
            which = "arm"
        elif self._widget.radio_both.isChecked():
            which = "both"
        else:
            QMessageBox.warning(self._widget, "Choose what to save!",
                                "You must choose which Robot group you are saving for.")
            return

        side = ""
        if self._widget.radio_right.isChecked():
            side = "right"
        elif self._widget.radio_left.isChecked():
            side = "left"
        elif self._widget.radio_bimanual.isChecked():
            side = "bimanual"
        else:
            QMessageBox.warning(self._widget, "Choose what side to save!",
                                "You must choose the side of the Robot you are saving for.")
            return

        try:
            SrStateSaverUnsafe(name, which, side)
            if self._has_state(name, '').exists:
                state = self._get_state(name, '').state.joint_state
                rounded_positions = [round(position, 5) for position in state.position]
                joints = dict(zip(state.name, rounded_positions))
                QMessageBox.information(self._widget, "State Save Successful!",
                                        "State '{}' saved for {} {}.\n\n".format(name, side, which) +
                                        "Joint names and angles: {}\n".format(joints))
        except Exception as error:
            QMessageBox.warning(self._widget, "Could not save for %s." % which,
                                "State saver failed: " + str(error))
            return
