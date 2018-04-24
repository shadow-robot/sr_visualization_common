#!/usr/bin/env python
#
# Copyright 2018 Shadow Robot Company Ltd.

import os
import rospkg
import rospy
import sys
from sr_gui_dynamic_plot_tool.dynamic_plot_tool import CreatePlotConfigurations
from collections import namedtuple
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
import threading
import subprocess

TopicStruct = namedtuple('TopicStructure', "topic_name topic_field msg_type time_receipt")


class SrAddInterfaceEntries():
    def __init__(self):
        self._widget_choice = {}
        self._joint_state_msg = JointState()
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self._joint_state_callback)
        self.xml_cfg_name = "hand_e_configuration.xml"

    def define_interface_setting(self):
        """
        Add here the entries for your interface.
        Format - append name of item and number of items.
        By default buttons for hand, fingers and joints will be added
        """
        self._widget_choice['Configuration'] = ["Position_Control"]
        return self._widget_choice

    def define_plot_settings(self, choices):
        """
        Add here your plot settings
        @param choices - list containing user selection from the GUI
        """
        hand_choice = choices["Hand"]
        joint_choice = choices["Joint"]
        configuration_choice = choices["Configuration"]

        full_choice_argument = hand_choice + "_" + joint_choice
        if full_choice_argument[4:] == "FJ0":
            joint_state_choice_argument = full_choice_argument[:6]+"2"
        else:
            joint_state_choice_argument = full_choice_argument
        joint_state_selection = self._get_joint_state_topic(joint_state_choice_argument)

        controller_type = self._check_loaded_controllers()
        # Position Control Topic
        if controller_type == "trajectory":
            position_control_topic = self.create_trajectory_control_topic(hand_choice, joint_state_selection,
                                                                          time_receipt=False)
            position_control_time_receipt = self.create_trajectory_control_topic(hand_choice, joint_state_selection,
                                                                                 time_receipt=True)
        elif controller_type == "position":
            position_control_topic = self.create_position_control_topic(full_choice_argument,
                                                                        time_receipt=False)
            position_control_time_receipt = self.create_position_control_topic(full_choice_argument,
                                                                               time_receipt=True)
        else:
            rospy.logerr("No position or trajectory controller loaded")

        # Joint State topic
        joint_state_position_topic = self.create_joint_state_topic(joint_state_selection, "position",
                                                                   time_receipt=False)
        joint_state_time_receipt = self.create_joint_state_topic(joint_state_selection, "position",
                                                                 time_receipt=True)

        # Create configuration xml file.
        plots = CreatePlotConfigurations(1, 1, self.xml_cfg_name)
        plots_list = plots._plots

        # Add topics to plot
        if configuration_choice == "Position_Control":
            plots_list[0].set_title_and_frame_rate("{}_{}".format(hand_choice, joint_choice), 30)
            try:
                plots_list[0].add_curve(position_control_time_receipt, position_control_topic, 0, "Commanded Position")
                plots_list[0].add_curve(joint_state_time_receipt, joint_state_position_topic, 1, "Measured Position")
            except:
                rospy.logerr("Could not create position control configuration, are controller loaded?")
        else:
            rospy.logerr("No configuration selected")

        t = threading.Thread(target=self._start_rqt)
        t.start()

    def _start_rqt(self):
        package_dir = rospkg.RosPack().get_path("sr_gui_dynamic_plot_tool")
        xml_dir = package_dir + "/xml_configurations"

        subprocess.call("rosrun rqt_multiplot rqt_multiplot --multiplot-config {}/{}".format(xml_dir,
                        self.xml_cfg_name), shell=True)

    def _get_joint_state_topic(self, selected_joint_name):
        for index, name in enumerate(self._joint_state_msg.name):
            if name == selected_joint_name:
                return index

    def _joint_state_callback(self, data):
        self._joint_state_msg = data

    def _check_loaded_controllers(self):
        list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers', ListControllers)
        try:
            resp1 = list_controllers()
            for controller in resp1.controller:
                if controller.type == "sr_mechanism_controllers/SrhJointPositionController":
                    return "position"
                elif controller.type == "position_controllers/JointTrajectoryController":
                    return "trajectory"
                else:
                    rospy.logwarn("No position or trajectory controller loaded")
        except rospy.ServiceException:
            rospy.logerr("Could not get any controller")

    def create_trajectory_control_topic(self, hand_choice, joint_selected, time_receipt):
        TrajCtrlTopic = TopicStruct(topic_name="/{}_trajectory_controller/state".format(hand_choice),
                                    topic_field="desired/positions/{}".format(joint_selected),
                                    msg_type="control_msgs/JointTrajectoryControllerState",
                                    time_receipt=time_receipt)
        return TrajCtrlTopic

    def create_position_control_topic(self, joint_selection, time_receipt):
        joint_selection = joint_selection.lower()
        PosCtrlTopic = TopicStruct(topic_name="/sh_{}_position_controller/command".format(joint_selection),
                                   topic_field="data",
                                   msg_type="std_msgs/Float64",
                                   time_receipt=time_receipt)
        return PosCtrlTopic

    def create_joint_state_topic(self, joint_selected, topic_field, time_receipt):
        JointPositionTopic = TopicStruct(topic_name="/joint_states",
                                         topic_field="{}/{}".format(topic_field, joint_selected),
                                         msg_type="sensor_msgs/JointState",
                                         time_receipt=time_receipt)
        return JointPositionTopic
