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

import os
import rospkg
import rospy

from xml.etree import ElementTree as ET

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtCore import Qt
import QtWidgets
from QtWidgets import QWidget, QMessageBox
from QtGui import QFont
from sr_utilities.hand_finder import HandFinder

import xml.etree.ElementTree as xmlTool


class SrGuiDynamicPlotTool(Plugin):
    """
    A rosgui plugin to easily decide which hand's information to plot.
    """

    def __init__(self, context):
        super(SrGuiDynamicPlotTool, self).__init__(context)
        self.setObjectName('SrGuiDynamicPlotTool')
        self._widget = QWidget()
        self._script_name = ""
        self._rospack = rospkg.RosPack()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_gui_dynamic_plot_tool'), 'uis', 'SrGuiDynamicPlotTool.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('SrGuiDynamicPlotToolUi')
        context.add_widget(self._widget)

        self.layout = self._widget.layout()

        package_dir = rospkg.RosPack().get_path("sr_gui_dynamic_plot_tool")
        self.script_dir = package_dir + "/src/sr_gui_dynamic_plot_tool/scripts"
        self.list_scripts = []
        for script in os.listdir(self.script_dir):
            if script.endswith('.py'):
                if not script.startswith('__'):
                    self.list_scripts.append(script)
        self._widget.select_script.addItems(self.list_scripts)
        self._widget.run_button.pressed.connect(self.run_script)

    def run_script(self):
        """
        Run script configuration selected
        """
        script_name = self._widget.select_script.currentText()
        module_name = "sr_gui_dynamic_plot_tool.scripts." + script_name[:-3]
        module = __import__(module_name, fromlist=['SrAddInterfaceEntries'])
        user_entry_class = getattr(module, 'SrAddInterfaceEntries')
        self._user_entry_class = user_entry_class()
        self._widget_choices = self._user_entry_class.define_interface_setting()
        if script_name != self._script_name:
            self._script_name = script_name
            self.add_widgets(self._widget_choices)

    def add_widgets(self, widget_choices):
        """
        Dynamically creates the interfaces for selecting plots
        """
        subframe = QtWidgets.QFrame()
        sublayout = QtWidgets.QHBoxLayout()

        # Add widgets based on script selections
        self.plot_selection_interface = AddWidget(self._widget, widget_choices)
        sublayout.setContentsMargins(0, 0, 0, 0)
        sublayout.setAlignment(Qt.AlignCenter)
        sublayout.addWidget(self.plot_selection_interface)

        # Add plot button
        plot_button = QtWidgets.QPushButton()
        plot_button.setText("Plot")
        plot_button.setFixedSize(150, 100)
        plot_button.setFont(QFont("Helvetica", 15))
        sublayout.addWidget(plot_button)

        subframe.setLayout(sublayout)
        self.layout.addWidget(subframe)

        plot_button.pressed.connect(self.plot_user_choices)

    def plot_user_choices(self):
        user_choices = self.plot_selection_interface._user_selections
        self.plot_selection_interface.selection_button_hand.setChecked(False)
        for finger_button in self.plot_selection_interface.selection_button_finger:
            finger_button.setChecked(False)
        for joint_button in self.plot_selection_interface.selection_button_joint:
            joint_button.setChecked(False)
        for configuration_button in self.plot_selection_interface.selection_button_configuration:
            configuration_button.setChecked(False)
        self._user_entry_class.define_plot_settings(user_choices)


class AddWidget(QWidget):
    """
    Create widgets to select hand, finger and joint that you want to plot
    """
    def __init__(self, parent, widget_choices):
        QWidget.__init__(self, parent)
        self._widget_choices = widget_choices
        self.setWindowTitle("Plot selection interface")
        self._hand_finder = HandFinder()
        self._user_selections = {}
        self._user_selections.fromkeys(["Hand", "Finger", "Joint", "Configuration"])
        plot_interface_frame = QtWidgets.QFrame()
        self.plot_interface_layout = QtWidgets.QVBoxLayout()
        self.plot_interface_layout.setAlignment(Qt.AlignTop)
        self.plot_interface_layout.addWidget(plot_interface_frame)

        label_name = QtWidgets.QLabel()
        label_name.setText("Plot selection interface")
        label_name.setAlignment(Qt.AlignTop)
        label_name.setFont(QFont("Helvetica", 14))
        self.plot_interface_layout.addWidget(label_name)

        self.selection_button_hand = QtWidgets.QToolButton()
        self.selection_button_finger = []
        self.selection_button_joint = []
        self.selection_button_configuration = []

        # Use HandFinder to check which hand is connected and create the corresponding buttons
        if self._hand_finder.hand_e_available():
            hand_name, hand_prefix, hand_serial = self._hand_finder.get_hand_e(number=0)
            hand_joints = self._hand_finder.get_hand_joints()
            self._create_hand_widget(hand_prefix, hand_name)
            self._create_joint_widget(hand_name, hand_prefix, hand_joints)
        elif self._hand_finder.hand_h_available():
            hand_name, hand_prefix, hand_serial = self._hand_finder.get_hand_h(number=0)
            hand_parameters = rospy.get_param("/fh_hand")
            self._create_hand_widget(hand_prefix, hand_name)
            self._create_finger_widget(hand_parameters, hand_prefix)
            self._create_joint_widget(hand_name)

        # Create configuration widget
        for key_name in widget_choices:
            self._create_configuration_widget(key_name, widget_choices[key_name])

        self.setLayout(self.plot_interface_layout)
        self.show()

    def _create_hand_widget(self, prefix, name):
        """
        Create hand selection buttons
        @param prefix - string hand prefix e.g. H0, sr
        @param name - string name of the hand found
        """
        sublayout_hand = QtWidgets.QHBoxLayout()
        sublayout_hand.setAlignment(Qt.AlignLeft)
        sublayout_hand.setContentsMargins(0, 0, 0, 0)
        subframe_hand = QtWidgets.QFrame()
        label_name = QtWidgets.QLabel()
        label_name.setText("Select Hand")
        label_name.setAlignment(Qt.AlignTop)
        label_name.setFont(QFont("Helvetica", 12))
        self.plot_interface_layout.addWidget(label_name)

        self.selection_button_hand.setCheckable(True)
        self.selection_button_hand.setFixedSize(60, 40)
        self.selection_button_hand.setText(prefix[:-1])
        self.selection_button_hand.setObjectName(prefix[:-1])

        sublayout_hand.addWidget(self.selection_button_hand)
        subframe_hand.setLayout(sublayout_hand)
        self.plot_interface_layout.addWidget(subframe_hand)
        self.selection_button_hand.released.connect(self._hand_button_released)

    def _create_finger_widget(self, hand_parameters, hand_prefix):
        """
        Create finger selection buttons
        @param hand_parameters - dictionary that cointains hand parameters
        @param hand_prefix - string hand prefix e.g. H0, sr
        """
        sublayout_finger = QtWidgets.QHBoxLayout()
        sublayout_finger.setAlignment(Qt.AlignLeft)
        sublayout_finger.setContentsMargins(0, 0, 0, 0)
        subframe_finger = QtWidgets.QFrame()
        label_name = QtWidgets.QLabel()
        label_name.setText("Select Finger")
        label_name.setAlignment(Qt.AlignTop)
        label_name.setFont(QFont("Helvetica", 12))
        self.plot_interface_layout.addWidget(label_name)

        for i, key in enumerate(hand_parameters[hand_prefix[:-1]].get('fingers')):
            selection_button_finger = QtWidgets.QToolButton()
            selection_button_finger.setFixedSize(60, 30)
            selection_button_finger.setCheckable(True)
            selection_button_finger.setText(key)
            selection_button_finger.setObjectName(key)
            selection_button_finger.released.connect(self._finger_button_released)
            self.selection_button_finger.append(selection_button_finger)
            sublayout_finger.addWidget(selection_button_finger)
        subframe_finger.setLayout(sublayout_finger)
        self.plot_interface_layout.addWidget(subframe_finger)

    def _create_joint_widget(self, hand_name, hand_prefix=None, joints=None):
        """
        Create joint selection buttons
        @param hand_name - string name of the hand found
        @param hand_prefix - string hand prefix e.g. H0, sr
        """
        sublayout_joint = QtWidgets.QHBoxLayout()
        sublayout_joint.setAlignment(Qt.AlignLeft)
        sublayout_joint.setContentsMargins(0, 0, 0, 0)
        subframe_joint = QtWidgets.QFrame()
        label_name = QtWidgets.QLabel()
        label_name.setText("Select Joint")
        label_name.setAlignment(Qt.AlignTop)
        label_name.setFont(QFont("Helvetica", 12))
        self.plot_interface_layout.addWidget(label_name)
        if hand_name == "hand_h":
            number_of_joint = 3
            for joint in range(0, number_of_joint):
                selection_button_joint = QtWidgets.QToolButton()
                selection_button_joint.setFixedSize(60, 30)
                selection_button_joint.setCheckable(True)
                selection_button_joint.setText("J"+str(joint))
                selection_button_joint.setObjectName("J"+str(joint))
                selection_button_joint.released.connect(self._joint_button_released)
                self.selection_button_joint.append(selection_button_joint)
                sublayout_joint.addWidget(selection_button_joint)
            subframe_joint.setLayout(sublayout_joint)
            self.plot_interface_layout.addWidget(subframe_joint)
        else:
            for joint in joints[hand_prefix[:-1]]:
                selection_button_joint = QtWidgets.QToolButton()
                selection_button_joint.setFixedSize(40, 30)
                selection_button_joint.setCheckable(True)
                if joint[4:] == "FJ1":
                    selection_button_joint.setText(str(joint[3:6])+"0")
                    selection_button_joint.setObjectName(str(joint[3:6])+"0")
                    selection_button_joint.released.connect(self._joint_button_released)
                    self.selection_button_joint.append(selection_button_joint)
                    sublayout_joint.addWidget(selection_button_joint)
                elif joint[4:] == "FJ2":
                    pass
                else:
                    selection_button_joint.setText(str(joint[3:]))
                    selection_button_joint.setObjectName(str(joint[3:]))
                    selection_button_joint.released.connect(self._joint_button_released)
                    self.selection_button_joint.append(selection_button_joint)
                    sublayout_joint.addWidget(selection_button_joint)
            subframe_joint.setLayout(sublayout_joint)
            self.plot_interface_layout.addWidget(subframe_joint)

    def _create_configuration_widget(self, name, parameters):
        """
        Create widgets for the entries in the interface settings
        @param parameters - dict with type and entries for the widgets
        """
        sublayout_configuration = QtWidgets.QHBoxLayout()
        sublayout_configuration.setAlignment(Qt.AlignLeft)
        sublayout_configuration.setContentsMargins(0, 0, 0, 0)
        subframe_configuration = QtWidgets.QFrame()
        label_name = QtWidgets.QLabel()
        label_name.setText("Select "+name)
        label_name.setFont(QFont("Helvetica", 12))
        self.plot_interface_layout.addWidget(label_name)
        for field in parameters:
            selection_button_configuration = QtWidgets.QToolButton()
            selection_button_configuration.setFixedSize(150, 40)
            selection_button_configuration.setCheckable(True)
            selection_button_configuration.setText(field)
            selection_button_configuration.setObjectName(field)
            selection_button_configuration.released.connect(self._configuration_button_released)
            self.selection_button_configuration.append(selection_button_configuration)
            sublayout_configuration.addWidget(selection_button_configuration)
        subframe_configuration.setLayout(sublayout_configuration)
        self.plot_interface_layout.addWidget(subframe_configuration)

    def _hand_button_released(self):
        sending_button = self.sender()
        self._user_selections["Hand"] = str(sending_button.objectName())

    def _finger_button_released(self):
        sending_button = self.sender()
        self._user_selections["Finger"] = str(sending_button.objectName())

    def _joint_button_released(self):
        sending_button = self.sender()
        self._user_selections["Joint"] = str(sending_button.objectName())

    def _configuration_button_released(self):
        sending_button = self.sender()
        self._user_selections["Configuration"] = str(sending_button.objectName())


class CreatePlotConfigurations():
    """
    Dinamically create plot configuration
    """
    def __init__(self, rows, columns, configuration_name):
        self._plots = []
        self._plot_rows = []
        self._plot_columns = []
        package_dir = rospkg.RosPack().get_path("sr_gui_dynamic_plot_tool")
        self._xml_config_dir = package_dir + "/xml_configurations"
        self._base_configuration_xml = xmlTool.parse('{}/empty_configuration.xml'.format(self._xml_config_dir))
        self._xml_root = self._base_configuration_xml.getroot()
        self._generate_xml(rows, columns, configuration_name)

    def _generate_xml(self, rows, columns, configuration_name):
        """
        Add right number of plots in rows and columns
        @param rows - int for the number of rows of plot that have to be added to the configuration
        @param columns - int for the number of columns in a row of plots
        @param configuration_name - string with the name of the xml file to generate
        """
        for child in self._xml_root.findall("table"):
            for item in child:
                if item.tag == "plots":
                    for row in range(0, rows):
                        new_row_tag = xmlTool.SubElement(item, "row_{}".format(row))
                        for column in range(0, columns):
                            new_column_tag = xmlTool.SubElement(new_row_tag, "col_{}".format(column))
                            new_axes_tag = xmlTool.SubElement(new_column_tag, "axes")
                            new_axes_bis_tag = xmlTool.SubElement(new_axes_tag, "axes")
                            new_x_axis_tag = xmlTool.SubElement(new_axes_bis_tag, "x_axis")
                            self._set_axis(new_x_axis_tag)
                            new_y_axis_tag = xmlTool.SubElement(new_axes_bis_tag, "y_axis")
                            self._set_axis(new_y_axis_tag)
                            curves_tag = xmlTool.SubElement(new_column_tag, "curves")
                            self._plots.append(Plot(row, column, configuration_name))

        self._base_configuration_xml.write("{}/{}".format(self._xml_config_dir, configuration_name))
        return self._plots

    def _set_axis(self, parent_tag):
        custom_title_tag = xmlTool.SubElement(parent_tag, "custom_title")
        custom_title_tag.text = "Untitled Axis"
        title_type_tag = xmlTool.SubElement(parent_tag, "title_type")
        title_type_tag.text = "0"
        title_visible_tag = xmlTool.SubElement(parent_tag, "title_visible")
        title_visible_tag.text = "true"


class Plot():
    """
    Create Plot Xml
    """
    def __init__(self, row, column, configuration_name):
        self._row = row
        self._column = column
        self._configuration_name = configuration_name
        package_dir = rospkg.RosPack().get_path("sr_gui_dynamic_plot_tool")
        self._xml_config_dir = package_dir + "/xml_configurations"

    def set_title_and_frame_rate(self, plot_title, frame_rate):
        """
        @param plot_title - string indicating the plot title
        @param frame_rate - int frame rate for plotting
        """
        configuration_xml = xmlTool.parse('{}/{}'.format(self._xml_config_dir, self._configuration_name))
        xml_root = configuration_xml.getroot()
        for row in xml_root.iter("row_{}".format(self._row)):
            for col_tag in row.iter("col_{}".format(self._column)):
                legend_tag = xmlTool.SubElement(col_tag, "legend")
                visible_legend_tag = xmlTool.SubElement(legend_tag, "visible")
                visible_legend_tag.text = "true"
                plot_rate_tag = xmlTool.SubElement(col_tag, "plot_rate")
                plot_rate_tag.text = str(frame_rate)
                plot_title_tag = xmlTool.SubElement(col_tag, "title")
                plot_title_tag.text = plot_title
        configuration_xml.write("{}/{}".format(self._xml_config_dir, self._configuration_name))

    def add_curve(self, x_axis_topic, y_axis_topic, curve_number, curve_name):
        """
        Function to add topic to plot to the xml configuration
        @param x_axis_topic - named tuple that contains informations
        (topic_name, field_to_plot, topic_msg_type) of the topic to plot in x_axis
        @param y_axis_topic - named tuple that contains informations
        of the topic to plot in y_axis
        @param curve_number - int number of the curve in the plot
        @param curve_name - string with name of the curve
        """
        configuration_xml = xmlTool.parse('{}/{}'.format(self._xml_config_dir, self._configuration_name))
        xml_root = configuration_xml.getroot()
        for row in xml_root.iter("row_{}".format(self._row)):
            for col in row.find("col_{}".format(self._column)):
                if col.tag == "curves":
                    topic_curve_tag = xmlTool.SubElement(col, "curve_{}".format(curve_number))
                    axes_curve_tag = xmlTool.SubElement(topic_curve_tag, "axes")
                    self._add_axis_topic(axes_curve_tag, "x_axis", x_axis_topic)
                    self._add_axis_topic(axes_curve_tag, "y_axis", y_axis_topic)
                    self._add_color_settings(topic_curve_tag, "#000000")
                    self._add_data_settings(topic_curve_tag, "3")
                    self._add_style_settings(topic_curve_tag)
                    sub_queue_size_tag = xmlTool.SubElement(topic_curve_tag, "subscriber_queue_size")
                    sub_queue_size_tag.text = "100"
                    title_tag = xmlTool.SubElement(topic_curve_tag, "title")
                    title_tag.text = curve_name

        configuration_xml.write("{}/{}".format(self._xml_config_dir, self._configuration_name))

    def _add_axis_topic(self, parent_tag, name_of_axis, topic):
        """
        Generate x or y axis for a given curve
        @param parent_tag - parent xml tag element
        """
        axis_tag = xmlTool.SubElement(parent_tag, name_of_axis)
        field_tag = xmlTool.SubElement(axis_tag, "field")
        field_tag.text = topic.topic_field
        field_type_tag = xmlTool.SubElement(axis_tag, "field_type")
        if topic.time_receipt:
            field_type_tag.text = "1"
        else:
            field_type_tag.text = "0"
        scale_tag = xmlTool.SubElement(axis_tag, "scale")
        abs_max_tag = xmlTool.SubElement(scale_tag, "absolute_maximum")
        abs_max_tag.text = "1000"
        abs_min_tag = xmlTool.SubElement(scale_tag, "absolute_minimum")
        abs_min_tag.text = "0"
        rel_max_tag = xmlTool.SubElement(scale_tag, "relative_maximum")
        rel_max_tag.text = "0"
        rel_min_tag = xmlTool.SubElement(scale_tag, "relative_minimum")
        rel_min_tag.text = "1000"
        type_scale_tag = xmlTool.SubElement(scale_tag, "type")
        type_scale_tag.text = "0"
        topic_tag = xmlTool.SubElement(axis_tag, "topic")
        topic_tag.text = topic.topic_name
        topic_type_tag = xmlTool.SubElement(axis_tag, "type")
        topic_type_tag.text = topic.msg_type

    def _add_color_settings(self, parent_tag, color):
        """
        Add color settings for curve
        """
        color_tag = xmlTool.SubElement(parent_tag, "color")
        custom_color_tag = xmlTool.SubElement(color_tag, "custom_color")
        custom_color_tag.text = color
        type_color_tag = xmlTool.SubElement(color_tag, "type")
        type_color_tag.text = "0"

    def _add_data_settings(self, parent_tag, data_type):
        """
        Add data settings for curve
        """
        data_tag = xmlTool.SubElement(parent_tag, "data")
        circular_buf_capacity_tag = xmlTool.SubElement(data_tag, "circular_buffer_capacity")
        circular_buf_capacity_tag.text = "1000"
        time_frame_length_tag = xmlTool.SubElement(data_tag, "time_frame_length")
        time_frame_length_tag.text = "10"
        type_tag = xmlTool.SubElement(data_tag, "type")
        type_tag.text = data_type

    def _add_style_settings(self, parent_tag):
        """
        Add style settings for curve
        """
        style_tag = xmlTool.SubElement(parent_tag, "style")
        lines_interp_tag = xmlTool.SubElement(style_tag, "lines_interpolate")
        lines_interp_tag.text = "false"
        pen_style_tag = xmlTool.SubElement(style_tag, "pen_style")
        pen_style_tag.text = "1"
        pen_width_tag = xmlTool.SubElement(style_tag, "pen_width")
        pen_width_tag.text = "1"
        render_antialias_tag = xmlTool.SubElement(style_tag, "render_antialias")
        render_antialias_tag.text = "false"
        steps_invert_tag = xmlTool.SubElement(style_tag, "steps_invert")
        steps_invert_tag.text = "false"
        sticks_baseline_tag = xmlTool.SubElement(style_tag, "sticks_baseline")
        sticks_baseline_tag.text = "0"
        sticks_orientation_tag = xmlTool.SubElement(style_tag, "sticks_orientation")
        sticks_orientation_tag.text = "2"
        type_tag = xmlTool.SubElement(style_tag, "type")
        type_tag.text = "0"
