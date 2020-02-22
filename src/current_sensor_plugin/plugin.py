import os
import rospy

import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from . import settings

class CurrentSensorPlugin(Plugin):

    def __init__(self, context):
        super(CurrentSensorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CurrentSensorPlugin')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(settings.RESOURCE_PATH, 'current_sensor.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Watney Mk. 2 Current Sensor Plot')

        # From RQT Tutorial: Show _widget.windowTitle on left-top of each plugin. This is useful
        # when you open multiple plugins at once. Also if you open multiple instances of your plugin at once,
        # these lines add number to make it easy to tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

