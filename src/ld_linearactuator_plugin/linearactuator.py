import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

from std_msgs import msg

ACTUATOR_BUTTON_VALUES = [
    0,
    64,
    128,
    192,
    255
]

ACTUATOR_BUTTON_BASE = 'actuator_button'

ACTUATOR_TOPIC = "/life_detection/actuator_position"



class MyPlugin(Plugin):
    reset_button_signal = QtCore.pyqtSignal(bool)

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rover_ui'), 'resource', 'ld_linearactuator.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('GUI Experiment Object Name')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.actuator_button_pub = rospy.Publisher(ACTUATOR_TOPIC, msg.UInt8)

        for i, attr in enumerate(generate_actuator_button_names()):
            getattr(self._widget, attr).clicked[bool].connect(self.on_actuator_button_click(i))

        self._widget.actuator_slider.setRange(0, len(ACTUATOR_BUTTON_VALUES) - 1)
        self._widget.actuator_slider.valueChanged.connect(self.on_slider_change)

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


    def on_actuator_button_click(self, button_index):
        def _on_click():
            m = msg.UInt8(ACTUATOR_BUTTON_VALUES[button_index])
            self.actuator_button_pub.publish(m)
            self._widget.actuator_slider.setSliderPosition(len(ACTUATOR_BUTTON_VALUES) - button_index - 1)

        return _on_click

    def on_slider_change(self, position):
        m = msg.UInt8(ACTUATOR_BUTTON_VALUES[position])
        # self.actuator_button_pub.publish(m)


    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog


def generate_actuator_button_names():
    return [("%s_%i" % (ACTUATOR_BUTTON_BASE, i)) for i in range(1, len(ACTUATOR_BUTTON_VALUES) + 1)]
