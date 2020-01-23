import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

from std_msgs import msg

JOINT_SLIDER_COUNT = 6

JOINT_SLIDER_BASE = 'jointslider'


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
        ui_file = os.path.join(rospkg.RosPack().get_path('rover_ui'), 'resource', 'arm_control.ui')
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

        self.reset_button_pub = rospy.Publisher("sometopic", msg.String)

        self._widget.reset_button.clicked[bool].connect(self.on_reset_button)
        for i, attr in enumerate(generate_slider_names(JOINT_SLIDER_COUNT), start=1):
            getattr(self._widget, attr).valueChanged.connect(self.on_slider_change(i))

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

    def on_reset_button(self):
        m = msg.String("reset button pushed!")
        self.reset_button_pub.publish(m)

        for attr in generate_slider_names(JOINT_SLIDER_COUNT):
            getattr(self._widget, attr).setSliderPosition(0)


    def on_slider_change(self, slider):
        def _on_slider_change(position):
            m = msg.String("Slider {} at position {}".format(slider, position))
            self.reset_button_pub.publish(m)

        return _on_slider_change

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog


def generate_slider_names(count):
    return [("%s_%i" % (JOINT_SLIDER_BASE, i)) for i in range(1, count + 1)]
