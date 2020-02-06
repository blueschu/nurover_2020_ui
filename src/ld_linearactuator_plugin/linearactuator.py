import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

from std_msgs import msg

from . import settings, collection_sites


class MyPlugin(Plugin):
    reset_button_signal = QtCore.pyqtSignal(bool)

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(settings.RESOURCE_PATH, 'ld_linearactuator.ui')
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

        self.actuator_button_pub = rospy.Publisher(
            settings.ROS_TOPICS.actuator_position.topic,
            settings.ROS_TOPICS.actuator_position.type,
        )
        self.collection_sites = [
            collection_sites.CollectionSite(name_index, actuator_position)
            for name_index, actuator_position in enumerate(settings.COLLECTION_SITE_POSITIONS, start=1)
        ]
        self.active_collection_site = self.collection_sites[0]


        # for i, attr in enumerate(generate_actuator_button_names()):
        #     getattr(self._widget, attr).clicked[bool].connect(self.on_actuator_button_click(i))

        # self._widget.actuator_slider.setRange(0, len(ACTUATOR_BUTTON_VALUES) - 1)
        # self._widget.actuator_slider.valueChanged.connect(self.on_slider_change)

        self.register_signals()

    def register_signals(self):

        # Register signals for collection site buttons
        for site in self.collection_sites:
            getattr(self._widget, site.button_name).clicked[bool].connect(
                self.on_collection_site_select(site)
            )


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

    def on_collection_site_select(self, collection_site):

        def _on_click():
            self.active_collection_site = collection_site
            m = settings.ROS_TOPICS.actuator_position.type(collection_site.actuator_position)
            self.actuator_button_pub.publish(m)

            self.refresh_collection_site_pixmaps()

            # self._widget.actuator_slider.setSliderPosition(len(ACTUATOR_BUTTON_VALUES) - button_index - 1)

        return _on_click

    def refresh_collection_site_pixmaps(self):
        for site in self.collection_sites:
            active = (site == self.active_collection_site)
            getattr(self._widget, site.label_name).setPixmap(site.pixmap(active))

    # def on_slider_change(self, position):
    #     m = msg.UInt8(ACTUATOR_BUTTON_VALUES[position])
    #     # self.actuator_button_pub.publish(m)

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog

# def generate_actuator_button_names():
#     return [("%s_%i" % (ACTUATOR_BUTTON_BASE, i)) for i in range(1, len(ACTUATOR_BUTTON_VALUES) + 1)]
