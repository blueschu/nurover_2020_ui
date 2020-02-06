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

        # Create publisher instances for the the ROS topics this plugin writes to
        self.actuator_button_pub = rospy.Publisher(
            settings.ROS_TOPICS.actuator_position.topic,
            settings.ROS_TOPICS.actuator_position.type,
        )
        self.undetermined_pub = rospy.Publisher(
            settings.ROS_TOPICS.undetermined.topic,
            settings.ROS_TOPICS.undetermined.type,
        )

        # Create the collection site objects used by this plugin
        self.collection_sites = [
            collection_sites.CollectionSite(name_index, actuator_position)
            for name_index, actuator_position in enumerate(settings.COLLECTION_SITE_POSITIONS, start=1)
        ]
        self.active_collection_site = self.collection_sites[0]
        self.dirt_in_vacuum_chamber = False

        # for i, attr in enumerate(generate_actuator_button_names()):
        #     getattr(self._widget, attr).clicked[bool].connect(self.on_actuator_button_click(i))

        # self._widget.actuator_slider.setRange(0, len(ACTUATOR_BUTTON_VALUES) - 1)
        # self._widget.actuator_slider.valueChanged.connect(self.on_slider_change)

        self.register_signals()

    def register_signals(self):

        # Register signals for collection site buttons
        for site in self.collection_sites:
            self.get_widget_attr(site.button_name).clicked[bool].connect(
                self.on_collection_site_select(site)
            )

        # Register signals for the "important routine" button
        self.get_widget_attr(settings.OBJECT_NAMES.routine_button['important']).clicked.connect(self.on_click_important)

        self.get_widget_attr(settings.OBJECT_NAMES.reset_collection_states_button).clicked.connect(
            self.on_click_reset_collection_states
        )

        # Signals for all of the manual control toggles
        for button_key, object_name in settings.OBJECT_NAMES.control_button.items():
            self.get_widget_attr(object_name).toggled.connect(
                # Generate the method name that is responsible for handling the toggle signal for the current button
                getattr(self, 'on_toggle_control_{}'.format(button_key))
            )

        self.get_widget_attr(settings.OBJECT_NAMES.control_button['vacuum']).toggled.connect(
            self.on_toggle_control_vacuum
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
        """
        Construct the callback that should be called when the user select's the given collection site.
        """

        def _on_click():
            self.active_collection_site = collection_site
            m = settings.ROS_TOPICS.actuator_position.type(collection_site.actuator_position)
            self.actuator_button_pub.publish(m)

            self.refresh_collection_site_pixmaps()

        return _on_click

    def refresh_collection_site_pixmaps(self):
        """
        Reload all image labels for the collection sites.
        """
        for site in self.collection_sites:
            active = (site == self.active_collection_site)
            getattr(self._widget, site.label_name).setPixmap(site.pixmap(active))

    def on_click_important(self):
        """
        Callback called when the user clicks on the important button
        """
        import random
        self.get_widget_attr(settings.OBJECT_NAMES.routine_button['important']).setText(
            random.choice(settings.IMPORTANT_MESSAGES)
        )

    def on_click_reset_collection_states(self):
        for site in self.collection_sites:
            site.status = collection_sites.SiteStatus.Empty
        self.refresh_collection_site_pixmaps()

    def on_toggle_control_vacuum(self, checked):
        button = self.get_widget_attr(settings.OBJECT_NAMES.control_button['vacuum'])
        if checked:
            button.setText("Running")
            self.undetermined_pub.publish("Running vacuum")

            # If the valve is currently close, register that there is now dirt in the vacuum chamber
            if not self.check_control_on('valve'):
                self.dirt_in_vacuum_chamber = True
        else:
            button.setText("Not Running")
            self.undetermined_pub.publish("Stopping vacuum")

    def on_toggle_control_valve(self, checked):
        button = self.get_widget_attr(settings.OBJECT_NAMES.control_button['valve'])
        if checked:
            button.setText("Open")
            self.undetermined_pub.publish("Opening Valve")

            # If there is dirt currently in the vacuum, register that is is now in the active collection site
            if self.dirt_in_vacuum_chamber and self.active_collection_site.is_empty:
                self.active_collection_site.status = collection_sites.SiteStatus.Filled
                self.refresh_collection_site_pixmaps()
                self.dirt_in_vacuum_chamber = False
        else:
            button.setText("Closed")
            self.undetermined_pub.publish("Closing Valve")

    def on_toggle_control_vibration(self, checked):
        button = self.get_widget_attr(settings.OBJECT_NAMES.control_button['vibration'])
        if checked:
            button.setText("Running")
            self.undetermined_pub.publish("Running vibration motors")
        else:
            button.setText("Not Running")
            self.undetermined_pub.publish("Stopping vibration motors")

    def on_toggle_control_pump(self, checked):
        button = self.get_widget_attr(settings.OBJECT_NAMES.control_button['pump'])
        if checked:
            button.setText("Running")
            self.undetermined_pub.publish("Running pump")

            if self.active_collection_site.is_filled:
                self.active_collection_site.status = collection_sites.SiteStatus.Used
                self.refresh_collection_site_pixmaps()
        else:
            button.setText("Not Running")
            self.undetermined_pub.publish("Stopping pump")


    def check_control_on(self, button_name):
        """
        Check whether the control button with the given button name is currently checked
        """
        return self.get_widget_attr(settings.OBJECT_NAMES.control_button[button_name]).isChecked()

    def get_widget_attr(self, attr):
        return getattr(self._widget, attr)

    def splash_status_message(self, message):
        # TODO
        pass

    # def on_slider_change(self, position):
    #     m = msg.UInt8(ACTUATOR_BUTTON_VALUES[position])
    #     # self.actuator_button_pub.publish(m)

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog
