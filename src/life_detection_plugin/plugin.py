import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QApplication, QStyle, QCommonStyle
from python_qt_binding import QtCore, QtGui

from . import settings, collection_sites, routine


class LifeDetectionPlugin(Plugin):

    def __init__(self, context):
        super(LifeDetectionPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('LifeDetectionPlugin')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(settings.RESOURCE_PATH, 'life_detection.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Watney Mk. 2 Life Detection')

        # From RQT Tutorial: Show _widget.windowTitle on left-top of each plugin. This is useful
        # when you open multiple plugins at once. Also if you open multiple instances of your plugin at once,
        # these lines add number to make it easy to tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Create publisher instances for the the ROS topics this plugin writes to
        self.publishers = {
            ros_topic: rospy.Publisher(
                ros_topic.topic,
                ros_topic.type,
            ) for ros_topic in settings.ROS_TOPICS.all_topics()
        }

        # Create the collection site objects used by this plugin
        self.collection_sites = [
            collection_sites.CollectionSite(name_index, actuator_position)
            for name_index, actuator_position in enumerate(settings.COLLECTION_SITE_POSITIONS, start=1)
        ]
        self.active_collection_site = self.collection_sites[0]
        # Whether there is currently dirt in the vacuum chamber
        self.dirt_in_vacuum_chamber = False
        # Whether a life detection is currently running. Only one may run at a time.
        self._routine_running = False

        self.register_signals()
        self.set_startup_icons()

    def get_widget_attr(self, attr):
        """
        Return the given attributed of this plugin's widget

        Implemented to improve code readability.
        """
        return getattr(self._widget, attr)

    def register_signals(self):
        """
        Register the Qt signals required by this plugin
        """

        # Register signals for collection site buttons
        for site in self.collection_sites:
            self.get_widget_attr(site.button_name).clicked[bool].connect(
                self.make_collection_site_select_callback(site)
            )

        self.get_widget_attr(settings.OBJECT_NAMES.reset_collection_states_button).clicked.connect(
            self.on_click_reset_collection_states
        )

        for routine_name, object_name in settings.OBJECT_NAMES.routine_button.items():
            self.get_widget_attr(object_name).clicked.connect(
                # Generate the method name that is responsible for handling the clock signal for the routine button
                getattr(self, 'on_run_routine_{}'.format(routine_name))
            )

        # Signals for all of the manual control toggles
        for button_key, object_name in settings.OBJECT_NAMES.control_button.items():
            self.get_widget_attr(object_name).toggled.connect(
                # Generate the method name that is responsible for handling the toggle signal for the current button
                getattr(self, 'on_toggle_control_{}'.format(button_key))
            )

    def set_startup_icons(self):
        # Set button icons
        def _set_icon(widget, icon):
            w = self.get_widget_attr(widget)
            w.setIcon(w.style().standardIcon(icon))

        self.set_icon(settings.OBJECT_NAMES.button_vacuum_up, QStyle.SP_ArrowUp)
        self.set_icon(settings.OBJECT_NAMES.button_vacuum_down, QStyle.SP_ArrowDown)
        self.set_icon(settings.OBJECT_NAMES.reset_collection_states_button, QStyle.SP_TrashIcon)

        style = QCommonStyle()
        self.get_widget_attr(settings.OBJECT_NAMES.sync_label).setPixmap(
            QtGui.QPixmap(style.standardIcon(QStyle.SP_DialogApplyButton).pixmap(QtCore.QSize(15, 15)))
        )

        # Set "Off" icon for all of the manual control
        for _, object_name in settings.OBJECT_NAMES.control_button.items():
            self.set_icon(object_name, QStyle.SP_DialogNoButton)

    def shutdown_plugin(self):
        """
        Un-register all ROS publishers.
        """
        for pub in self.publishers.values():
            pub.unregister()


    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def make_collection_site_select_callback(self, collection_site):
        """
        Construct the callback that should be called when the user select's the given collection site.
        """

        def _on_click():
            self.active_collection_site = collection_site
            self.ros_publish_message(
                settings.ROS_TOPICS.actuator_position,
                collection_site.actuator_position
            )

            r = routine.Routine(self._widget, settings.OBJECT_NAMES.progress_bar_layout)
            r.add_step(2000, 'Moving linear actuator...', None, None)
            self.run_routine(r)

            self.refresh_collection_site_pixmaps()

        return _on_click

    def refresh_collection_site_pixmaps(self):
        """
        Reload all image labels for the collection sites.
        """
        for site in self.collection_sites:
            active = (site == self.active_collection_site)
            getattr(self._widget, site.label_name).setPixmap(site.pixmap(active))

    def on_click_reset_collection_states(self):
        for site in self.collection_sites:
            site.status = collection_sites.SiteStatus.Empty
        self.refresh_collection_site_pixmaps()

    def on_toggle_control_vacuum(self, checked):
        button = self.get_widget_attr(settings.OBJECT_NAMES.control_button['vacuum'])
        if checked:
            button.setText("Running")
            self.set_icon(button, QStyle.SP_DialogYesButton)
            self.ros_publish_message(settings.ROS_TOPICS.vibration_motor_activation, True)

            # If the valve is currently close, register that there is now dirt in the vacuum chamber
            if not self.check_control_on('valve'):
                self.dirt_in_vacuum_chamber = True
        else:
            button.setText("Not Running")
            self.set_icon(button, QStyle.SP_DialogNoButton)
            self.ros_publish_message(settings.ROS_TOPICS.vibration_motor_activation, False)

    def on_toggle_control_valve(self, checked):
        button = self.get_widget_attr(settings.OBJECT_NAMES.control_button['valve'])
        if checked:
            button.setText("Open")
            self.set_icon(button, QStyle.SP_DialogYesButton)
            self.ros_publish_message(settings.ROS_TOPICS.valve_servo, True)

            # If there is dirt currently in the vacuum, register that is is now in the active collection site
            if self.dirt_in_vacuum_chamber and self.active_collection_site.is_empty:
                self.active_collection_site.status = collection_sites.SiteStatus.Filled
                self.refresh_collection_site_pixmaps()
                self.dirt_in_vacuum_chamber = False
        else:
            button.setText("Closed")
            self.set_icon(button, QStyle.SP_DialogNoButton)
            self.ros_publish_message(settings.ROS_TOPICS.valve_servo, False)

    def on_toggle_control_vibration(self, checked):
        button = self.get_widget_attr(settings.OBJECT_NAMES.control_button['vibration'])
        if checked:
            button.setText("Running")
            self.set_icon(button, QStyle.SP_DialogYesButton)
            self.ros_publish_message(settings.ROS_TOPICS.vibration_motor_activation, True)
        else:
            button.setText("Not Running")
            self.set_icon(button, QStyle.SP_DialogNoButton)
            self.ros_publish_message(settings.ROS_TOPICS.vibration_motor_activation, False)

    def on_toggle_control_pump(self, checked):
        button = self.get_widget_attr(settings.OBJECT_NAMES.control_button['pump'])
        if checked:
            button.setText("Running")
            self.set_icon(button, QStyle.SP_DialogYesButton)
            self.ros_publish_message(settings.ROS_TOPICS.water_solenoid, True)

            if self.active_collection_site.is_filled:
                self.active_collection_site.status = collection_sites.SiteStatus.Used
                self.refresh_collection_site_pixmaps()
        else:
            button.setText("Not Running")
            self.set_icon(button, QStyle.SP_DialogNoButton)
            self.ros_publish_message(settings.ROS_TOPICS.water_solenoid, False)

    def on_run_routine_collect(self):
        """
        Signal handler for when the user presses the "Collect Sample" routine button.
        """
        if self.dirt_in_vacuum_chamber:
            confirmed = self.prompt_confirmation(
                "There is already dirt in the vacuum chamber.\n"
                "Are you sure you want to start the sample collection procedure?"
            )
            if not confirmed:
                return

        if not self.active_collection_site.is_empty:
            confirmed = self.prompt_confirmation(
                "The currently selected collection site is not empty.\n"
                "Are your sure you want to start the sample collection procedure?"
            )
            if not confirmed:
                return

        vacuum_duration = 1000 * self.get_widget_attr(settings.OBJECT_NAMES.routine_button['collect'] + '_spin').value()
        r = routine.Routine(self._widget, settings.OBJECT_NAMES.progress_bar_layout)
        r.add_step_click_control(3000, "Closing valve...", 'valve', clicked=False)
        r.add_step_click_control(vacuum_duration, "Running Vacuum...", 'vacuum', clicked=True)
        r.add_step_click_control(3000, "Powering down vacuum...", 'vacuum', clicked=False)
        r.add_step_click_control(3000, "Opening valve...", 'valve', clicked=True)

        self.run_routine(r)

    def on_run_routine_purge(self):
        """
        Signal handler for when the user presses the "Purge" routine button.
        """

        purge_duration = 1000 * self.get_widget_attr(settings.OBJECT_NAMES.routine_button['purge'] + '_spin').value()
        r = routine.Routine(self._widget, settings.OBJECT_NAMES.progress_bar_layout)
        r.add_step_click_control(3000, "Opening valve...", 'valve', clicked=True)
        # Durations of less tha the timer tick will result in a "busy indicator" instead of the usual progress bar.
        # Because of this, we need to use the routine timer tick to indicate an instantaneous step.
        r.add_step_click_control(settings.ROUTINE_TIMER_TICK, "Starting vacuum", 'vacuum', clicked=True)
        r.add_step_click_control(purge_duration, "Running vibration motors", 'vibration', clicked=True)
        r.add_step_click_control(settings.ROUTINE_TIMER_TICK, "Stopping vibration motors", 'vibration', clicked=False)
        r.add_step_click_control(3000, "Powering down vacuum...", 'vacuum', clicked=False)

        self.run_routine(r)

    def on_run_routine_runtest(self):
        """
        Signal handler for when the user presses the "Run Test" routine button.
        """
        if not self.active_collection_site.is_filled:
            confirmed = self.prompt_confirmation(
                "The currently selected collection is not filled with a fresh sample.\n"
                "Are your sure you want to start a test in this site?"
            )
            if not confirmed:
                return

        test_duration = 1000 * self.get_widget_attr(settings.OBJECT_NAMES.routine_button['runtest'] + '_spin').value()
        r = routine.Routine(self._widget, settings.OBJECT_NAMES.progress_bar_layout)
        r.add_step_toggle_control(test_duration, "Pumping water into collection site...", 'pump', start_clicked=True)

        self.run_routine(r)

    def on_run_routine_important(self):
        """
        Callback called when the user clicks on the important button
        """
        import random
        self.get_widget_attr(settings.OBJECT_NAMES.routine_button['important']).setText(
            random.choice(settings.IMPORTANT_MESSAGES)
        )

    def check_control_on(self, button_name):
        """
        Check whether the control button with the given button name is currently checked
        """
        return self.get_widget_attr(settings.OBJECT_NAMES.control_button[button_name]).isChecked()

    def prompt_confirmation(self, message, title="Confirmation Required"):
        """Create a Yes/No confirmation message. Return True is the user selected 'Yes'. """
        reply = QMessageBox.question(self._widget, title, message, QMessageBox.Yes, QMessageBox.Cancel)
        return reply == QMessageBox.Yes

    def run_routine(self, active_routine):
        """
        Run the given life detection routine.
        """
        if self._routine_running:
            QMessageBox.critical(
                self._widget,
                "Routine Start Error",
                "Cannot start new routine - a routine is already running!",
                QMessageBox.Ok
            )
            return

        self._routine_running = True

        def notify_routine_end():
            self._routine_running = False

        active_routine.run(notify_routine_end)

        # Due to an issue with threading and the Qt event loop with ROS, the timer will not run its timeout
        # callable UNLESS control is explicitly returned to the applicaion to process events.
        while self._routine_running:
            QApplication.processEvents(QtCore.QEventLoop.AllEvents)

    def set_icon(self, widget, icon):
        if not isinstance(widget, QWidget):
            widget = self.get_widget_attr(widget)
        widget.setIcon(widget.style().standardIcon(icon))

    def ros_publish_message(self, settings_topic, *args, **kwargs):
        self.publishers[settings_topic].publish(
            settings_topic.type(*args, **kwargs)
        )