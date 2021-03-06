"""
Current sensor plotting plugin for Watney Mk. 2.

For a brief pyqtgraph tutorial, see https://www.learnpyqt.com/courses/graphics-plotting/plotting-pyqtgraph/
For a tutorial on adding pyqtgraph widgets with QtCreator, see https://www.learnpyqt.com/courses/qt-creator/embed-pyqtgraph-custom-widgets-qt-app/
For a tutorial on creating slots/signals, see https://www.pythoncentral.io/pysidepyqt-tutorial-creating-your-own-signals-and-slots/.
For QtCore.Qt constants: https://doc.qt.io/qtforpython/PySide2/QtCore/Qt.html


Tentative Pre-Competition Improvements
- Allow for live selection of sensor colors
- Save chosen sensor color between sessions
- Allow for live selection of color thresholds
- Allow for live selection of colors associated with color thresholds
"""

import os
import random
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, QFrame

try:
    import pyqtgraph
except:
    print "The python package 'pyqtgraph' is not installed but is required by the current sensor" \
          "plugin. Please run\n" \
          "    pip install pyqtgraph\n" \
          "to install this dependency."
    raise

from . import settings, current_display


class CurrentSensorPlugin(Plugin):
    some_sig = QtCore.pyqtSignal(tuple)

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

        # Register subscriber for the current sensor topic
        #
        # For this subscriber's callback, we use a lambda which extracts
        # the tuple data from the message and immediately emits a signal
        # to a custom slot. This approach was chosen since we need to process
        # the message from the RQT/GUI thread and *NOT* from the rospy thread
        # due to the fast that we need to make changes to Qt widgets, which
        # is not safe to do from non-gui threads (testing this resulted in
        # seg faults).
        #
        # If additional logic needs to be accomplished from the rospy thread,
        # migrate the lambda below to a method of this plugin.
        self.current_sub = rospy.Subscriber(
            settings.ROS_TOPICS.current.topic,
            settings.ROS_TOPICS.current.type,
            lambda message: self.some_sig.emit(message.data),
        )
        self.undetermined_pub = rospy.Publisher(
            settings.ROS_TOPICS.undetermined.topic,
            settings.ROS_TOPICS.undetermined.type,
        )

        self.current_sensor_displays = []

        self.setup_plot(
            self.get_widget_attr(settings.OBJECT_NAMES.current_plot)
        )

        self.some_sig.connect(self.on_receive_current_measurement_signal)

        self.startup_time = rospy.Time.now()
        self.last_current_display_time = self.startup_time - settings.DEFAULT_CURRENT_IGNORE_DURATION

    def shutdown_plugin(self):
        self.current_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def setup_plot(self, widget):
        # Add Background colour to white
        widget.setBackground('w')
        # Add Title
        widget.setTitle('Live Current Sensor Telemetry')
        # Add Axis Labels
        widget.setLabel('left', 'Current (Amps)', size=30)
        widget.setLabel('bottom', 'Time (s)', size=30)
        # Add legend
        widget.addLegend()
        # Add grid
        widget.showGrid(x=True, y=True)
        # Set Range
        widget.setYRange(0, 10, padding=0)
        for threshold, (r, g, b) in settings.CURRENT_BAR_COLOR_THRESHOLDS:
            widget.addLine(y=float(threshold) / 1000, pen=pyqtgraph.mkPen(r, g, b, style=QtCore.Qt.DotLine, width=1.6))

    def grow_current_sensor_display(self):
        """
        Adds an extra current sensor output to this plugin.

        This method will generate an additional line on the current sensor plot and
        an additional progress bar to represent new current sensor.
        """
        current_sensor_index = len(self.current_sensor_displays) + 1
        # Pick a known distinctive color if one is still available. Otherwise, generate
        # a random color for the new sensor.
        try:
            sensor_color = settings.DEFAULT_SENSOR_COLORS[current_sensor_index - 1]
        except IndexError:
            sensor_color = tuple(random.randint(0, 255) for _ in range(3))

        # Create pyqtgraph plot dataline for the new current sensor
        plot_widget = self.get_widget_attr(settings.OBJECT_NAMES.current_plot)
        pen = pyqtgraph.mkPen(sensor_color, style=QtCore.Qt.SolidLine, width=2)
        data_line = plot_widget.plot([], [], name='Sensor %i' % current_sensor_index,
                                     pen=pen)  # possible include symbol='+'

        # Create a progress bar for the new current sensor
        layout = QVBoxLayout()
        progress_bar_widget = QProgressBar(self._widget)
        progress_bar_widget.setOrientation(QtCore.Qt.Vertical)
        progress_bar_widget.setMaximum(10000)
        progress_bar_widget.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(progress_bar_widget, alignment=QtCore.Qt.AlignCenter)
        # Create a layout for displaying the sensor's color and value
        label_layout = QHBoxLayout()
        label_pixamp = QtGui.QPixmap(10, 10)
        label_pixamp.fill(QtGui.QColor(*sensor_color))
        label_color = QLabel(self._widget)
        label_color.setPixmap(label_pixamp)
        label_color.setFixedWidth(10)
        label_layout.addWidget(label_color)
        label_measurement = QLabel(self._widget)
        label_measurement.setAlignment(QtCore.Qt.AlignCenter)
        label_layout.addWidget(label_measurement)
        layout.addLayout(label_layout)
        # Add sensor name label
        label = QLabel(self._widget)
        label.setText('Sensor %i' % current_sensor_index)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        layout.addWidget(label)

        # Add new progress bar to the main window
        all_bar_layout = self.get_widget_attr(settings.OBJECT_NAMES.current_bar_layout)
        all_bar_layout.addLayout(layout)

        current_sensor_display = current_display.CurrentDisplay(data_line, progress_bar_widget, label_measurement)
        self.current_sensor_displays.append(current_sensor_display)

    @QtCore.pyqtSlot(tuple)
    def on_receive_current_measurement_signal(self, current_measurements):
        current_time = rospy.Time.now()
        plot_time = current_time - self.startup_time

        # if (current_time - self.last_current_display_time) < settings.DEFAULT_CURRENT_IGNORE_DURATION:
        #     return
        # else:
        #     self.last_current_display_time = current_time

        for index, current_measurement in enumerate(current_measurements):
            try:
                self.current_sensor_displays[index].update_outputs(plot_time.to_sec(), current_measurement)
            except IndexError:
                self.grow_current_sensor_display()
                self.current_sensor_displays[index].update_outputs(plot_time.to_sec(), current_measurement)

    def get_widget_attr(self, attr):
        """
        Return the given attributed of this plugin's widget

        Implemented to improve code readability.
        """
        return getattr(self._widget, attr)
