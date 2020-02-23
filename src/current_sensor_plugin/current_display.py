import random

from python_qt_binding import QtGui, QtCore

from . import settings


class CurrentDisplay(object):
    """
    Container class for all of the widgets associated with a given motor's current data.
    """

    def __init__(self, plot_data_line, progress_bar_widget, max_measurements=35):
        self._plot_data_line = plot_data_line
        self._progress_bar_widget = progress_bar_widget
        self._max_measurements = max_measurements
        self._data_time = []
        self._data_current = []
        self._data_finger = 0

    def update_outputs(self, time_secs, current_measurement):
        if len(self._data_time) == self._max_measurements:
            self._data_time[self._data_finger] = time_secs
            self._data_current[self._data_finger] = current_measurement
            self._data_finger = (self._data_finger + 1) % self._max_measurements
        else:
            self._data_time.append(time_secs)
            self._data_current.append(current_measurement)

        # Update the plot associated with this current sensor
        self._plot_data_line.setData(self._data_time, self._data_current)
        # Update the progress bar associated with this current sensor
        self._set_bar_value(current_measurement)

    def _set_bar_value(self, current_measurement):
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Highlight, self._color_for_current(current_measurement))
        self._progress_bar_widget.setPalette(palette)
        self._progress_bar_widget.setValue(current_measurement * 1000)


    def _color_for_current(self, current_measurement):
        color = QtGui.QColor(QtCore.Qt.darkGray)
        current_measurement *= 1000  # Convert amps to milliamps
        for (threshold, (r, g, b)) in settings.CURRENT_BAR_COLOR_THRESHOLDS:
            if current_measurement >= threshold:
                color = QtGui.QColor.fromRgb(r, g, b)
        return color

