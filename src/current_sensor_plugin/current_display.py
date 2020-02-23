import random

from python_qt_binding import QtGui, QtCore

from . import settings


class CurrentDisplay(object):
    """
    Container class for all of the widgets associated with a given motor's current data.
    """

    def __init__(self, plot_data_line, progress_bar_widget, max_measurements=20):
        self._plot_data_line = plot_data_line
        self._progress_bar_widget = progress_bar_widget
        self._max_measurements = max_measurements
        self._data_time = []
        self._data_current = []

    def update_outputs(self, time_secs, current_measurement):
        self._data_time.append(time_secs)
        self._data_current.append(current_measurement)

        for data_list in (self._data_time, self._data_current):
            if len(data_list) > self._max_measurements:
                data_list.pop(0)

        self._plot_data_line.setData(self._data_time, self._data_current)
        self.set_bar_value(current_measurement)

    def set_bar_value(self, current_measurement):
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

        # color_map = settings.CURRENT_BAR_COLOR_THRESHOLDS
        # if current_measurement >= color_map['guzzling']:
        #     color = QtCore.Qt.red
        # elif current_measurement >= color_map['drinking']:
        #     color = QtCore.Qt.darkYellow
        # elif current_measurement >= color_map['sipping']:
        #     color = QtCore.Qt.yellow
        # elif current_measurement >= color_map['tasting']:
        #     color = QtCore.Qt.darkGreen
        # else:
        #     color = QtCore.Qt.green
        #
        # return QtGui.QColor(color)
