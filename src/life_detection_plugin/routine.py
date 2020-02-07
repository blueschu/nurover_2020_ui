from collections import namedtuple

from python_qt_binding import QtCore
from python_qt_binding.QtWidgets import QProgressBar, QApplication

from . import settings


class Routine(object):
    """
    A Life Detection Routine, incorporating a sequence of steps with associated durations.
    """
    _Step = namedtuple('_Step', ['duration', 'description', 'start_callback', 'finish_callback'])

    def __init__(self, widget, layout_object_name):
        self._widget = widget
        self._layout_object = getattr(self._widget, layout_object_name)

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._on_timer_timout)
        self._timer_counter = 0

        self._steps = []
        self._progress_bars = []
        self._active_bar_index = 0
        self._finalizer = None

    def run(self, callback=None):
        """Run this routine. Call the given callback once all steps are complete."""
        if not self._steps:
            return

        self._finalizer = callback
        self._create_progress_bars()

        # Run the start callback associated with the first step, if one was provided.
        start_callback = self._steps[0].start_callback
        if start_callback:
            start_callback()

        # Start the timer to periodically
        self._timer.start(settings.ROUTINE_TIMER_TICK)

    def add_step(self, duration, description, start_callback, finish_callback):
        """
        Add a step to this routine that runs for the given duration in milliseconds and that starts
        and finished with the given callbacks.
        """
        self._steps.append(Routine._Step(duration, description, start_callback, finish_callback))

    def add_step_click_control(self, duration, description, control_name, clicked):
        """
        Add a step to this routine that sets the given control to the specified clicked status.
        """
        self.add_step(
            duration,
            description,
            self._make_click_control(control_name, clicked),
            None,
        )

    def add_step_toggle_control(self, duration, description, control_name, start_clicked):
        """
        Add a step to this routine that toggles the given control at is start and end.

        If `start_clicked` is True, the given control will start "clicked" Otherwise, it will start "unclicked".
        """
        self.add_step(
            duration,
            description,
            self._make_click_control(control_name, start_clicked),
            self._make_click_control(control_name, not start_clicked),
        )

    def _reset(self):
        """
        Reset the state of this routine, deleting all progress bar widgets.
        """
        for bar in self._progress_bars:
            bar.deleteLater()
        self._progress_bars = []
        self._timer_counter = 0
        self._active_bar_index = 0

        if self._finalizer:
            self._finalizer()

    def _create_progress_bars(self):
        """
        Generate the progress bar widgets associated with this routine.
        """
        for step in self._steps:
            progress_bar = QProgressBar(self._widget)
            progress_bar.setRange(0, step.duration / settings.ROUTINE_TIMER_TICK)
            progress_bar.setValue(0)
            progress_bar.setFormat("{} (pending)".format(step.description))
            self._layout_object.addWidget(progress_bar)
            self._progress_bars.append(progress_bar)

    def _on_timer_timout(self):
        """
        Callback for this routine's timer. Called periodically upon each timeout event.
        """
        self._timer_counter += 1
        current_bar = self._progress_bars[self._active_bar_index]

        # Update the current progress bar
        current_bar.setValue(self._timer_counter)
        current_bar.setFormat("{} ({}s left)".format(
            self._steps[self._active_bar_index].description,
            ((current_bar.maximum() - self._timer_counter) * settings.ROUTINE_TIMER_TICK) // 1000
        ))

        # Check if the current progress bar is full
        if self._timer_counter >= current_bar.maximum():
            self._cycle_progress_bar()

    def _cycle_progress_bar(self):
        """
        Cycles the current progress bar to the bar associated with the next step in this routine.

        Call the finish callback for the current bar, and the start call for the next bar. If all
        steps have been completed, reset this routine.
        """
        self._active_bar_index += 1
        self._timer_counter = 0

        if self._active_bar_index == len(self._progress_bars):
            self._timer.stop()
            self._reset()
        else:
            self._progress_bars[self._active_bar_index - 1].setFormat(
                "{} (done)".format(self._steps[self._active_bar_index -1].description)
            )
            finish = self._steps[self._active_bar_index - 1].finish_callback
            start = self._steps[self._active_bar_index].start_callback
            if finish:
                finish()
            if start:
                start()

    def _get_control_button(self, control_name):
        """
        Return the button widget associated with the given control
        """
        return getattr(self._widget, settings.OBJECT_NAMES.control_button[control_name])

    def _make_click_control(self, control_name, ignore_if=None):
        """
        Construct a signal callback that will "click" the control button specified.

        If `ignore_if` is provided, this callback will perform no action is the specified control is already in
        the given state.

        For example, `self.make_control('vacuum', ignore_if=True)` will produce a callable that will turn on the
        vacuum if it is not already on. If the vacuum is already on (i.e. its click state is set to `True`), no
        action will be performed.
        """
        button = self._get_control_button(control_name)

        def _click_control():
            if (ignore_if is None) or (button.isChecked() != ignore_if):
                button.animateClick()

        return _click_control
