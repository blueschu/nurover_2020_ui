import os
from collections import namedtuple

import rospkg
import rospy
from std_msgs import msg


class _RosTopics(object):
    _Topic = namedtuple('_Topic', ['topic', 'type'])

    current = _Topic("/drive/motor_current", msg.Float32MultiArray)

    undetermined = _Topic("/undetermined_topic", msg.String)


class _ObjectNames(object):
    """
    Mapping to the Qt object names of the various widgets used by this plugin.
    """
    current_plot = 'graph_widget'

    current_bar_layout = 'current_bar_layout'


ROS_TOPICS = _RosTopics()

OBJECT_NAMES = _ObjectNames()

PACKAGE_PATH = rospkg.RosPack().get_path('rover_ui')

RESOURCE_PATH = os.path.join(PACKAGE_PATH, 'resource')

DEFAULT_CURRENT_IGNORE_DURATION = rospy.Duration.from_sec(1)
"""
The amount of time for which new current messages will be ignored after displaying
a set of current measurements. Increase this value to improve plotting performance.
"""

CURRENT_BAR_COLOR_THRESHOLDS = [
    (0000, (0x8A, 0xE2, 0x34)),
    (1500, (0x4E, 0x9A, 0x06)),
    (3000, (0xFC, 0xE9, 0x4F)),
    (5000, (0xFC, 0xAF, 0x3E)),
    (6800, (0xEF, 0x29, 0x29)),
]
"""
Thresholds in milliamps for current sensor bar colors.
"""

DEFAULT_SENSOR_COLORS = [
    (0x3c, 0xb4, 0x4b),
    (0x43, 0x63, 0xd8),
    (0x42, 0xd4, 0xf4),
    (0x91, 0x1e, 0xb4),
    (0x9A, 0x63, 0x24),
    (0xf5, 0x82, 0x31),
]
"""
The color used to represented the first N current sensors displayed by this plugin.
Additional sensors will be represented by randomly chosen colors.
"""
