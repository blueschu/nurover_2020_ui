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


