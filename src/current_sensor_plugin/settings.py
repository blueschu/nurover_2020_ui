import os
from collections import namedtuple

import rospkg
from std_msgs import msg


class _RosTopics(object):
    _Topic = namedtuple('_Topic', ['topic', 'type'])

    undetermined = _Topic("/undetermined_topic", msg.String)


class _ObjectNames(object):
    """
    Mapping to the Qt object names of the various widgets used by this plugin.
    """
    pass

ROS_TOPICS = _RosTopics()

OBJECT_NAMES = _ObjectNames()

PACKAGE_PATH = rospkg.RosPack().get_path('rover_ui')

RESOURCE_PATH = os.path.join(PACKAGE_PATH, 'resource')

