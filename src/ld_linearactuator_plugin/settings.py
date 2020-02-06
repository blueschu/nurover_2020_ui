import os
from collections import namedtuple

import rospkg
from std_msgs import msg


class _RosTopics(object):
    _Topic = namedtuple('_Topic', ['topic', 'type'])

    actuator_position = _Topic("/life_detection/actuator_position", msg.UInt8)

    undetermined = _Topic("/undetermined_topic", msg.String)


class _ObjectNames(object):
    collection_site_label = 'collection_site_label'

    collection_site_button = 'collection_site_button'

    routine_button = {
        'collect': 'routine_button_collect',
        'purge': 'routine_button_purge',
        'runtest': 'routine_button_runtest',
        'important': 'routine_button_important',
    }


ROS_TOPICS = _RosTopics()

OBJECT_NAMES = _ObjectNames()

PACKAGE_PATH = rospkg.RosPack().get_path('rover_ui')

RESOURCE_PATH = os.path.join(PACKAGE_PATH, 'resource')

IMPORTANT_MESSAGES = [
    "Launch Fireworks",
    "Do Not Press",
    "Fire Torpedos",
    "Initiate Peace Talks",
    "Start Martian Invasion",
    "Release Milk",
    "Dump Fuel",
    "Lower Ailerons",
    "Open Wormhole",
    "Read ISO-3103",
    "Panic!",
    "Cycle passwords",
    "Farm Potatos",
    "Deploy Solar Array",
    "Shields To Maximum",
    "Replace Ink",
]

COLLECTION_SITE_POSITIONS = [
    0,
    64,
    128,
    192,
    255
]
