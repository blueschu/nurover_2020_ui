import os
from collections import namedtuple

import rospkg
from std_msgs import msg


class _RosTopics(object):
    _Topic = namedtuple('_Topic', ['topic', 'type'])

    actuator_position = _Topic("/ld/actuator_servo", msg.UInt8)

    linkage_servo = _Topic("/ld/linkage_servo", msg.Float32)

    valve_servo = _Topic("/ld/trapdoor_servo", msg.Float32)

    water_solenoid = _Topic("ld/solenoid", msg.Bool)

    vacuum_activation = _Topic("ld/vacuum", msg.Bool)

    vibration_motor_activation = _Topic("/ld/motors", msg.Bool)

    undetermined = _Topic("/undetermined_topic", msg.String)


class _ObjectNames(object):
    """
    Mapping to the Qt object names of the various widgets used by this plugin.
    """
    collection_site_label = 'collection_site_label'

    collection_site_button = 'collection_site_button'

    reset_collection_states_button = 'button_reset_collection_states'

    routine_button = {
        'collect': 'routine_button_collect',
        'purge': 'routine_button_purge',
        'runtest': 'routine_button_runtest',
        'important': 'routine_button_important',
    }

    control_button = {
        'vacuum': 'control_button_vacuum',
        'valve': 'control_button_valve',
        'vibration': 'control_button_vibration',
        'pump': 'control_button_pump',
    }

    progress_bar_layout = 'progress_bars'

    button_vacuum_up = 'button_vacuum_up'

    button_vacuum_down = 'button_vacuum_down'

    sync_label = 'sync_label'


ROUTINE_TIMER_TICK = 5

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
    1,
    2,
    3,
    4,
]
