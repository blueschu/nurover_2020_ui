import os
from collections import namedtuple

import rospkg
from std_msgs import msg


class _RosTopics(object):
    _Topic = namedtuple('_Topic', ['topic', 'type'])

    actuator_position = _Topic("/life_detection/actuator_position", msg.UInt8)


class _ObjectNames(object):

    collection_site_label = 'collection_site_label'

    collection_site_button = 'collection_site_button'


ROS_TOPICS = _RosTopics()

OBJECT_NAMES = _ObjectNames()

PACKAGE_PATH = rospkg.RosPack().get_path('rover_ui')

RESOURCE_PATH = os.path.join(PACKAGE_PATH, 'resource')
#
# # {
# #     'actuator_position': {
# #         'topic': ,
# #         'type': ,
# #     },
# # }
#
# BASE_OBJECT_NAME_ACTUATOR_BUTTON = 'collection_site_button'
#
# OBJECT_NAME_FORMATS = object()
# OBJECT_NAME_FORMATS.__dict__.update({
#
#     'test' : 1
#
# })
#
# OBJECT_COLLECTION_SITE_LABEL = "collection_site_label_{}"



COLLECTION_SITE_POSITIONS = [
    0,
    64,
    128,
    192,
    255
]