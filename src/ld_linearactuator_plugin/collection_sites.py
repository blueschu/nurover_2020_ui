import os
from enum import Enum, unique

from . import settings

from python_qt_binding.QtGui import QPixmap

_PIXEL_MAP_EMPTY = 'empty.png'

_PIXEL_MAP_FILLED = 'filled.png'

_PIXEL_MAP_USED = 'empty.png'

_PIXEL_MAP_EMPTY_ACTIVE = 'empty_active.png'

_PIXEL_MAP_FILLED_ACTIVE = 'filled_active.png'

_PIXEL_MAP_USED_ACTIVE = 'empty_active.png'


@unique
class _SiteStatus(Enum):
    Empty = 1
    Filled = 2
    Used = 3


_pixel_map_cache = {
    (_SiteStatus.Empty, False): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_EMPTY),
    (_SiteStatus.Filled, False): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_FILLED),
    (_SiteStatus.Used, False): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_USED),
    (_SiteStatus.Empty, True): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_EMPTY_ACTIVE),
    (_SiteStatus.Filled, True): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_FILLED_ACTIVE),
    (_SiteStatus.Used, True): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_USED_ACTIVE),
}


class CollectionSite(object):

    def __init__(self, index, actuator_position):
        self.status = _SiteStatus.Empty
        self.actuator_position = actuator_position
        self.index = index

    def __eq__(self, other):
        return self.index == other.index

    @property
    def button_name(self):
        return "{}_{}".format(settings.OBJECT_NAMES.collection_site_button, self.index)

    @property
    def label_name(self):
        return "{}_{}".format(settings.OBJECT_NAMES.collection_site_label, self.index)

    def pixmap(self, active):
        return QPixmap(_pixel_map_cache[(self.status, active)])
