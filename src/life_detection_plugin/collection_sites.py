"""
Utilities for tracking the states of the Life Detection system's collection sites.
"""

import os
from enum import Enum, unique

from . import settings

from python_qt_binding.QtGui import QPixmap

_PIXEL_MAP_EMPTY = 'empty.png'

_PIXEL_MAP_FILLED = 'filled.png'

_PIXEL_MAP_USED = 'used.png'

_PIXEL_MAP_EMPTY_ACTIVE = 'empty_active.png'

_PIXEL_MAP_FILLED_ACTIVE = 'filled_active.png'

_PIXEL_MAP_USED_ACTIVE = 'used_active.png'


@unique
class SiteStatus(Enum):
    """The possible states for the rover's collections sites"""
    Empty = 1
    Filled = 2
    Used = 3


_pixel_map_cache = {
    (SiteStatus.Empty, False): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_EMPTY),
    (SiteStatus.Filled, False): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_FILLED),
    (SiteStatus.Used, False): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_USED),
    (SiteStatus.Empty, True): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_EMPTY_ACTIVE),
    (SiteStatus.Filled, True): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_FILLED_ACTIVE),
    (SiteStatus.Used, True): os.path.join(settings.RESOURCE_PATH, _PIXEL_MAP_USED_ACTIVE),
}
"""
Mapping to the image files associated with the various states of the collection sites.

Keys of are of the form (SiteStatus, is_site_active).
"""


class CollectionSite(object):
    """A collection site with an associated label and selection button."""

    def __init__(self, index, actuator_position):
        self.status = SiteStatus.Empty
        self.actuator_position = actuator_position
        self.index = index

    def __eq__(self, other):
        return self.index == other.index

    @property
    def button_name(self):
        """Return the object name of the Qt button associated with this collection site."""
        return "{}_{}".format(settings.OBJECT_NAMES.collection_site_button, self.index)

    @property
    def label_name(self):
        """Return the object name of the Qt label associated with this collection site."""
        return "{}_{}".format(settings.OBJECT_NAMES.collection_site_label, self.index)

    @property
    def is_empty(self):
        """Return True is this site is empty."""
        return self.status == SiteStatus.Empty

    @property
    def is_filled(self):
        """Return True is this site is currently filled with an unused sample."""
        return self.status == SiteStatus.Filled

    @property
    def is_used(self):
        """Return True is this site's sample has already been used."""
        return self.status == SiteStatus.Used

    def pixmap(self, active):
        """Return the ``QPixmap`` image that corresponds to the state of this collection site."""
        return QPixmap(_pixel_map_cache[(self.status, active)])
