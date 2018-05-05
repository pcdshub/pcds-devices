"""
PCDS detectors and overrides for ophyd detectors.

All components at the detector level such as plugins  or image processing
functions needed by all instances of a detector are added here.
"""
import logging

from ophyd.areadetector import cam
from ophyd.areadetector.detectors import DetectorBase
from ophyd.device import Component as Cpt

from .plugins import ImagePlugin, StatsPlugin

logger = logging.getLogger(__name__)


__all__ = ['PCDSDetectorBase',
           'PCDSDetector']


class PCDSDetectorBase(DetectorBase):
    """
    Standard area detector with no plugins.
    """
    cam = Cpt(cam.CamBase, ":")


class PCDSDetector(PCDSDetectorBase):
    """
    Standard area detector with standard plugins.

    Geared towards analyzing a beam spot.

    IMAGE2: reduced rate image
    Stats2: reduced rate stats
    """
    image = Cpt(ImagePlugin, ':IMAGE2:', read_attrs=['array_data'])
    stats = Cpt(StatsPlugin, ':Stats2:', read_attrs=['centroid',
                                                     'mean_value',
                                                     'sigma_x',
                                                     'sigma_y'])

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stage_sigs['stats.compute_statistics'] = 'Yes'
        self.stage_sigs['stats.compute_centroid'] = 'Yes'
