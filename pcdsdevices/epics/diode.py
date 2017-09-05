#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Diodes
"""
############
# Standard #
############
import logging

###############
# Third Party #
###############

##########
# Module #
##########
from .aerotech import DiodeAero
from .device import Device
from .component import Component
from .areadetector.detectors import GigeDetector

logger = logging.getLogger(__name__)


class DiodeBase(Device):
    """
    Base class for the diode.
    """
    pass


class HamamatsuDiode(DiodeBase):
    """
    Class for the Hamamatsu diode.
    """
    pass


class HamamatsuXMotionDiode(Device):
    """
    Class for the Hamamatsu diode but with an X motor
    """
    diode = Component(HamamatsuDiode, ":DIODE")
    x = Component(DiodeAero, ":X")


class HamamatsuXYMotionCamDiode(HamamatsuXMotionDiode):
    """
    Class for the Hamamatsu diode but with X and Y motors
    """
    y = Component(DiodeAero, ":Y")
    cam = Component(GigeDetector, ":CAM")
    
