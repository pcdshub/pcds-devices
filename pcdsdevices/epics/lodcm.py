#!/usr/bin/env python
# -*- coding: utf-8 -*-
from enum import Enum

from .device import Device
from .state import statesrecord_class
from .component import Component


LodcmStates = statesrecord_class("LodcmStates", ":OUT", ":C", ":Si")


class LODCM(Device):
    h1n_state = Component(LodcmStates, ":H1N")
    h2n_state = Component(LodcmStates, ":H2N")

    light_states = Enum("LightStates", "BLOCKED MAIN MONO BOTH UNKNOWN",
                        start=0)

    def destination(self, line=None):
        """
        Return where the light is going for a given line at the current LODCM
        state.

        Parameters
        ----------
        line: str or int, optional
            The starting line for the beam. If this is 1 or "MAIN", light is
            coming in on H1N from the main line. If this is 2 or "MONO", light
            is coming in on H2N to the mono line (e.g. XCS periscope)

        Returns
        -------
        destination: str
            "BLOCKED" if the light (probably) does not go through.
            "MAIN" if the light continues on the main line.
            "MONO" if the light continues on the mono line.
            "BOTH" if the light continues on both lines.
            "UNKNOWN" if the state is strange.
        """
        if line is None:
            line = self.light_states.MAIN
        else:
            try:
                line = self.light_states[line]
            except:
                line = self.light_states(line)
        if line == self.light_states.MAIN:
            # H2N:     OUT      C       Si
            table = [["MAIN", "MAIN", "MAIN"],  # H1N at OUT
                     ["MAIN", "BOTH", "MAIN"],  # H1N at C
                     ["BLOCKED", "BLOCKED", "MONO"]]  # H1N at Si
            try:
                n1 = ("OUT", "C", "Si").index(self.h1n_state.value)
                n2 = ("OUT", "C", "Si").index(self.h2n_state.value)
            except ValueError:
                return "UNKNOWN"
            return table[n1][n2]
        elif line == self.light_states.MONO:
            table = ["MONO", "BLOCKED", "BLOCKED"]
            try:
                n2 = ("OUT", "C", "Si").index(self.h2n_state.value)
            except ValueError:
                return "UNKNOWN"
            return table[n2]
