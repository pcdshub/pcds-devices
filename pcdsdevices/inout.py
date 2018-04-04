"""
Module to centralize code related to devices that can be ``IN`` or ``OUT``.

The classes that are defined here can be used to create other devices that need
methods such as ``insert`` or ``remove`` and the ability to mark discrete
states as in the beam or out of the beam.
"""
import math

from ophyd.sim import NullStatus

from .doc_stubs import basic_positioner_init, insert_remove
from .state import StatePositioner, StateRecordPositioner, PVStatePositioner


class InOutPositioner(StatePositioner):
    """
    Basic in/out `StatePositioner`. It can be inserted, removed and queried for
    insertion and removal state. It can also define transmission values for the
    various states.
%s
    Attributes
    ----------
    in_states: ``list` of ``str``
        State values that should be considered ``IN``.

    out_states: ``list`` of ``str``
        State values that should be considered ``OUT``.

    _transmission: ``dict{str: float}``
        Mapping from each state to the transmission ratio. This should be a
        number from 0 to 1. Default values will be 1 (full transmission) for
        ``out_states``, 0 (full block) for ``in_states``, and nan (no idea!)
        for unaccounted states.
    """
    __doc__ = __doc__ % basic_positioner_init

    states_list = ['IN', 'OUT']
    in_states = ['IN']
    out_states = ['OUT']
    _transmission = {}

    def __init__(self, prefix, *, name, **kwargs):
        super().__init__(prefix, name=name, **kwargs)
        self._trans_enum = {}
        self._extend_trans_enum(self.in_states, 0)
        self._extend_trans_enum(self.out_states, 1)

    @property
    def inserted(self):
        """
        True if the device is inserted
        """
        return self._pos_in_list(self.in_states)

    @property
    def removed(self):
        """
        True if the device is removed
        """
        return self._pos_in_list(self.out_states)

    def insert(self, moved_cb=None, timeout=None, wait=False):
        """
        Insert this device.

        Moves this device to the first state on the `in_states` list.
        """
        return self.move(self.in_states[0], moved_cb=moved_cb,
                         timeout=timeout, wait=wait)

    def remove(self, moved_cb=None, timeout=None, wait=False):
        """
        Macro to move this device to the first state on the out_states list.
        If we're already at some other out state, do nothing instead.
        """
        if self.removed:
            return NullStatus()
        return self.move(self.out_states[0], moved_cb=moved_cb,
                         timeout=timeout, wait=wait)

    insert.__doc__ += insert_remove
    remove.__doc__ += insert_remove

    @property
    def transmission(self):
        state = self.get_state(self.position)
        return self._trans_enum.get(state, math.nan)

    def _extend_trans_enum(self, state_list, default):
        for state in state_list:
            enumst = self.get_state(state)
            self._trans_enum[enumst] = self._transmission.get(state, default)

    def _pos_in_list(self, state_list):
        current_state = self.get_state(self.position)
        for state in state_list:
            if current_state == self.get_state(state):
                return True
        return False


class InOutRecordPositioner(StateRecordPositioner, InOutPositioner):
    """
    `InOutPositioner` for a standard states record.

    Positioner for a motor that moves to states ``IN`` and ``OUT`` using a
    standard states record. This can be subclassed for other states records
    that involve inserting and removing something into the beam.
    """
    __doc__ += basic_positioner_init


class TTReflaser(InOutRecordPositioner):
    """
    Motor stack that includes both a timetool and a reflaser.
    """
    __doc__ += basic_positioner_init

    states_list = ['TT', 'REFL', 'OUT']
    in_states = ['TT', 'REFL']


class InOutPVStatePositioner(PVStatePositioner, InOutPositioner):
    """
    `InOutPositioner` on top of a `PVStatePositioner`

    Positioner for a set of PVs that result in aggregate IN and OUT states for
    a single device. This must be subclassed and provided a _state_logic
    attribute to be used. Consult the `PVStatePositioner` documentation for
    more information.
    """
    __doc__ += basic_positioner_init
