from ophyd.device import Component as Cpt
from ophyd.pseudopos import PseudoSingle

from pcdsdevices.device import ObjectComponent as OCpt
from pcdsdevices.interface import FltMvInterface
from pcdsdevices.pseudopos import (PseudoPositioner, PseudoSingleInterface,
                                   pseudo_position_argument,
                                   real_position_argument)


class Lens4EScan(FltMvInterface, PseudoPositioner):
    _real = ("mono_energy", "beam_size")
    _pseudo = ("energy", )

    mono_energy = OCpt('{mono_energy}')
    beam_size = OCpt('{lens_stack.beam_size}')

    energy = Cpt(PseudoSingleInterface)

    tab_whitelist = ['calc_lens_pos', 'mono_energy', 'stack']
    tab_component_names = True

    def __init__(self, mono_energy, lens_stack,
                 desired_beamsize=5e-6,
                 *args, **kwargs):
        self.mono = mono_energy
        self.stack = lens_stack
        self.desired_beamsize = desired_beamsize
        super().__init__(*args, **kwargs)

    @pseudo_position_argument
    def forward(self, pseudo_pos):
        return self.RealPosition(mono_energy=pseudo_pos.energy,
                                 beam_size=self.desired_beamsize)

    @real_position_argument
    def inverse(self, real_pos):
        return self.PseudoPosition(energy=real_pos.mono_energy)

    @pseudo_position_argument
    def move(self, position, *args, **kwargs):
        self.stack.energy = position.energy
        st = super().move(position, *args,
                          moved_cb=self.cb_open_beamstop,
                          **kwargs)
        return st

    def calc_lens_pos(self, energy):
        """
        Return the expected lens position (x,y,z) for a given energy.
        """
        stack_current_energy = self.stack.energy
        if self.stack._which_E != 'User':
            stack_current_energy = None
        self.stack.energy = energy
        lens_pos = self.stack.forward(beam_size=self.desired_beamsize)
        self.stack.energy = stack_current_energy
        return lens_pos

    def cb_open_beamstop(self, obj):
        """
        Callback function to open the attenuator or pulse picker at the end of a
        move. If the atttenuator is used, this function assumes that the blade #9
        is used to block the beam.
        """
        if 'Attenuator' in self.stack._att_obj.__class__.__name__:
            self.stack._att_obj.filters[9].remove()
        elif 'PulsePicker' in self.stack._att_obj.__class__.__name__:
            self.stack._att_obj.open()
        return

    def _concurrent_move(self, real_pos, **kwargs):
        """
        Try done fix: override the waiting list with the pseudopos parents,
        not pseudosingle, since cb is called on the parent in this case.

        This is needed because PseudoSingle does not report the right status,
        as they defer the motion to to their parent class (PseudoPositioner).
        When building a pseudo-positioner made of PseudoSingles, one needs to
        point the _real_waiting to the PseudoSingles' parent class.
        """
        for real_axis in self._real:
            if isinstance(real_axis, PseudoSingle):
                self._real_waiting.append(real_axis.parent)
            else:
                self._real_waiting_append(real_axis)

        for real, value in zip(self._real, real_pos):
            self.log.debug("[concurrent] Moving %s to %s", real.name, value)
            real.move(value, wait=False, moved_cb=self._real_finished, **kwargs)
