import enum
import logging
import time
import typing
from collections import namedtuple

import numpy as np
from lightpath import LightpathState
from ophyd.device import Component as Cpt
from ophyd.device import Device
from ophyd.device import FormattedComponent as FCpt
from ophyd.pseudopos import pseudo_position_argument, real_position_argument
from ophyd.signal import EpicsSignal, EpicsSignalRO, Signal
from ophyd.status import MoveStatus

from .beam_stats import BeamEnergyRequest
from .device import GroupDevice
from .device import UnrelatedComponent as UCpt
from .epics_motor import IMS, EpicsMotorInterface
from .interface import BaseInterface, FltMvInterface, LightpathMixin
from .pseudopos import (PseudoPositioner, PseudoSingleInterface, SyncAxis,
                        SyncAxisOffsetMode)
from .pv_positioner import PVPositionerIsClose
from .signal import InternalSignal
from .utils import doc_format_decorator, get_status_float

logger = logging.getLogger(__name__)

# Constants
si_111_dspacing = 3.1356011499587773
si_511_dspacing = 1.0452003833195924

# Defaults
default_theta0_deg = 15.1027
default_theta0 = default_theta0_deg * np.pi/180
default_dspacing = si_111_dspacing
default_gr = 3.175
default_gd = 231.303


class CCMMotor(PVPositionerIsClose):
    """
    Goofy records used in the CCM.
    """
    # Tolerance from old xcs python code
    atol = 3e-4

    setpoint = Cpt(EpicsSignal, ":POSITIONSET", auto_monitor=True,
                   doc='The motor setpoint. Writing begins a move.')
    readback = Cpt(EpicsSignalRO, ":POSITIONGET", auto_monitor=True,
                   kind='hinted',
                   doc='The current motor position.')


class CCMAlio(CCMMotor):
    """
    Controls specifically the Alio motor.

    Adds some controller-specific items.
    """
    cmd_home = Cpt(EpicsSignal, ':ENABLEPLC11', kind='omitted',
                   doc='Tells the controller to home the motor.')
    cmd_kill = Cpt(EpicsSignal, ':KILL', kind='omitted',
                   doc='Tells the controller to kill the PID.')

    def home(self) -> None:
        """
        Finds the reference used for the Alio's position.

        Same as pressing "HOME" in the edm screen.
        """
        self.cmd_home.put(1)

    def kill(self) -> None:
        """
        Terminates the motion PID

        Same as pressing "KILL" in the edm screen.
        """
        self.cmd_kill.put(1)


class CCMPico(EpicsMotorInterface):
    """
    The Pico motors used here seem non-standard, as they are missing spg.

    They still need the direction_of_travel fix from PCDSMotorBase.
    This is a bit hacky for now, something should be done in the epics_motor
    file to accomodate these.
    """
    direction_of_travel = Cpt(Signal, kind='omitted',
                              doc='The direction the motor is moving.')

    def _pos_changed(
        self,
        timestamp: typing.Optional[float] = None,
        old_value: typing.Optional[float] = None,
        value: typing.Optional[float] = None,
        **kwargs
    ) -> None:
        """
        Callback for when the readback position changes.

        Stores the internal travelling direction of the motor to account for
        the fact that our EPICS motor does not have a TDIR field.

        Uses the standard ophyd arguments for a value subscription.
        """
        try:
            comparison = int(value > old_value)
            self.direction_of_travel.put(comparison)
        except TypeError:
            # We have some sort of null/None/default value
            logger.debug('Could not compare value=%s > old_value=%s',
                         value, old_value)
        # Pass information to PositionerBase
        super()._pos_changed(timestamp=timestamp, old_value=old_value,
                             value=value, **kwargs)


class CCMConstantWarning(enum.IntEnum):
    # No warning
    NO_WARNING = 0
    # Disconnected at start, still disconnected
    ALWAYS_DISCONNECT = 1
    # Got a good value, disconnected later
    VALID_DISCONNECT = 2
    # Got a bad value, disconnected later
    INVALID_DISCONNECT = 3
    # Connected with a bad value
    INVALID_CONNECT = 4


class CCMConstantsMixin(Device):
    """
    Mixin class that includes PVs that hold CCM constants.

    This allows us to keep the CCM parameters synchronized between sessions
    and between different devices in the same session.

    Contains user PVs for theta0, dspacing, gr, and gd.
    These are intended to be run from the user pv notepad IOCs and are
    available in the most recent tags.

    Prefix can be any of the prefixes from any of the CCM motors.
    """
    theta0_deg = FCpt(
        EpicsSignal,
        '{_constants_prefix}:THETA0',
        kind='config',
        doc='Reference angle for the first crystal in deg.',
    )
    dspacing = FCpt(
        EpicsSignal,
        '{_constants_prefix}:DSPACING',
        kind='config',
        doc='Crystal lattice spacing.',
    )
    gr = FCpt(
        EpicsSignal,
        '{_constants_prefix}:GR',
        kind='config',
        doc=(
            'The radius of the sapphire ball '
            'connected to the Alio stage in mm.'
        ),
    )
    gd = FCpt(
        EpicsSignal,
        '{_constants_prefix}:GD',
        kind='config',
        doc=(
            'Distance between the rotation axis and the '
            'center of the sapphire sphere located on the '
            'Alio stage in mm.'
        ),
    )

    _enable_warn_constants: bool = True
    _theta0_deg: float
    _dspacing: float
    _gd: float
    _gr: float
    _initialized_signal_names: set
    _prev_warnings: list[CCMConstantWarning]
    _init_time: float

    def __init__(self, prefix: str, *args, **kwargs):
        if 'XPP' in prefix:
            self._constants_prefix = 'XPP:CCM'
        elif 'XCS' in prefix:
            self._constants_prefix = 'XCS:CCM'
        else:
            self._constants_prefix = 'TST:CCM'
        self._theta0_deg = default_theta0_deg
        self._dspacing = default_dspacing
        self._gd = default_gd
        self._gr = default_gr
        self._initialized_signal_names = set()
        self._prev_warnings = [CCMConstantWarning.NO_WARNING] * 4
        self._init_time = time.monotonic()
        super().__init__(prefix, *args, **kwargs)

    @theta0_deg.sub_value
    @dspacing.sub_value
    @gr.sub_value
    @gd.sub_value
    def _update_constant(
        self,
        value: float,
        obj: EpicsSignal,
        **kwargs
    ) -> None:
        """
        Put PV updates to an attribute for the calculation.

        Calculations should never rely on network resources directly.
        Instead, let us monitor and cache the values.
        """
        if obj is self.theta0_deg:
            self._theta0_deg = value
        elif obj is self.dspacing:
            self._dspacing = value
        elif obj is self.gr:
            self._gr = value
        elif obj is self.gd:
            self._gd = value
        self._initialized_signal_names.add(obj.name)

    @property
    def theta0_deg_val(self) -> float:
        """
        The theta0 value currently used in calculations.

        This is the reference angle for the first crystal in deg.

        This will be the value from the PV if things are working properly,
        otherwise it will fall back to the default value.
        """
        if self.theta0_deg.name in self._initialized_signal_names:
            return self._theta0_deg
        return default_theta0_deg

    @property
    def theta0_rad_val(self) -> float:
        """
        The theta0 value currently used in calculations.

        This is the reference angle for the first crystal in rad.

        This will be the value from the PV if things are working properly,
        otherwise it will fall back to the default value.
        """
        return self.theta0_deg_val * np.pi / 180

    @property
    def dspacing_val(self) -> float:
        """
        The dspacing value currently used in calculations.

        This is the crystal lattice spacing.

        This will be the value from the PV if things are working properly,
        otherwise it will fall back to the default value.

        This is necessary because a value of 0 is nonphysical and in the case
        of a disconnected value the show must go on.
        """
        if (
            self._dspacing
            and self.dspacing.name in self._initialized_signal_names
        ):
            return self._dspacing
        return default_dspacing

    @property
    def gr_val(self) -> float:
        """
        The gr value currently used in calculations.

        This is the radius of the sapphire ball
        connected to the Alio stage in mm.

        This will be the value from the PV if things are working properly,
        otherwise it will fall back to the default value.

        This is necessary because a value of 0 is nonphysical and in the case
        of a disconnected value the show must go on.
        """
        if self._gr and self.gr.name in self._initialized_signal_names:
            return self._gr
        return default_gr

    @property
    def gd_val(self) -> float:
        """
        The gd value currently used in calculations.

        This is the distance between the rotation axis and the
        center of the sapphire sphere located on the
        Alio stage in mm.

        This will be the value from the PV if things are working properly,
        otherwise it will fall back to the default value.

        This is necessary because a value of 0 is nonphysical and in the case
        of a disconnected value the show must go on.
        """
        if self._gd and self.gd.name in self._initialized_signal_names:
            return self._gd
        return default_gd

    @doc_format_decorator(
        default_theta0_deg=default_theta0_deg,
        default_dspacing=default_dspacing,
        default_gr=default_gr,
        default_gd=default_gd,
    )
    def reset_calc_constant_defaults(self, confirm: bool = True) -> None:
        """
        Put the default values into the ccm constants.

        This can be useful if values were reset due to autosave errors or if
        they've otherwise accidentally been set to crazy values.

        This relies on the default values in ccm.py being set up reasonably:
        theta0_deg = {default_theta0_deg}
        dspacing = {default_dspacing}
        gr = {default_gr}
        gd = {default_gd}

        Parameters
        ----------
        confirm : bool, optional
            If True, we'll ask for confirmation from the user before doing
            the reset. This is because an accidental reset can cost some
            time as we scramble to figure out what the values should be
            restored to.
        """
        if confirm:
            response = input(
                'Are you sure you want to reset the CCM constants? (y/n)\n'
                f'theta0_deg = {default_theta0_deg}\n'
                f'dspacing = {default_dspacing}\n'
                f'gr = {default_gr}\n'
                f'gd = {default_gd}\n'
            )
            if response.lower() != 'y':
                self.log.info('Aborting CCM reset_defaults')
                return
        self.log.info('Resetting to default CCM constants')
        self.theta0_deg.put(default_theta0_deg)
        self.dspacing.put(default_dspacing)
        self.gr.put(default_gr)
        self.gd.put(default_gd)

    def warn_invalid_constants(self, only_new: bool = False) -> None:
        """
        Warn if we have invalid values for our calculation constants.

        The motivation here is twofold:
        1. It should be easy for the user to know what is wrong and
           why. The calculations should not be opaque.
        2. The user should still be able to do "something" if there is
           an issue here.

        For values to be valid, the PVs need to be connected and all
        but theta0 must be nonzero. Theta0 should also not be zero,
        but it doesn't break the math and someone could conceivably
        set it to zero during debug.

        If this isn't satisfied, we will show an appropriate warning
        message when this method is called. The intention is for this
        to pop up whenever we run the forward/inverse calculations.

        For more detail, consider the following failure modes:
        1. The constant PVs don't connect and never connect
          - In this case, we must warn that the constants IOC is off
          - We should use the default values in calculations
        2. The constant PVs connect, but their values are zero
          - In this case, we must warn that the constant values were lost
          - We should use the default values in calculations
        3. The constant PVs connect, but disconnect later
          - In this case, we should warn that the IOC died
          - We should continue using the last known good values

        Parameters
        ----------
        only_new : bool, optional
            If False, the default, always show us the warnings.
            If True, do not show warnings if they have not changed.
        """
        if not self._enable_warn_constants:
            return
        if time.monotonic() - self._init_time < 10:
            return
        sigs = (self.theta0_deg, self.dspacing, self.gr, self.gd)
        vals = (self._theta0_deg, self._dspacing, self._gr, self._gd)
        default_vals = (default_theta0_deg, default_dspacing,
                        default_gr, default_gd)
        for num, (sig, val, default, old_warning) in enumerate(zip(
            sigs, vals, default_vals, self._prev_warnings,
        )):
            new_warning = self._check_valid_constant(sig, val)
            if new_warning != old_warning or not only_new:
                self._show_constant_warning(new_warning, sig, val, default)
            self._prev_warnings[num] = new_warning

    def _check_valid_constant(
        self,
        sig: EpicsSignal,
        val: float,
    ) -> CCMConstantWarning:
        """
        Return the CCMConstantWarning state for a particular signal.

        This is used internally in warn_invalid_constants.

        Parameters
        ----------
        sig : EpicsSignal
            The signal to check for.
        val : float
            The most recent cached value from that signal.

        Returns
        -------
        warn : CCMConstantWarning
            One of the enum states that represents our constant warning state.
            Note that one of the states is NO_WARNING- we don't necessarily
            have a problem.
        """
        good_value = val or sig is self.theta0_deg
        if sig.connected:
            if good_value:
                return CCMConstantWarning.NO_WARNING
            else:
                return CCMConstantWarning.INVALID_CONNECT
        elif sig.name in self._initialized_signal_names:
            if good_value:
                return CCMConstantWarning.VALID_DISCONNECT
            else:
                return CCMConstantWarning.INVALID_DISCONNECT
        return CCMConstantWarning.ALWAYS_DISCONNECT

    def _show_constant_warning(
        self,
        warning: CCMConstantWarning,
        sig: EpicsSignal,
        val: float,
        default: float
    ) -> None:
        """
        Log an appropriate warning to the object logger.

        This is used internally in warn_invalid_constants.

        Parameters
        ----------
        warning : CCMConstantWarning
            The appropriate warning category.
        sig : EpicsSignal
            The signal we're warning about, for use in the message.
        val : float
            The cached signal value, for use in the message.
        default : float
            The default constant value, for use in the message.
        """
        if warning == CCMConstantWarning.ALWAYS_DISCONNECT:
            self.log.warning(
                f'Calculation constant {sig.name} never connected. '
                'The IOC is probably offline or misconfigured. '
                f'Using the default value {default} for '
                'calculations.'
            )
        elif warning == CCMConstantWarning.VALID_DISCONNECT:
            self.log.warning(
                f'Calculation constant {sig.name} previously '
                'connected, but is now disconnected. '
                'The IOC must have gone down. '
                f'Using the last known good value {val}.'
            )
        elif warning == CCMConstantWarning.INVALID_DISCONNECT:
            self.log.warning(
                f'Calculation constant {sig.name} previously '
                'connected, but is now disconnected. '
                'The IOC must have gone down. '
                'Never had a good value, using the default value '
                f'{default} for calculations.'
            )
        elif warning == CCMConstantWarning.INVALID_CONNECT:
            self.log.warning(
                f'Calculation constant {sig.name} has an '
                f'invalid value of {val}. Using the default value '
                f'{default} for calculations. '
                'Consider calling reset_calc_constant_defaults to '
                'restore the default values to the constant PVs.'
            )


class CCMEnergy(FltMvInterface, PseudoPositioner, CCMConstantsMixin):
    """
    CCM energy motor.

    Calculates the current CCM energy using the alio position, and
    requests moves to the alio based on energy requests.

    Presents itself like a motor.

    Parameters
    ----------
    prefix : str
        The PV prefix of the Alio motor, e.g. XPP:MON:MPZ:07A
    """
    # Pseudo motor and real motor
    energy = Cpt(
        PseudoSingleInterface,
        egu='keV',
        kind='hinted',
        limits=(4, 25),
        verbose_name='CCM Photon Energy',
        doc=(
            'PseudoSingle that moves the calculated CCM '
            'selected energy in keV.'
        ),
    )
    alio = Cpt(CCMAlio, '', kind='normal',
               doc='The motor that rotates the CCM crystal.')

    # Calculation intermediates
    theta_deg = Cpt(InternalSignal, kind='normal',
                    doc='The crystal angle in degrees.')
    wavelength = Cpt(InternalSignal, kind='normal',
                     doc='The wavelength picked by the CCM in Angstroms.')
    resolution = Cpt(
        InternalSignal,
        kind='normal',
        doc=(
            'A measure of how finely we can control the ccm '
            'output at this position in eV/um.'
        ),
    )

    tab_component_names = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Alias the constants signals onto the main energy pseudomotor
        self.energy.theta0_deg = self.theta0_deg
        self.energy.dspacing = self.dspacing
        self.energy.gr = self.gr
        self.energy.gd = self.gd
        # Alias the position setter as well
        self.energy.set_current_position = self.set_current_position

    @alio.sub_default
    def _update_intermediates(
        self,
        value: typing.Optional[float] = None,
        **kwargs
    ) -> None:
        """
        Updates the calculation intermediates when the alio position updates.

        This includes the theta_deg, wavelength, and resolution signals.

        Parameters
        ----------
        value : float, optional
            The position of the alio motor, called as it updates.
            If this is not provided, the update will do nothing.
        kwargs : dict
            Other keywords from the subscription are ignored.
        """
        if value is None:
            return
        self.warn_invalid_constants(only_new=True)
        theta = alio_to_theta(
            value,
            self.theta0_rad_val,
            self.gr_val,
            self.gd_val,
        )
        wavelength = theta_to_wavelength(theta, self.dspacing_val)
        self.theta_deg.put(theta * 180/np.pi, force=True)
        self.wavelength.put(wavelength, force=True)

        res_delta = 1e-4
        ref1 = self.alio_to_energy(value - res_delta/2)
        ref2 = self.alio_to_energy(value + res_delta/2)
        self.resolution.put(abs((ref1 - ref2) / res_delta), force=True)

    @pseudo_position_argument
    def forward(self, pseudo_pos: namedtuple) -> namedtuple:
        """
        PseudoPositioner interface function for calculating the setpoint.

        Converts the requested energy to the real position of the alio.
        """
        pseudo_pos = self.PseudoPosition(*pseudo_pos)
        energy = pseudo_pos.energy
        alio = self.energy_to_alio(energy)
        return self.RealPosition(alio=alio)

    @real_position_argument
    def inverse(self, real_pos: namedtuple) -> namedtuple:
        """
        PseudoPositioner interface function for calculating the readback.

        Converts the real position of the alio to the calculated energy.
        """
        real_pos = self.RealPosition(*real_pos)
        alio = real_pos.alio
        energy = self.alio_to_energy(alio)
        return self.PseudoPosition(energy=energy)

    def energy_to_alio(self, energy: float) -> float:
        """
        Converts energy to alio.

        Parameters
        ----------
        energy : float
            The photon energy (color) in keV.

        Returns
        -------
        alio : float
            The alio position in mm
        """
        self.warn_invalid_constants(only_new=True)
        wavelength = energy_to_wavelength(energy)
        theta = wavelength_to_theta(wavelength, self.dspacing_val)
        alio = theta_to_alio(
            theta,
            self.theta0_rad_val,
            self.gr_val,
            self.gd_val,
        )
        return alio

    def alio_to_energy(self, alio: float) -> float:
        """
        Converts alio to energy.

        Parameters
        ----------
        alio : float
            The alio position in mm

        Returns
        -------
        energy : float
            The photon energy (color) in keV.
        """
        self.warn_invalid_constants(only_new=True)
        theta = alio_to_theta(
            alio,
            self.theta0_rad_val,
            self.gr_val,
            self.gd_val,
        )
        wavelength = theta_to_wavelength(theta, self.dspacing_val)
        energy = wavelength_to_energy(wavelength)
        return energy

    def set_current_position(self, energy: float) -> None:
        """
        Adjust the offset to make input energy the current position.

        This changes the value of the theta0 PV.
        """
        self.warn_invalid_constants(only_new=True)
        wavelength = energy_to_wavelength(energy)
        theta_rad_calc = wavelength_to_theta(wavelength, self.dspacing_val)
        theta_rad_no_offset = alio_to_theta(
            self.alio.position,
            theta0=0,
            gr=self.gr_val,
            gd=self.gd_val,
        )
        new_theta0_rad = theta_rad_calc - theta_rad_no_offset
        new_theta0_deg = new_theta0_rad * 180 / np.pi
        self.theta0_deg.put(new_theta0_deg)

    def move(self, *args, kill=True, wait=True, **kwargs):
        """
        Overwrite the move method to add a PID kill at the
        end of each move.

        Context: the PID loop keeps looking for the final position forever.
        The motor thus runs at too high duty cycles, heats up and causes
        vacuum spikes in the chamber. This has led to MPS trips.
        In addition, there is serious potential to fry the motor itself,
        as it is run too intensively.
        """
        if kill:
            # Must always wait if we are killing the PID
            wait = True
        st = super().move(*args, wait=wait, **kwargs)
        time.sleep(0.01)  # safety wait. Necessary?
        if kill:
            self.alio.kill()
        return st


class CCMEnergyWithVernier(CCMEnergy):
    """
    CCM energy motor and the vernier.

    Moves the alio based on the requested energy using the values
    of the calculation constants, and reports the current energy
    based on the alio's position.

    Also moves the vernier when a move is requested to the alio.
    Note that the vernier is in units of eV, while the energy
    calculations are in units of keV.

    Parameters
    ----------
    prefix : str
        The PV prefix of the Alio motor, e.g. XPP:MON:MPZ:07A
    hutch : str, optional
        The hutch we're in. This informs us as to which vernier
        PVs to write to. If omitted, we can guess this from the
        prefix.
    """
    acr_energy = FCpt(BeamEnergyRequest, '{hutch}', kind='normal',
                      doc='Requests ACR to move the Vernier.')

    # These are duplicate warnings with main energy motor
    _enable_warn_constants: bool = False
    hutch: str

    def __init__(
        self,
        prefix: str,
        hutch: typing.Optional[str] = None,
        **kwargs
    ):
        # Put some effort into filling this automatically
        # CCM exists only in two hutches
        if hutch is not None:
            self.hutch = hutch
        elif 'XPP' in prefix:
            self.hutch = 'XPP'
        elif 'XCS' in prefix:
            self.hutch = 'XCS'
        else:
            self.hutch = 'TST'
        super().__init__(prefix, **kwargs)

    @pseudo_position_argument
    def forward(self, pseudo_pos: namedtuple) -> namedtuple:
        """
        PseudoPositioner interface function for calculating the setpoint.

        Converts the requested energy to the real position of the alio,
        and also converts that energy to eV and passes it along to
        the vernier.
        """
        pseudo_pos = self.PseudoPosition(*pseudo_pos)
        energy = pseudo_pos.energy
        alio = self.energy_to_alio(energy)
        vernier = energy * 1000
        return self.RealPosition(alio=alio, acr_energy=vernier)

    @real_position_argument
    def inverse(self, real_pos: namedtuple) -> namedtuple:
        """
        PseudoPositioner interface function for calculating the readback.

        Converts the real position of the alio to the calculated energy
        """
        real_pos = self.RealPosition(*real_pos)
        alio = real_pos.alio
        energy = self.alio_to_energy(alio)
        return self.PseudoPosition(energy=energy)


class CCMEnergyWithACRStatus(CCMEnergyWithVernier):
    """
    CCM energy motor and ACR beam energy request with status.
    Note that in this case vernier indicates any ways that ACR will act on the
    photon energy request. This includes the Vernier, but can also lead to
    motion of the undulators or the K.

    Parameters
    ----------
    prefix : str
        The PV prefix of the Alio motor, e.g. XPP:MON:MPZ:07A
    hutch : str, optional
        The hutch we're in. This informs us as to which vernier
        PVs to write to. If omitted, we can guess this from the
        prefix.
    acr_status_sufix : str
        Prefix to the SIOC PV that ACR uses to report the move status.
        For HXR this usually is 'AO805'.
    """
    acr_energy = FCpt(BeamEnergyRequest, '{hutch}',
                      pv_index='{pv_index}',
                      acr_status_suffix='{acr_status_suffix}',
                      add_prefix=('suffix', 'write_pv', 'pv_index',
                                  'acr_status_suffix'),
                      kind='normal',
                      doc='Requests ACR to move the energy.')

    def __init__(
        self,
        prefix: str,
        hutch: typing.Optional[str] = None,
        acr_status_suffix='AO805',
        pv_index=2,
        **kwargs
    ):
        self.acr_status_suffix = acr_status_suffix
        self.pv_index = pv_index
        super().__init__(prefix, **kwargs)


class CCMX(SyncAxis):
    """
    Combined motion of the CCM X motors.

    You can use this device like a motor, and the position setpoint will be
    forwarded to both x motors.

    This is used to bring the CCM in and out of the beam.

    Parameters
    ----------
    prefix : str, optional
        Devices are required to have a positional argument here,
        but this is not used. If provided, it should be the same as
        down_prefix (x1).
    down_prefix : str, required keyword
        The prefix for the downstream ccm x translation motor (x1).
    up_prefix : str, required keyword
        The prefix for the upstream ccm x translation motor (x2).
    """
    down = UCpt(IMS, kind='normal',
                doc='Downstream ccm x translation motor (x1).')
    up = UCpt(IMS, kind='normal',
              doc='Upstream ccm x translation motor(x2).')

    offset_mode = SyncAxisOffsetMode.STATIC_FIXED
    tab_component_names = True

    def __init__(
        self,
        prefix: typing.Optional[str] = None,
        **kwargs
    ):
        UCpt.collect_prefixes(self, kwargs)
        prefix = prefix or self.unrelated_prefixes['down_prefix']
        super().__init__(prefix, **kwargs)


class CCMY(SyncAxis):
    """
    Combined motion of the CCM Y motors.

    You can use this device like a motor, and the position setpoint will be
    forwarded to all three y motors.

    These motors are typically powered off for RP reasons.

    Parameters
    ----------
    prefix : str, optional
        Devices are required to have a positional argument here,
        but this is not used. If provided, it should be the same as
        down_prefix (y1).
    down_prefix : str, required keyword
        The prefix for the downstream ccm y translation motor (y1).
    up_north_prefix : str, required keyword
        The prefix for the north upstream ccm y translation motor (y2).
    up_south_prefix : str, required keyword
        The prefix for the south upstream ccm y translation motor (y3).
    """
    down = UCpt(IMS, kind='normal',
                doc='Downstream ccm y translation motor (y1).')
    up_north = UCpt(IMS, kind='normal',
                    doc='North upstream ccm y translation motor (y2).')
    up_south = UCpt(IMS, kind='normal',
                    doc='South upstream ccm y translation motor (y3).')

    offset_mode = SyncAxisOffsetMode.STATIC_FIXED
    tab_component_names = True

    def __init__(
        self,
        prefix: typing.Optional[str] = None,
        **kwargs
    ):
        UCpt.collect_prefixes(self, kwargs)
        prefix = prefix or self.unrelated_prefixes['down_prefix']
        super().__init__(prefix, **kwargs)


class CCM(BaseInterface, GroupDevice, LightpathMixin, CCMConstantsMixin):
    """
    The full CCM assembly.

    This requires a huge number of motor pv prefixes to be passed in.
    Pay attention to this docstring because most of the arguments are in
    the kwargs.

    Parameters
    ----------
    prefix : str, optional
        Devices are required to have a positional argument here,
        but this is not used. If provided, it should be the same as
        alio_prefix.
    in_pos : float, required keyword
        The x position to consider as "inserted" into the beam.
    out_pos : float, required keyword
        The x position to consider as "removed" from the beam.
    alio_prefix : str, required keyword
        The PV prefix of the Alio motor, e.g. XPP:MON:MPZ:07A
    theta2fine_prefix : str, required keyword
        The PV prefix of the motor that controls the fine adjustment
        of the of the second crystal's theta angle.
    theta2coarse_prefix : str, required keyword
        The PV prefix of the motor that controls the coarse adjustment
        of the of the second crystal's theta angle.
    chi2_prefix : str, required keyword
        The PV prefix of the motor that controls the adjustment
        of the of the second crystal's chi angle.
    x_down_prefix : str, required keyword
        The prefix for the downstream ccm x translation motor (x1).
    x_up_prefix : str, required keyword
        The prefix for the upstream ccm x translation motor (x2).
    y_down_prefix : str, required keyword
        The prefix for the downstream ccm y translation motor (y1).
    y_up_north_prefix : str, required keyword
        The prefix for the north upstream ccm y translation motor (y2).
    y_up_south_prefix : str, required keyword
        The prefix for the south upstream ccm y translation motor (y3).
    acr_status_pv_index : int
        The index for the energy request PV in the case of the acr status
        wait. Default: 2.
    acr_status_suffix : str
        The suffix for the ACR status energy change move. Default to 'AO805'
    """
    energy = Cpt(
        CCMEnergy, '', kind='hinted',
        doc=(
            'PseudoPositioner that moves the alio in '
            'terms of the calculated CCM energy.'
        ),
    )
    energy_with_vernier = Cpt(
        CCMEnergyWithVernier, '', kind='normal',
        doc=(
            'PseudoPositioner that moves the alio in '
            'terms of the calculated CCM energy while '
            'also requesting a vernier move.'
        ),
    )
    energy_with_acr_status = FCpt(
        CCMEnergyWithACRStatus, '{prefix}', kind='normal',
        acr_status_suffix='{acr_status_suffix}',
        add_prefix=('suffix', 'write_pv', 'acr_status_suffix'),
        doc=(
            'PseudoPositioner that moves the alio in '
            'terms of the calculated CCM energy while '
            'also requesting an energy change to ACR. '
            'This will wait on ACR to complete the move.'
        ),
    )

    alio = UCpt(CCMAlio, kind='normal',
                doc='The motor that rotates the CCM crystal.')
    theta2fine = UCpt(
        CCMMotor, atol=0.01, kind='normal',
        doc=(
            'The motor that controls the fine adjustment '
            'of the of the second crystal theta angle.'
        ),
    )
    theta2coarse = UCpt(
        CCMPico, kind='normal',
        doc=(
            'The motor that controls the coarse adjustment '
            'of the of the second crystal theta angle.'
        ),
    )
    chi2 = UCpt(
        CCMPico, kind='normal',
        doc=(
            'The motor that controls the adjustment of the'
            'second crystal chi angle.'
        ),
    )
    x = UCpt(CCMX, add_prefix=[], kind='normal',
             doc='Combined motion of the CCM X motors.')
    y = UCpt(CCMY, add_prefix=[], kind='normal',
             doc='Combined motion of the CCM Y motors.')

    lightpath_cpts = ['x.up.user_readback']
    tab_whitelist = ['x1', 'x2', 'y1', 'y2', 'y3', 'E', 'E_Vernier',
                     'energy_with_acr_status', 'th2fine', 'alio2E', 'E2alio',
                     'alio', 'home', 'kill', 'insert', 'remove', 'inserted',
                     'removed']

    _in_pos: float
    _out_pos: float

    def __init__(
        self,
        *,
        prefix: typing.Optional[str] = None,
        in_pos: float,
        out_pos: float,
        **kwargs
    ):
        UCpt.collect_prefixes(self, kwargs)
        self._in_pos = in_pos
        self._out_pos = out_pos
        prefix = prefix or self.unrelated_prefixes['alio_prefix']
        self.acr_status_suffix = kwargs.get('acr_status_suffix', 'AO805')
        self.acr_status_pv_index = kwargs.get('acr_status_suffix', 2)
        super().__init__(prefix, **kwargs)

        # Aliases: defined by the scientists
        self.x1 = self.x.down
        self.x2 = self.x.up
        self.y1 = self.y.down
        self.y2 = self.y.up_north
        self.y3 = self.y.up_south
        self.E = self.energy.energy
        self.E.readback.name = f'{self.name}E'
        self.E_Vernier = self.energy_with_vernier.energy
        self.E_Vernier.readback.name = f'{self.name}E_Vernier'
        self.th2coarse = self.theta2coarse
        self.th2fine = self.theta2fine
        self.alio2E = self.energy.alio_to_energy
        self.E2alio = self.energy.energy_to_alio
        self.home = self.alio.home
        self.kill = self.alio.kill

        self._inserted = False
        self._removed = False

    def format_status_info(self, status_info: dict[str, typing.Any]) -> str:
        """
        Define how we're going to format the state of the CCM for the user.
        """
        # Pull out the numbers we want and format them, or show N/A if failed
        alio = get_status_float(status_info, 'alio', 'position', precision=4)
        theta = get_status_float(status_info, 'energy', 'theta_deg', 'value',
                                 precision=3)
        wavelength = get_status_float(status_info, 'energy', 'wavelength',
                                      'value', precision=4)
        energy = get_status_float(status_info, 'energy', 'energy', 'position',
                                  precision=4)
        res_mm = get_status_float(status_info, 'energy', 'resolution', 'value',
                                  scale=1e3, precision=1)
        res_um = get_status_float(status_info, 'energy', 'resolution', 'value',
                                  precision=2)
        x_down = get_status_float(status_info, 'x', 'down', 'position',
                                  precision=3)
        x_up = get_status_float(status_info, 'x', 'up', 'position',
                                precision=3)
        try:
            xavg = np.average([float(x_down), float(x_up)])
            xavg = f'{xavg:.3f}'
        except Exception:
            xavg = 'N/A'

        # Fill out the text
        text = f'alio   (mm): {alio}\n'
        text += f'angle (deg): {theta}\n'
        text += f'lambda  (A): {wavelength}\n'
        text += f'Energy (keV): {energy}\n'
        text += f'res (eV/mm): {res_mm}\n'
        text += f'res (eV/um): {res_um}\n'
        text += f'x @ (mm): {xavg} [x1,x2={x_down},{x_up}]\n'
        return text

    def calc_lightpath_state(self, x_up: float) -> LightpathState:
        """
        Update the fields used by the lightpath to determine in/out.

        Compares the x position with the saved in and out values.
        """
        self._inserted = bool(np.isclose(x_up, self._in_pos))
        self._removed = bool(np.isclose(x_up, self._out_pos))
        if self._removed:
            self._transmission = 1
        else:
            # Placeholder "small attenuation" value
            self._transmission = 0.9

        return LightpathState(
            inserted=self._inserted,
            removed=self._removed,
            output={self.output_branches[0]: self._transmission}
        )

    @property
    def inserted(self):
        return self._inserted

    @property
    def removed(self):
        return self._removed

    def insert(self, wait: bool = False) -> MoveStatus:
        """
        Move the x motors to the saved "in" position.

        Parameters
        ----------
        wait : bool, optional
            If True, wait for the move to complete.
            If False, return without waiting.

        Returns
        -------
        move_status : MoveStatus
            A status object that tells you information about the
            success/failure/completion status of the move.
        """
        return self.x.move(self._in_pos, wait=wait)

    def remove(self, wait: bool = False) -> MoveStatus:
        """
        Move the x motors to the saved "out" position.

        Parameters
        ----------
        wait : bool, optional
            If True, wait for the move to complete.
            If False, return without waiting.

        Returns
        -------
        move_status : MoveStatus
            A status object that tells you information about the
            success/failure/completion status of the move.
        """
        return self.x.move(self._out_pos, wait=wait)


# Calculations between alio position and energy, with all intermediates.
def theta_to_alio(theta: float, theta0: float, gr: float, gd: float) -> float:
    """
    Converts theta angle (rad) to alio position (mm).

    Theta_B:       scattering angle, the angle reduces when rotating clockwise
                   (Bragg angle)
    Theta_0:       scattering angle offset of the Si (111) first crystal
    Delta_Theta:   the effective scattering angle (adjusted with Alio stage)
    R = 0.003175m: radius of the sapphire ball connected to the Alio stage
    D = 0.232156m: distance between the Theta_B rotation axis and the center
                   of the sapphire sphere located on the Alio stage.
                   note: The current value that we're using for D is 0.231303 -
                   possibly measured by metrology

    Theta_B = Theta_0 + Delta_Theta
    Conversion formula:
    x = f(Delta_Theta) = D * tan(Delta_Theta)+(R/cos(Delta_Theda))-R
    Note that for ∆θ = 0, x = R
    """
    t_rad = theta - theta0
    return gr * (1 / np.cos(t_rad) - 1) + gd * np.tan(t_rad)


def alio_to_theta(alio: float, theta0: float, gr: float, gd: float) -> float:
    """
    Converts alio position (mm) to theta angle (rad).

    Conversion function
    theta_angle = f(x) = 2arctan * [(sqrt(x^2 + D^2 + 2Rx) - D)/(2R + x)]
    Note that for x = −R, θ = 2 arctan(−R/D)
    """
    return theta0 + 2 * np.arctan(
        (np.sqrt(alio ** 2 + gd ** 2 + 2 * gr * alio) - gd) / (2 * gr + alio)
    )


def wavelength_to_theta(wavelength: float, dspacing: float) -> float:
    """Converts wavelength (A) to theta angle (rad)."""
    return np.arcsin(wavelength/2/dspacing)


def theta_to_wavelength(theta: float, dspacing: float) -> float:
    """Converts theta angle (rad) to wavelength (A)."""
    return 2*dspacing*np.sin(theta)


def energy_to_wavelength(energy: float) -> float:
    """Converts photon energy (keV) to wavelength (A)."""
    return 12.39842/energy


def wavelength_to_energy(wavelength: float) -> float:
    """Converts wavelength (A) to photon energy (keV)."""
    return 12.39842/wavelength
