"""
Module for square-one tri-sphere motion system.

An Ophyd classes for controlling a square-one tri-sphere motion
system. The motion system consists of multiple axes, each of which can be
controlled individually or collectively.
The axes include X, Y, Z, rX, rY, and rZ.

Reference: https://www.sqr-1.com/tsp.html
"""
from __future__ import annotations

import enum
import logging
import typing

from ophyd.device import Component as Cpt
from ophyd.device import Device
from ophyd.device import FormattedComponent as FCpt
from ophyd.pv_positioner import PVPositionerIsClose
from ophyd.signal import EpicsSignal
from ophyd.status import MoveStatus as StatusBase
from ophyd.status import wait as status_wait

from pcdsdevices.interface import FltMvInterface

logger = logging.getLogger(__name__)


class Axis(str, enum.Enum):
    """
    An enumeration representing axes available in tri-sphere table stage.

    - 'X': X-axis translation
    - 'Y': Y-axis translation
    - 'Z': Z-axis translation
    - 'rX': X-axis rotation
    - 'rY': Y-axis rotation
    - 'rZ': Z-axis rotation
    """

    X = 'X'
    Y = 'Y'
    Z = 'Z'
    rX = 'rX'
    rY = 'rY'
    rZ = 'rZ'


class SQR1Axis(FltMvInterface, PVPositionerIsClose):
    """
    Single axis of the square-one tri-sphere motion system.

    A PVPositionerIsClose subclass that includes attributes for setting
    the setpoint, reading back the current position, actuating the motion,
    and stopping the motion.

    Parameters
    ----------
    prefix : str
        The EPICS PV prefix for this axis.
    axis : str
        The axis identifier ('X', 'Y', 'Z', 'rX', 'rY', 'rZ').
    sync_setpoints : callable or None, optional
        A callback function to synchronize setpoints before moving the axis.
    ``**kwargs`` : dict
        Additional keyword arguments to pass to the base class constructor

    Attributes
    ----------
    setpoint : `ophyd.signal.EpicsSignal`
        An EPICS signal for setting the desired position of the axis.
    readback : `ophyd.signal.EpicsSignal`
        An EPICS signal for reading back the current position of the axis.
    actuate : `ophyd.signal.EpicsSignal`
        An EPICS signal for initiating the motion of the axis.
    actuate_value : any, optional
        The value to be set on the 'actuate' signal to initiate motion.
    stop_signal : `ophyd.signal.EpicsSignal`
        An EPICS signal for stopping the motion of the axis.
    stop_value : any, optional
        The value to be set on the 'stop_signal' to stop motion.

    Example:
        >>> axis_x = SQR1Axis(prefix="SQR1:AXIS_X", axis="X")
        >>> axis_x.move(5.0)  # Move the X-axis to position 5.0
    """

    setpoint = FCpt(EpicsSignal, "{prefix}:TARGET:{_axis}", kind="normal")
    readback = FCpt(EpicsSignal, "{prefix}:TARGET:{_axis}:RBV", kind="hinted")
    actuate = Cpt(EpicsSignal, ":MOV", kind="normal")
    actuate_value = 1
    stop_signal = Cpt(EpicsSignal, ":KILL", kind="normal")
    stop_value = 1

    def __init__(
        self,
        prefix: str,
        axis: Axis,
        sync_setpoints: typing.Callable | None = None,
        **kwargs,
    ):
        self._axis = axis.value
        self._sync_setpoints = sync_setpoints
        super().__init__(prefix, **kwargs)

    def move(
        self,
        position: float,
        wait: bool = True,
        timeout: float = None,
        moved_cb: typing.Callable | None = None,
        sync_enable: bool = True,
    ):
        """
        Move the axis to the specified position.

        Parameters
        ----------
        position : float
            The desired position to move the axis to.
        wait : bool, optional
            If `True` (default), wait for motion to complete before returning.
            If `False`, return immediately after initiating the motion.
        timeout : float, optional
            The maximum time (in seconds) to wait for the motion to complete.
        moved_cb : callable or None, optional
            A callback function to execute when the motion has completed.
        sync_enable : bool, optional
            If `True` (default) synchronize setpoints before moving the axis.

        Returns
        -------
        MoveStatus
            Track the state of a movement from some initial to final position.

        Example
        -------
        >>> axis_x = SQR1Axis(prefix="SQR1:AXIS_X", axis="X")
        >>> axis_x.move(5.0)  # Move the X-axis to position 5.0
        """
        if sync_enable and self._sync_setpoints:
            self._sync_setpoints()
        return super().move(position, wait, timeout, moved_cb)

    def set(
        self,
        new_position: typing.Any,
        *,
        timeout: float = None,
        moved_cb: typing.Callable = None,
        wait: bool = False,
    ) -> StatusBase:
        """
        Set SQR1 single axis new position.

        Parameters
        ----------
        new_position : typing.Any
            The target position to move to.
        timeout : float, optional
            The maximum time to wait for the move to complete.If None,
            no timeout is applied.
        moved_cb : typing.Callable, optional
            A callback function that is called when the move is complete.
        wait : bool, optional
            always will be false. kept for consistent method signature
            across future implementations.

        Returns
        -------
        StatusBase
            Status object to indicate when the motion is done.

        Notes
        -----
        This method calls the `move` method with the provided parameters,
        while keeping `wait=False` and `sync_enable=False` to allow multi-axis
        scans without one axis blocking other axes.

        """
        return self.move(new_position,
                         wait=False,
                         moved_cb=moved_cb,
                         timeout=timeout,
                         sync_enable=False)


class SQR1(Device):
    """
    Ophyd device represents the entire square-one tri-sphere motion system.

    It consists of multiple axes (X, Y, Z, rX, rY, rZ) that can be controlled
    individually or simultaneously.It aggregates instances of the `SQR1Axis`
    class for each axis and provides methods for multi-axis movement and
    stopping the motion.

    Parameters
    ----------
    prefix : str, optional
        The PV prefix for all components of the device
    name : str, keyword only
        The name of the device (as will be reported via read()`
    kind : a member of the :class:`~ophydobj.Kind` :class:`~enum.IntEnum`
        (or equivalent integer), optional
        Default is ``Kind.normal``. See :class:`~ophydobj.Kind` for options.
    read_attrs : sequence of attribute names
        DEPRECATED: the components to include in a normal reading
        (i.e., in ``read()``)
    configuration_attrs : sequence of attribute names
        DEPRECATED: the components to be read less often (i.e., in
        ``read_configuration()``) and to adjust via ``configure()``
    parent : instance or None, optional
        The instance of the parent device, if applicable

    Attributes
    ----------
    x : `SQR1Axis`
        An instance of the `SQR1Axis` class representing the X-axis.
    y : `SQR1Axis`
        An instance of the `SQR1Axis` class representing the Y-axis.
    z : `SQR1Axis`
        An instance of the `SQR1Axis` class representing the Z-axis.
    rx : `SQR1Axis`
        An instance of the `SQR1Axis` class representing the rX-axis.
    ry : `SQR1Axis`
        An instance of the `SQR1Axis` class representing the rY-axis.
    rz : `SQR1Axis`
        An instance of the `SQR1Axis` class representing the rZ-axis.
    stop_signal : `ophyd.signal.EpicsSignal`
        An EPICS signal for stopping the motion of the entire tri-sphere
        motion system.

    Example:
        >>> tri_sphere = SQR1(prefix="SQR1:", name="tri_sphere")
        >>> tri_sphere.multi_axis_move(x_sp=1.0, y_sp=2.0, z_sp=3.0)
        >>> tri_sphere.stop()
    """

    x = Cpt(SQR1Axis, "", axis=Axis.X, rtol=0.001)
    y = Cpt(SQR1Axis, "", axis=Axis.Y, rtol=0.001)
    z = Cpt(SQR1Axis, "", axis=Axis.Z, rtol=0.001)
    rx = Cpt(SQR1Axis, "", axis=Axis.rX, rtol=0.001)
    ry = Cpt(SQR1Axis, "", axis=Axis.rY, rtol=0.001)
    rz = Cpt(SQR1Axis, "", axis=Axis.rZ, rtol=0.001)
    actuate = Cpt(EpicsSignal, ":MOV", kind="normal")
    actuate_value = 1
    stop_signal = Cpt(EpicsSignal, ":KILL", kind="normal")
    stop_value = 1

    def __init__(
        self,
        prefix="",
        *,
        name,
        kind=None,
        read_attrs=None,
        configuration_attrs=None,
        parent=None,
        **kwargs,
    ):
        super().__init__(
            prefix,
            name=name,
            kind=kind,
            read_attrs=read_attrs,
            configuration_attrs=configuration_attrs,
            parent=parent,
            **kwargs,
        )
        self.x._sync_setpoints = self.sync_setpoints
        self.y._sync_setpoints = self.sync_setpoints
        self.z._sync_setpoints = self.sync_setpoints
        self.rx._sync_setpoints = self.sync_setpoints
        self.ry._sync_setpoints = self.sync_setpoints
        self.rz._sync_setpoints = self.sync_setpoints

    def sync_setpoints(self):
        """
        Synchronize setpoints of multiple axes with their respective readback values.

        This method has to be called at tri-sphere motion system startup or
        to avoid moving uninitialized axes to zero unintentionally.
        """
        logger.debug("sync_setpoint")
        self.x.setpoint.put(self.x.readback.get())
        self.y.setpoint.put(self.y.readback.get())
        self.z.setpoint.put(self.z.readback.get())
        self.rx.setpoint.put(self.rx.readback.get())
        self.ry.setpoint.put(self.ry.readback.get())
        self.rz.setpoint.put(self.rz.readback.get())

    def multi_axis_move(
        self,
        preset_pos: str | None = None,
        x_sp: float | None = None,
        y_sp: float | None = None,
        z_sp: float | None = None,
        rx_sp: float | None = None,
        ry_sp: float | None = None,
        rz_sp: float | None = None,
        wait: bool = True,
        timeout: float = 30.0,
    ):
        """
        Move one or multiple axes simultaneously to specified positions.

        Parameters
        ----------
        preset_pos: str or None, optional
            Move to a preset position labeled by the preset_pos string.
        x_sp : float or None, optional
            The desired position for the X-axis.If None (default), the current
            position is used.
        y_sp : float or None, optional
            The desired position for the Y-axis.If None (default), the current
            position is used.
        z_sp : float or None, optional
            The desired position for the Z-axis.If None (default), the current
            position is used.
        rx_sp : float or None, optional
            The desired position for the rX-axis.If None (default), the current
            position is used.
        ry_sp : float or None, optional
            The desired position for the rY-axis.If None (default), the current
            position is used.
        rz_sp : float or None, optional
            The desired position for the rZ-axis.If None (default), the current
            position is used.
        wait : bool, optional
            If True (default), wait for motion to complete before returning.
            If False, return immediately after initiating the motion.
        timeout : float, optional
            The maximum time (in seconds) to wait for the motion to complete.
            Default is 30.0 seconds.

        Returns
        -------
        status: MoveStatus
            status object that combins the motion of all axes.

        Notes
        -----
        If any of axes fail to initiate motion, the overall status is False.

        Example
        -------
        >>> tri_sphere = SQR1(prefix="SQR1:", name="tri_sphere")
        >>> status = tri_sphere.multi_axis_move(x_sp=1.0, y_sp=2.0, z_sp=3.0)
        """

        if preset_pos is not None:
            axis_list = [self.x, self.y, self.z, self.rz, self.ry, self.rz]
            if any(hasattr(a.presets.positions, preset_pos) for a in axis_list):
                x_sp = (getattr(self.x.presets.positions, preset_pos)).pos
                y_sp = (getattr(self.y.presets.positions, preset_pos)).pos
                z_sp = (getattr(self.z.presets.positions, preset_pos)).pos
                rx_sp = (getattr(self.rx.presets.positions, preset_pos)).pos
                ry_sp = (getattr(self.ry.presets.positions, preset_pos)).pos
                rz_sp = (getattr(self.rz.presets.positions, preset_pos)).pos
            else:
                logger.error('One of the axes is missing the desired state.')

        x_sp = self.x.readback.get() if x_sp is None else x_sp
        y_sp = self.y.readback.get() if y_sp is None else y_sp
        z_sp = self.z.readback.get() if z_sp is None else z_sp
        rx_sp = self.rx.readback.get() if rx_sp is None else rx_sp
        ry_sp = self.ry.readback.get() if ry_sp is None else ry_sp
        rz_sp = self.rz.readback.get() if rz_sp is None else rz_sp

        x_status = self.x.move(x_sp, wait=False, timeout=timeout,
                               sync_enable=False)
        y_status = self.y.move(y_sp, wait=False, timeout=timeout,
                               sync_enable=False)
        z_status = self.z.move(z_sp, wait=False, timeout=timeout,
                               sync_enable=False)
        rx_status = self.rx.move(rx_sp, wait=False, timeout=timeout,
                                 sync_enable=False)
        ry_status = self.ry.move(ry_sp, wait=False, timeout=timeout,
                                 sync_enable=False)
        rz_status = self.rz.move(rz_sp, wait=False, timeout=timeout,
                                 sync_enable=False)

        status = x_status & y_status & z_status & rx_status & \
            ry_status & rz_status
        if wait:
            status_wait(status)

        return status

    def stop(self):
        """stop command for all axes."""
        self.stop_signal.put(self.stop_value)

    def preset_list(self):
        """return a list of preset positions."""
        preset_list = []
        axis_list = [self.x, self.y, self.z, self.rz, self.ry, self.rz]
        for axis in axis_list:
            for preset in list(vars(axis.presets.positions).keys()):
                if preset not in preset_list:
                    preset_list.append(preset)
        return preset_list
