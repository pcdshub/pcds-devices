from ophyd import Component as Cpt
from ophyd import EpicsSignal
from ophyd import FormattedComponent as FCpt
from ophyd.signal import AttributeSignal

from pcdsdevices.inout import InOutPositioner


class LaserShutter(InOutPositioner):
    """Controls shutter controlled by Analog Output"""
    # EpicsSignals
    voltage = Cpt(EpicsSignal, '')
    state = FCpt(AttributeSignal, 'voltage_check')

    # Constants
    out_voltage = 4.5
    in_voltage = 0.0
    barrier_voltage = 1.4

    tab_whitelist = ['open', 'close']

    @property
    def voltage_check(self):
        """Return the position we believe shutter based on the channel"""
        if self.voltage.get() >= self.barrier_voltage:
            return 'OUT'
        else:
            return 'IN'

    def _do_move(self, state):
        """Override to just put to the channel"""
        if state.name == 'IN':
            self.voltage.put(self.in_voltage)
        elif state.name == 'OUT':
            self.voltage.put(self.out_voltage)
        else:
            raise ValueError("%s is in an invalid state", state)

    def open(self):
        self.remove()
        return

    def close(self):
        self.insert()
        return


# #############for flipper mirror###################
class LaserFlipper(InOutPositioner):
    """Controls shutter controlled by Analog Output"""
    # EpicsSignals
    voltage = Cpt(EpicsSignal, '')
    state = FCpt(AttributeSignal, 'voltage_check')

    # Constants
    out_voltage = -5.0
    in_voltage = 0.0
    barrier_voltage = -1.0

    tab_whitelist = ['open', 'close']

    @property
    def voltage_check(self):
        """Return the position we believe shutter based on the channel"""
        if self.voltage.get() <= self.barrier_voltage:
            # print(self.voltage.get())
            return 'OUT'
        else:
            return 'IN'

    def _do_move(self, state):
        """Override to just put to the channel"""
        if state.name == 'IN':
            self.voltage.put(self.in_voltage)
        elif state.name == 'OUT':
            self.voltage.put(self.out_voltage)
        else:
            raise ValueError("%s is in an invalid state", state)

    def open(self):
        self.remove()
        return

    def close(self):
        self.insert()
        return


class LaserShutterMPOD(InOutPositioner):
    """Controls shutter controlled by Mpod"""
    # EpicsSignals
    voltage = Cpt(EpicsSignal, ':GetVoltageMeasurement',
                  write_pv=':SetVoltage', kind='normal')
    switch = Cpt(EpicsSignal, ':GetSwitch', write_pv=':SetSwitch',
                 kind='normal')
    state = FCpt(AttributeSignal, 'voltage_check')

    # Constants
    set_voltage = 24.0
    out_voltage = 24.0
    in_voltage = 0.0  # 1.0
    barrier_voltage = 1.4

    @property
    def voltage_check(self):
        """Return the position we believe shutter based on the channel"""
        if abs(self.voltage.get_setpoint()-self.set_voltage) > 1:
            print('Set voltage not 24, fixing it')
            self.voltage.put(self.set_voltage)
        if self.switch.get() >= 1:
            return 'OUT'
        else:
            return 'IN'

    def _do_move(self, state):
        """Override to just put to the channel"""
        if state.name == 'IN':
            self.switch.put(0)
        elif state.name == 'OUT':
            self.switch.put(1)
        else:
            raise ValueError("%s is in an invalid state", state)
