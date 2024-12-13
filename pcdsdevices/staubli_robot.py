import time

from ophyd import Component as Cpt
from ophyd import FormattedComponent as FCpt
from ophyd import EpicsSignal, EpicsSignalRO, Device
from ophyd import PVPositioner

from .interface import BaseInterface
from .pv_positioner import PVPositionerIsClose



class RobotPositioner(PVPositionerIsClose):

    setpoint = FCpt(EpicsSignal, '{self.prefix}:MOV{self._suffix}', kind='hinted')
    readback = FCpt(EpicsSignalRO, '{self.prefix}:POS{self._suffix}', kind='hinted')

    def __init__(self, prefix, *, name, ax_suffix=None, atol=0.1, rtol=0, **kwargs):
        self._suffix = ax_suffix
        self.atol = atol
        self.rtol = rtol

        super().__init__(prefix, name=name, **kwargs)



class CartesianCoord(Device):
    """ Control the robot in its cartesian coordinate system """

    x = FCpt(EpicsSignal, '{self.prefix}:ABS:X', kind='hinted')
    y = FCpt(EpicsSignal, '{self.prefix}:ABS:Y', kind='hinted')
    z = FCpt(EpicsSignal, '{self.prefix}:ABS:Z', kind='hinted')
    rx = FCpt(EpicsSignal, '{self.prefix}:ABS:RX', kind='hinted')
    ry = FCpt(EpicsSignal, '{self.prefix}:ABS:RY', kind='hinted')
    rz = FCpt(EpicsSignal, '{self.prefix}:ABS:RZ', kind='hinted')
    go = FCpt(EpicsSignal, '{self.prefix}:MOV:XYZRXRYRZ', kind='hinted')

    def __init__(self, prefix, *, name, **kwargs):
        super().__init__(prefix, name=name, **kwargs)
        self._all_axs = [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    def move(self, x, y, z, rx, ry, rz):
        for pos, coord in zip([x, y, z, rx, ry, rz], self._all_axs):
            print(f"{coord.name}: {pos}")
            coord.put(pos)
        time.sleep(0.1)
        self.go.put(1)



class SphericalCoord(Device):
    """ Control the robot in its spherical coordinate system """

    azi = FCpt(EpicsSignal, '{self.prefix}:ABS:AZI', kind='hinted')
    ele = FCpt(EpicsSignal, '{self.prefix}:ABS:ELE', kind='hinted')
    rad = FCpt(EpicsSignal, '{self.prefix}:ABS:RAD', kind='hinted')
    go = FCpt(EpicsSignal, '{self.prefix}:MOV:AZELRA', kind='hinted')

    def __init__(self, prefix, *, name, **kwargs):
        super().__init__(prefix, name=name, **kwargs)
        self._all_axs = [self.azi, self.ele, self.rad]


    def move(self, azi, ele, rad):
        for pos, coord in zip([azi, ele, rad], self._all_axs):
            print(f"{coord.name}: {pos}")
            coord.put(pos)
        time.sleep(0.1)
        self.go.put(1)



class InteractionPoint(BaseInterface, Device):
    x = Cpt(EpicsSignal, ':COORD:IP:X', write_pv=':COORD:IP:X:SET', kind='hinted')
    y = Cpt(EpicsSignal, ':COORD:IP:Y', write_pv=':COORD:IP:Y:SET', kind='hinted')
    z = Cpt(EpicsSignal, ':COORD:IP:Z', write_pv=':COORD:IP:Z:SET', kind='hinted')
    rx = Cpt(EpicsSignal, ':COORD:IP:RX', write_pv=':COORD:IP:RX:SET', kind='hinted')
    ry = Cpt(EpicsSignal, ':COORD:IP:RY', write_pv=':COORD:IP:RY:SET', kind='hinted')
    rz = Cpt(EpicsSignal, ':COORD:IP:RZ', write_pv=':COORD:IP:RZ:SET', kind='hinted')



class ToolCenterPoint(BaseInterface, Device):
    x = Cpt(EpicsSignal, ':COORD:TCP:X', write_pv=':COORD:TCP:X:SET', kind='hinted')
    y = Cpt(EpicsSignal, ':COORD:TCP:Y', write_pv=':COORD:TCP:Y:SET', kind='hinted')
    z = Cpt(EpicsSignal, ':COORD:TCP:Z', write_pv=':COORD:TCP:Z:SET', kind='hinted')
    rx = Cpt(EpicsSignal, ':COORD:TCP:RX', write_pv=':COORD:TCP:RX:SET', kind='hinted')
    ry = Cpt(EpicsSignal, ':COORD:TCP:RY', write_pv=':COORD:TCP:RY:SET', kind='hinted')
    rz = Cpt(EpicsSignal, ':COORD:TCP:RZ', write_pv=':COORD:TCP:RZ:SET', kind='hinted')



class StaubliRobot(BaseInterface, Device):
    x = Cpt(RobotPositioner, '', ax_suffix=':X', egu='mm', kind='hinted')
    y = Cpt(RobotPositioner, '', ax_suffix=':Y', egu='mm', kind='hinted')
    z = Cpt(RobotPositioner, '', ax_suffix=':Z', egu='mm', kind='hinted')
    rx = Cpt(RobotPositioner, '', ax_suffix=':RX', egu='deg', kind='hinted')
    ry = Cpt(RobotPositioner, '', ax_suffix=':RY', egu='deg', kind='hinted')
    rz = Cpt(RobotPositioner, '', ax_suffix=':RZ', egu='deg', kind='hinted')

    _stop = Cpt(EpicsSignal, ':ARM:STOP', kind='hinted')


    def __init__(self, prefix, *, name, **kwargs):
        super().__init__(prefix, name=name, **kwargs)
        self.cartesian = CartesianCoord(prefix, name='robot_cartesian')
        self.spherical = SphericalCoord(prefix, name='robot_spherical')

        self.ip = InteractionPoint(prefix, name='robot_ip')
        self.tcp = ToolCenterPoint(prefix, name='robot_tcp')


    def stop(self):
        self._stop.put(1)


    def rob_pixel_az_el(self, i, j, pix_size=0.075, origin=[0, 0, 100]):
        # get robot position and angles, build matrices and vectors
        x = self.x.get().readback
        y = self.y.get().readback
        z = self.z.get().readback
        xyz_rob = np.array([[x,y,z]]).T
        origin = np.asarray([origin]).T
        print(f"Robot position: {xyz_rob.T}")
        print(f"Origin: {origin.T}\n")

        aa = self.rx.get().readback
        ab = self.ry.get().readback
        ac = self.rz.get().readback
        Rx = np.array([ [1, 0, 0], [0, np.cos(aa), -np.sin(aa)], [0, np.sin(aa), np.cos(aa)] ])
        Ry = np.array([ [np.cos(ab), 0, np.sin(ab)], [0, 1, 0], [-np.sin(ab), 0, np.cos(ab)] ])
        Rz = np.array([ [np.cos(ac), -np.sin(ac), 0], [np.sin(ac), np.cos(ac), 0], [0, 0, 1]])

        # pos_xyz: in the robot frame
        pos_xyz = ( np.array([[-i, j, 0]]).T - origin ) * pix_size
        pos_xyz = Rx @ Ry @ Rz @ pos_xyz + xyz_rob - origin / pix_size

        print(f"pos_xyz: {pos_xyz.T}\n")

        el = np.rad2deg( -np.arctan(pos_xyz[2] / pos_xyz[1]) )[0]
        az = np.rad2deg( np.arctan(pos_xyz[0] / pos_xyz[1]) )[0]

        # calculate 2-theta
        u = np.cos(el) * np.sin(az)
        v = np.sin(el)
        tth  = np.arcsin( np.sqrt(u**2 + v**2) )
        tth = np.rad2deg(tth)
        print(f"2-theta = {tth}")
        return az, el, tth

