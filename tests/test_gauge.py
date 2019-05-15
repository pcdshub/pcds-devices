import logging
import pytest

from ophyd.sim import make_fake_device
from pcdsdevices.gauge import GaugeSet, GaugeSetPirani, GaugeSetBase
from pcdsdevices.gauge import GaugeSetMks, GaugeSetPiraniMks

logger = logging.getLogger(__name__)


@pytest.fixture(scope='function')
def fake_gauge_set():
    FakeGaugeSet = make_fake_device(GaugeSetPirani)
    gs = FakeGaugeSet('Test:Gauge', name='test', index='99')
    gs.gcc.state.sim_put(1)
    gs.gcc.pressure.sim_put(99)
    gs.gpi.pressure.sim_put(98)
    return gs


def test_gauge_pressure(fake_gauge_set):
    logger.debug('test_gauge_pressure')
    gs = fake_gauge_set
    # Should return to original position on unstage
    assert gs.pressure() == 98
    gs.gcc.state.sim_put(0)
    assert gs.pressure() == 99


def test_gauge_factory():
    m = GaugeSet('TST:MY', name='test_gauge', index='99')
    assert isinstance(m, GaugeSetPirani)
    m = GaugeSet('TST:MY', name='test_gauge', index='99', onlyGCC=1)
    assert isinstance(m, GaugeSetBase)
    m = GaugeSet('TST:MY', name='test_gauge', index='99',
                 prefix_controller='TST:R99:GCT:99:A')
    assert isinstance(m, GaugeSetPiraniMks)
    m = GaugeSet('TST:MY', name='test_gauge', index='99',
                 prefix_controller='TST:R99:GCT:99:A', onlyGCC=True)
    assert isinstance(m, GaugeSetMks)
