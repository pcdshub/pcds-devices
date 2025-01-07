from ophyd import Component as Cpt
from ophyd import Device, EpicsSignal
from ophyd import FormattedComponent as FCpt

from .interface import BaseInterface
from .variety import set_metadata

# Qadc == Abaco FMC126 Digitzier
# Qadc134 == Abaco FMC134 Digitizer


class QadcCommon(BaseInterface, Device):
    """
    Common class for Abaco FMC digitizers. Used in Qadc and Qadc134.
    """
    config = Cpt(EpicsSignal, ":CONFIG", kind="config",
                 doc="Write the current config to the QADC")
    set_metadata(config, dict(variety='command-proc', value=1))
    start = Cpt(EpicsSignal, ":START", kind="normal",
                doc="Start/stop acquisition")


class QadcLcls1Timing(BaseInterface, Device):
    """
    LCLS-I timing interface for FMC134 QADCs.
    """
    trig_event = Cpt(
        EpicsSignal, ":TRIG_EVENT_RBV", write_pv=":TRIG_EVENT", kind="config",
        doc="LCLS-I event code to trigger on"
    )


class QadcLcls2Timing(BaseInterface, Device):
    """
    LCLS-II timing interface for FMC134 QADCs.
    """
    trig_rate_mode = Cpt(EpicsSignal, ":TRIG_RATEMODE", kind="config",
                         doc="LCLS-II trigger mode")
    trig_fixed_rate = Cpt(EpicsSignal, ":TRIG_FIXEDRATE", kind="config",
                          doc="Rate to use for fixed trigger mode")
    trig_ac_rate = Cpt(EpicsSignal, ":TRIG_ACRATE", kind="config",
                       doc="Rate to use for AC trigger mode")
    trig_ts_mask = Cpt(EpicsSignal, ":TRIG_TSMASK", kind="config",
                       doc="Trigger mask")
    trig_seq_num = Cpt(EpicsSignal, ":TRIG_SEQNUM", kind="config",
                       doc="Trigger sequence number")
    trig_seq_bit = Cpt(EpicsSignal, ":TRIG_SEQBIT", kind="config",
                       doc="Trigger sequence bit")
    trig_partition = Cpt(EpicsSignal, ":TRIG_PARTITION", kind="config",
                         doc="Trigger partition number")


class Qadc134Waveform(BaseInterface, Device):
    """
    Common class for Qadc134 waveforms. Includes common settings and controls
    for these waveforms, like averaging and background correction.
    """
    enable_averaging = FCpt(
        EpicsSignal, "{self.prefix}:ENABLE_AVERAGE_{self.chan}",
        kind="config", doc="Enable signal averaging"
    )
    n_average = FCpt(
        EpicsSignal, "{self.prefix}:N_AVERAGE_{self.chan}", kind="config",
        doc="Number of waveforms to average"
    )
    average = FCpt(
        EpicsSignal, "{self.prefix}:AVERAGE_{self.chan}", kind="normal",
        doc="Averaged waveform"
    )

    bg_enable = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_BGCORR", kind="config",
        doc="Enable bacground correction"
    )
    bg_start = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_BGSTART", kind="config",
        doc="Background start (64 sample increments)"
    )
    bg_end = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_BGEND", kind="config",
        doc="Background end (64 sample increments)"
    )

    cfd_enable = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_CFDEN", kind="config",
        doc="Enable CFD edge finding algorithm"
    )
    cfd_delay = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_CFDEL", kind="config",
        doc="CFD delay, in pixels"
    )
    cfd_threshold = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_CFDTH", kind="config",
        doc="CFD threshold, in volts"
    )
    cfd_edges = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_CFD_EDGES", kind="normal",
        doc="Array of up to 10 detected edges (in ns)"
    )
    cfd_edges = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_CFD_INDICES", kind="normal",
        doc="Array of up to 10 detected edge indices (in pixel)"
    )
    cfd_edge_1 = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_EDGE1", kind="normal",
        doc="First detected edge (cfd_edges[0])"
    )
    cfd_edge_2 = FCpt(
        EpicsSignal, "{self.prefix}:{self.chan}_EDGE2", kind="normal",
        doc="First detected edge (cfd_edges[1])"
    )

    def __init__(self, prefix, chan=None, **kwargs):
        self.chan = chan
        super().__init__(prefix, **kwargs)


class Qadc134Common(QadcCommon):
    """
    Common class for FMC134 digitizers.
    """
    ch0 = Cpt(Qadc134Waveform, '', chan="CH0", kind="normal",
              doc="Input 0 of the ADC")
    ch1 = Cpt(Qadc134Waveform, '', chan="CH1", kind="normal",
              doc="Input 1 of the ADC")
    ch2 = Cpt(Qadc134Waveform, '', chan="CH2", kind="normal",
              doc="Input 2 of the ADC")
    ch3 = Cpt(Qadc134Waveform, '', chan="CH3", kind="normal",
              doc="Input 3 of the ADC")
    trig_delay = Cpt(
        EpicsSignal, ":TRIG_DELAY_RBV", write_pv=":TRIG_DELAY", kind="config",
        doc="Trigger delay in EVR/TPR ticks"
    )
    full_en = Cpt(EpicsSignal, ":FULL_EN_RBV", write_pv=":FULL_EN", kind="config",
                  doc="Enable full stream")
    hi_thresh = Cpt(EpicsSignal, ":HI_THRESH_RBV", write_pv=":HI_THRESH", kind="config",
                    doc="High threshold, in Volts")
    hi_thresh_raw = Cpt(
        EpicsSignal, ":HI_THRESH_RAW_RBV", write_pv=":HI_THRESH_RAW", kind="config",
        doc="High threshold, raw"
    )
    ichan = Cpt(EpicsSignal, ":ICHAN_RBV", write_pv=":ICHAN", kind="config",
                doc="Channel to interleave on")
    interleave = Cpt(
        EpicsSignal, ":INTERLEAVE_RBV", write_pv=":INTERLEAVE", kind="config",
        doc="Interleave enabled?"
    )
    length = Cpt(EpicsSignal, ":LENGTH_RBV", write_pv=":LENGTH", kind="config",
                 doc="Waveform length")
    lo_thresh = Cpt(EpicsSignal, ":LO_THRESH_RBV", write_pv=":LO_THRESH",
                    kind="config", doc="Low threshold, in Volts")
    lo_thresh_raw = Cpt(
        EpicsSignal, ":LO_THRESH_RAW_RBV", write_pv=":LO_THRESH_RAW",
        kind="config", doc="Low threshold, raw")
    prescale = Cpt(EpicsSignal, ":PRESCALE_RBV", write_pv=":PRESCALE",
                   kind="config", doc="Trigger prescale divider")
    rows_after = Cpt(
        EpicsSignal, ":ROWS_AFTER_RBV", write_pv=":ROWS_AFTER", kind="config"
    )
    rows_before = Cpt(
        EpicsSignal, ":ROWS_BEFORE_RBV", write_pv=":ROWS_BEFORE", kind="config"
    )
    sparse_en = Cpt(EpicsSignal, ":SPARSE_EN_RBV", write_pv=":SPARSE_EN",
                    kind="config", doc="Enable sparsified mode")
    clear_config = Cpt(EpicsSignal, ":CLEAR_CONFIG", kind="config",
                       doc="Clear the current configuration")
    set_metadata(clear_config, dict(variety='command-proc', value=1))
    out0 = Cpt(Qadc134Waveform, '', chan="OUT0", kind="normal",
               doc="Interleaved waveform 0 of the ADC")
    out1 = Cpt(Qadc134Waveform, '', chan="OUT1", kind="normal",
               doc="Interleaved waveform 1 of the ADC")
    rawdata0 = Cpt(EpicsSignal, ":RAWDATA0", kind="normal",
                   doc="Raw output zero")
    rawdata1 = Cpt(EpicsSignal, ":RAWDATA1", kind="normal",
                   doc="Raw output one")
    sparse0 = Cpt(Qadc134Waveform, '', chan="SPARSE0", kind="normal",
                  doc="Sparsified waveform 0 of the ADC")
    sparse1 = Cpt(Qadc134Waveform, '', chan="SPARSE1", kind="normal",
                  doc="Sparsified waveform 1 of the ADC")


class Qadc134(Qadc134Common, QadcLcls1Timing):
    """
    Class for LCLS-I FMC134 digitizers. Implements LCLS-I timing interface.
    """


class Qadc134Lcls2(Qadc134Common, QadcLcls2Timing):
    """
    Class for LCLS-2 FMC134 digitizers. Implements LCLS-2 timing interface.
    """


class Qadc(QadcCommon):
    """
    Class for an older Abaco FMC PCIe digitzer, used in the LCLS-I hutches.
    """
    ch0 = Cpt(EpicsSignal, ":CH0", kind="normal", doc="Input 0 of the ADC")
    ch1 = Cpt(EpicsSignal, ":CH1", kind="normal", doc="Input 1 of the ADC")
    ch2 = Cpt(EpicsSignal, ":CH2", kind="normal", doc="Input 2 of the ADC")
    ch3 = Cpt(EpicsSignal, ":CH3", kind="normal", doc="Input 3 of the ADC")

    gain0_i = Cpt(EpicsSignal, ":GAIN0_I", kind="config")
    gain0_ni = Cpt(EpicsSignal, ":GAIN0_NI", kind="config")
    gain1_i = Cpt(EpicsSignal, ":GAIN1_I", kind="config")
    gain1_ni = Cpt(EpicsSignal, ":GAIN1_NI", kind="config")
    gain2_i = Cpt(EpicsSignal, ":GAIN2_I", kind="config")
    gain2_ni = Cpt(EpicsSignal, ":GAIN2_NI", kind="config")
    gain3_i = Cpt(EpicsSignal, ":GAIN3_I", kind="config")
    gain3_ni = Cpt(EpicsSignal, ":GAIN3_NI", kind="config")
    ichan = Cpt(EpicsSignal, ":ICHAN", kind="config")
    interleave = Cpt(EpicsSignal, ":INTERLEAVE", kind="config")
    length = Cpt(EpicsSignal, ":LENGTH", kind="config")
    off0_i = Cpt(EpicsSignal, ":OFF0_I", kind="config")
    off0_ni = Cpt(EpicsSignal, ":OFF0_NI", kind="config")
    off1_i = Cpt(EpicsSignal, ":OFF1_I", kind="config")
    off1_ni = Cpt(EpicsSignal, ":OFF1_NI", kind="config")
    off2_i = Cpt(EpicsSignal, ":OFF2_I", kind="config")
    off2_ni = Cpt(EpicsSignal, ":OFF2_NI", kind="config")
    off3_i = Cpt(EpicsSignal, ":OFF3_I", kind="config")
    off3_ni = Cpt(EpicsSignal, ":OFF3_NI", kind="config")
    out = Cpt(EpicsSignal, ":OUT", kind="normal")
    rawdata = Cpt(EpicsSignal, ":RAWDATA", kind="normal")
    train = Cpt(EpicsSignal, ":TRAIN", kind="config")
    trig_delay = Cpt(EpicsSignal, ":TRIG_DELAY", kind="config")
    trig_event = Cpt(EpicsSignal, ":TRIG_EVENT", kind="config")
