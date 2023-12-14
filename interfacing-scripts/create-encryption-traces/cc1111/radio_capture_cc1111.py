#!/usr/bin/python3
import time
import os
from gnuradio import blocks, gr, uhd, iio
import osmosdr
import numpy as np


target_freq = 848.323e6 
sampling_rate = 5e6
usrp_gain = 40
hackrf_gain = 68
hackrf_gain_if = 26
hackrf_gain_bb = 35
plutosdr_gain = 40
time_to_record = 600 #seconds


timestr = time.strftime("%Y%m%d-%H%M%S")

output_dir = "test-"+timestr
os.system("mkdir "+output_dir)

OUTFILE = output_dir+"/trace_output_raw" + timestr + ".raw"



class GNUradio(gr.top_block):
    """GNUradio capture from SDR to file."""
    def __init__(self, frequency=2.464e9, sampling_rate=5e6, conventional=False,
                 usrp_gain=40, hackrf_gain=0, hackrf_gain_if=40, hackrf_gain_bb=44, plutosdr_gain=64):
        gr.top_block.__init__(self, "Top Block")

        Radio = "hackrf"
        if(Radio == "hackrf"):
            radio_block = osmosdr.source(args="numchan=1 "+Radio+"=0")
            radio_block.set_center_freq(frequency, 0)
            radio_block.set_sample_rate(sampling_rate)
            # TODO tune parameters
            radio_block.set_freq_corr(0, 0)
            radio_block.set_dc_offset_mode(True, 0)
            radio_block.set_iq_balance_mode(True, 0)
            radio_block.set_gain_mode(True, 0)
            radio_block.set_gain(hackrf_gain, 0)
            if conventional:
                # radio_block.set_if_gain(27, 0)
                # radio_block.set_bb_gain(30, 0)
                radio_block.set_if_gain(25, 0)
                radio_block.set_bb_gain(27, 0)
            else:
                radio_block.set_if_gain(hackrf_gain_if, 0)
                radio_block.set_bb_gain(hackrf_gain_bb, 0)
            radio_block.set_antenna('', 0)
            radio_block.set_bandwidth(1e6, 0)
        self._file_sink = blocks.file_sink(gr.sizeof_gr_complex, OUTFILE)
        print(radio_block)
        print(self._file_sink)
        self.connect((radio_block, 0), (self._file_sink, 0))



    def reset_trace(self):
        """
        Remove the current trace file and get ready for a new trace.
        """
        self._file_sink.open(OUTFILE)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()


gnuradio = GNUradio(target_freq,
            sampling_rate,
            False,
            usrp_gain,
            hackrf_gain,
            hackrf_gain_if,
            hackrf_gain_bb,
            plutosdr_gain)

gnuradio.start()
time.sleep(time_to_record)

gnuradio.stop()
gnuradio.wait()


try:
    with open(OUTFILE) as f:
        trace = np.fromfile(f, dtype=np.complex64)

except Exception as inst:
    print(inst)
                

np.save(output_dir+"/all_traces.npy",trace)

#Fix permissions on files created by root
os.system('chmod 0777 -R ./'+output_dir)
os.system('rm -rf ./'+OUTFILE)
                
                
