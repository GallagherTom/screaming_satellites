#!/usr/bin/python3


import click
import collections
import enum
import json
import os
from os import path, system
import serial
import sys
import time
import logging
from Crypto.Cipher import AES
import zmq
import subprocess

from gnuradio import blocks, gr, uhd, iio
import osmosdr

import numpy as np

from matplotlib import pyplot as plt
from matplotlib import mlab

from scipy import signal
from scipy.signal import butter, lfilter
import peakutils


from . import analyze

logging.basicConfig()
l = logging.getLogger('reproduce')

Radio = enum.Enum("Radio", "USRP USRP_mini USRP_B210 USRP_B210_MIMO HackRF bladeRF PlutoSDR")

FirmwareMode = collections.namedtuple(
    "FirmwareMode",
    [
        "mode_command",         # command for entering the test mode
        "repetition_command",   # command for triggering repeated execution (or None)
        "action_command",       # command for starting (single or repeated) action
        "have_keys",            # whether the test mode works with keys
    ])


# These are the firmware modes we support; they can be selected with the "mode"
# key in the "firmware" section of the config file.
TINY_AES_MODE = FirmwareMode(
    have_keys=True, mode_command='n', repetition_command='n', action_command='r')
HW_CRYPTO_MODE = FirmwareMode(
    have_keys=True, mode_command='u', repetition_command='n', action_command='r')
HW_CRYPTO_KEYGEN_MODE = FirmwareMode(
    have_keys=True, mode_command='u', repetition_command='n', action_command='r')
HW_CRYPTO_ECB_MODE = FirmwareMode(
    have_keys=True, mode_command='U', repetition_command='n', action_command='r')
MASK_AES_MODE = FirmwareMode(
    have_keys=True, mode_command='w', repetition_command='n', action_command='r')
MASK_AES_MODE_SLOW = FirmwareMode(
    have_keys=True, mode_command='w', repetition_command=None, action_command='e')
TINY_AES_MODE_SLOW = FirmwareMode(
    have_keys=True, mode_command='n', repetition_command=None, action_command='e')
HW_CRYPTO_MODE_SLOW = FirmwareMode(
    have_keys=True, mode_command='u', repetition_command=None, action_command='e')
POWER_ANALYSIS_MODE = FirmwareMode(
    have_keys=False, mode_command='v', repetition_command=None, action_command='s')

# The config file's "firmware" section.
FirmwareConfig = collections.namedtuple(
    "FirmwareConfig",
    [
        # Algorithm to attack: tinyaes[_slow], hwcrypto[_slow], power; slow
        # modes use individual serial commands to trigger.
        "mode",
        # True to use a fixed key or False to vary it for each point.
        "fixed_key",
        # True to use a fixed plaintext or False to vary it for each point.
        "fixed_plaintext",
        # Fixed vs Fixed mode: alternate between two fixed p,k pairs
        # which show large distance according to the leak model
        "fixed_vs_fixed",
        # True to modulate data or False to use just the carrier.
        "modulate",

        # Mode-specific options

        # True to disable radio (conventional attack mode). Defaults at false
        # for compatibility
        "conventional",
        # If a masked version of AES is used, this decides which mode
        "mask_mode",
        # The sleep time between individual encryptions in slow mode collections
        "slow_mode_sleep_time",
    ])


# The config file's "collection" section.
CollectionConfig = collections.namedtuple(
    "CollectionConfig",
    [
        # Frequency to tune to, in Hz.
        "target_freq",
        # Sampling rate, in Hz.
        "sampling_rate",
        # How many different plaintext/key combinations to record.
        "num_points",
        # How many traces to use.
        "num_traces_per_point",
        # Multiplier to account for traces dropped due to signal processing
        "traces_per_point_multiplier",
        # Lower cut-off frequency of the band-pass filter.
        "bandpass_lower",
        # Upper cut-off frequency of the band-pass filter.
        "bandpass_upper",
        # Cut-off frequency of the low-pass filter.
        "lowpass_freq",
        # How much to drop at the start of the trace, in seconds.
        "drop_start",
        # How much to drop at the end of the trace, in seconds.
        "drop_end",
        # How much to include before the trigger, in seconds.
        "trigger_offset",
        # True for triggering on a rising edge, False otherwise.
        "trigger_rising",
        # Length of the signal portion to keep, in seconds, starting at
        # trigger - trigger_offset.
        "signal_length",
        # Name of the template to load, or None.
        "template_name",
        # Traces with a lower correlation will be discarded.
        "min_correlation",
        # Gain.
        "hackrf_gain",
        # Gain BB.
        "hackrf_gain_bb",
        # Gain IF.
        "hackrf_gain_if",
        # Gain.
        "plutosdr_gain",
        # Gain
        "usrp_gain",
        # Keep all raw
        "keep_all",
        # Channel
        "channel",
        # Trigger a frequency spike 
        "use_spike_of_target_freq",
        # broadcast_on_freq
        "broadcast_on_freq"
    ])

# Global settings, for simplicity
DEVICE = None
BAUD = None
OUTFILE = None
RADIO = None
RADIO_ADDRESS = None
COMMUNICATE_SLOW = None
YKUSH_PORT = None


class EnumType(click.Choice):
    """Teach click how to handle enums."""
    def __init__(self, enumcls):
        self._enumcls = enumcls
        click.Choice.__init__(self, enumcls.__members__)

    def convert(self, value, param, ctx):
        value = click.Choice.convert(self, value, param, ctx)
        return self._enumcls[value]


@click.group()
@click.option("-d", "--device", default="/dev/ttyACM0", show_default=True,
              help="The serial dev path of device tested for screaming channels")
@click.option("-b", "--baudrate", default=115200, show_default=True,
              help="The baudrate of the serial device")
@click.option("-y", "--ykush-port", default=0, show_default=True,
              help="If set, use given ykush-port to power-cycle the device")
@click.option("-s", "--slowmode", is_flag=True, show_default=True,
              help=("Enables slow communication mode for targets with a small"
                    "serial rx-buffer"))
@click.option("-r", "--radio", default="USRP", type=EnumType(Radio), show_default=True,
              help="The type of SDR to use.")
@click.option("--radio-address", default="10.0.3.40",
              help="Address of the radio (X.X.X.X for USRP, ip:X.X.X.X or usb:X.X.X for PlutoSDR).")
@click.option("-l", "--loglevel", default="INFO", show_default=True,
              help="The loglevel to be used ([DEBUG|INFO|WARNING|ERROR|CRITICAL])")
@click.option("-o", "--outfile", default="/tmp/time", type=click.Path(), show_default=True,
              help="The file to write the GNUradio trace to.")
def cli(device, baudrate, ykush_port, slowmode, radio, radio_address,
        outfile, loglevel, **kwargs):
    """
    Reproduce screaming channel experiments with vulnerable devices.

    This script assumes that the device has just been plugged in (or is in an
    equivalent state), that it is running our modified firmware, and that an SDR
    is available. It will carry out the chosen experiment, producing a trace and
    possibly other artifacts. Make sure that the "--outfile" option points to a
    file that doesn't exist or can be safely overwritten.

    Call any experiment with "--help" for details. You most likely want to use
    "collect".
    """
    global DEVICE, OUTFILE, RADIO, RADIO_ADDRESS, BAUD, COMMUNICATE_SLOW, YKUSH_PORT
    DEVICE = device
    BAUD = baudrate
    OUTFILE = outfile
    RADIO = radio
    RADIO_ADDRESS = radio_address
    COMMUNICATE_SLOW = slowmode
    YKUSH_PORT = ykush_port

    l.setLevel(loglevel)
    
#


def _encode_for_device(data):
    """
    Encode the given bytes in our special format.
    """
    return " ".join(str(data_byte) for data_byte in data)


def _send_parameter(ser, command, param):
    """
    Send a parameter (key or plaintext) to the target device.

    The function assumes that we've already entered tiny_aes mode.
    """
    command_line = '%s%s\r\n' % (command, _encode_for_device(param))
    l.debug('Sending command:  %s\n' % command_line)
    if not COMMUNICATE_SLOW:
        ser.write(command_line.encode())
    else:
        for p in command_line.split(' '):
            ser.write((p+' ').encode())
            time.sleep(.05)

    l.debug('Waiting check\n')
    x = ser.readline()
    print ("received: "+x.decode())
    if len(x) == 0:
        print("nothing received on timeout, ignoring error")
        return False
    #check = ''.join(chr(int(word)) for word in x.split(' '))
    # -- create check like this instead for ESP32:
    #response = ser.readline()
    #response = [ a for a in x.decode().split(' ') if a.isdigit() ]
    #check = ''.join(chr(int(word)) for word in response)
    param2 = '%s' %  _encode_for_device(param)
    
    print ("param: "+param2)
    print ("check: "+x.decode())
    if x.decode().strip() != param2.strip():
        print(("ERROR\n%s\n%s" % (_encode_for_device(param),
                                 _encode_for_device(x))))
        ser.write(b'q')
        sys.exit(1)
    l.debug('Check done\n')
    return True

def _send_key(ser, key):
    return _send_parameter(ser, 'k', key)

def _send_plaintext(ser, plaintext):
    return _send_parameter(ser, 'p', plaintext)

def _send_init(ser, init):
    return _send_parameter(ser, 'i', init)
    
    


def _open_serial_port():
    l.debug("Opening serial port")
    return serial.Serial(DEVICE, BAUD, timeout=5)




@cli.command()
@click.argument("config", type=click.File())
@click.argument("target-path", type=click.Path(exists=True, file_okay=False))
@click.option("--name", default="",
              help="Identifier for the experiment (obsolete; only for compatibility).")
@click.option("--average-out", type=click.Path(dir_okay=False),
              help="File to write the average to (i.e. the template candidate).")
@click.option("--plot/--no-plot", default=False, show_default=True,
              help="Plot the results of trace collection.")
@click.option("--max-power/--no-max-power", default=False, show_default=True,
              help="Set the output power of the device to its maximum.")
@click.option("--raw/--no-raw", default=False, show_default=True,
              help="Save the raw IQ data.")
@click.option("--saveplot/--no-saveplot", default=False, show_default=True,
              help="Plot the results of trace collection.")
@click.option("-p", "--set-power", default=0, show_default=True,
              help="If set, sets the device to a specific power level (overrides --max-power)")
def triage(config, target_path, name, average_out, plot, max_power, raw, saveplot, set_power):
    """
    Triage a device to determine configuration parameters to isolate a leak.

    The config is a JSON file containing parameters for trace analysis; see the
    definitions of FirmwareConfig and CollectionConfig for descriptions of each
    parameter.
    """
    # NO-OP defaults for mode dependent config options for backwards compatibility
    cfg_dict = json.load(config)
    cfg_dict["firmware"].setdefault('conventional', False)
    cfg_dict["firmware"].setdefault('mask_mode', 0)
    cfg_dict["firmware"].setdefault('slow_mode_sleep_time', 0.001)
    cfg_dict["firmware"].setdefault('fixed_vs_fixed', False)
    cfg_dict["firmware"].setdefault('fixed_plaintext', False)
    cfg_dict["collection"].setdefault('traces_per_point_multiplier', 1.2)
    cfg_dict["collection"].setdefault('hackrf_gain', 0)
    cfg_dict["collection"].setdefault('hackrf_gain_bb', 44)
    cfg_dict["collection"].setdefault('hackrf_gain_if', 40)
    cfg_dict["collection"].setdefault('plutosdr_gain', 64)
    cfg_dict["collection"].setdefault('usrp_gain', 40)
    cfg_dict["collection"].setdefault('keep_all', False)
    cfg_dict["collection"].setdefault('channel', 0)

    collection_config = CollectionConfig(**cfg_dict["collection"])
    firmware_config = FirmwareConfig(**cfg_dict["firmware"])

    if firmware_config.mode == "tinyaes":
        firmware_mode = TINY_AES_MODE
    elif firmware_config.mode == "tinyaes_slow":
        firmware_mode = TINY_AES_MODE_SLOW
    elif firmware_config.mode == "maskaes":
        firmware_mode = MASK_AES_MODE
    elif firmware_config.mode == "maskaes_slow":
        firmware_mode = MASK_AES_MODE_SLOW
    elif firmware_config.mode == "hwcrypto":
        firmware_mode = HW_CRYPTO_MODE
    elif firmware_config.mode == "hwcrypto_keygen":
        firmware_mode = HW_CRYPTO_KEYGEN_MODE
    elif firmware_config.mode == "hwcrypto_ecb":
        firmware_mode = HW_CRYPTO_ECB_MODE
    elif firmware_config.mode == "hwcrypto_slow":
        firmware_mode = HW_CRYPTO_MODE_SLOW
    elif firmware_config.mode == "power":
        firmware_mode = POWER_ANALYSIS_MODE
    else:
        raise Exception("Unsupported mode %s; this is a bug!" % firmware_config.mode)

    # assert (not plot) or (collection_config.num_traces_per_point <= 500), \
        # "Plotting a lot of data might lock up the computer! Consider reducing " \
        # "num_traces_per_point in the configuration file or enforce limits on resource consumption..."

    # Signal post-processing will drop some traces when their quality is
    # insufficient, so let's collect more traces than requested to make sure
    # that we have enough in the end.
    num_traces_per_point = int(collection_config.num_traces_per_point * collection_config.traces_per_point_multiplier)

    failure_tracker = 0
    success_tracker = 0

    # number of points
    num_points = int(collection_config.num_points)

    # fixed vs fixed
    fixed_vs_fixed = firmware_config.fixed_vs_fixed

    # Generate the plaintexts
    if fixed_vs_fixed:
        plaintexts = [b'\x00'*16 for _trace in range(num_points)]
    elif firmware_config.fixed_plaintext:
        plaintexts = [b'\xa4'*16]
    else:
        plaintexts = [os.urandom(16)
                    for _trace in range(1 if firmware_config.fixed_plaintext else num_points)]
    
    with open(path.join(target_path, 'pt_%s.txt' % name), 'w') as f:
        f.write('\n'.join(p.hex() for p in plaintexts))

    # Generate the key(s)
    if firmware_mode.have_keys:
        if fixed_vs_fixed:
            keys = [b'\x00'*16 if i%2==0 else b'\x30'*16 for i in range(num_points)]
        elif firmware_config.fixed_key:
            keys = [b'\x18'*16]
        else:
            keys = [os.urandom(16)
                    for _key in range(1 if firmware_config.fixed_key else num_points)]
        with open(path.join(target_path, 'key_%s.txt' % name), 'w') as f:
            f.write('\n'.join(k.hex() for k in keys))

    # If requested, reset target
    if YKUSH_PORT != 0:
        l.debug('Resetting device using ykush port %d' % YKUSH_PORT)
        system("ykushcmd -d %d" % YKUSH_PORT)
        system("ykushcmd -u %d" % YKUSH_PORT)
        time.sleep(3)



    with _open_serial_port() as ser:
        if YKUSH_PORT != 0:
            print((ser.readline()))

        # tmp increase power
        #l.debug('POWERPOWER')
        #ser.write(b'p')
        #print ser.readline()
        #ser.write(b'0')
        #print ser.readline()
        if set_power != 0:
            l.debug('Setting power level to '+str(set_power))
            ser.write(('p'+str(set_power)).encode('UTF-8'))
            ser.readline()
            ser.readline()
        elif max_power:
            l.debug('Setting power to level 0')
            ser.write(b'p0')
            ser.readline()
            ser.readline()
            
        if firmware_config.conventional:
            l.debug('Starting conventional mode, the radio is off')
        else:
            if collection_config.channel == -1:
                l.debug('Selecting frequency')
                ser.write(b'k')
                print((ser.readline()))
                ser.write(b'%02d\n'%collection_config.broadcast_on_freq)
                print((ser.readline()))
            else:
                l.debug('Selecting channel')
                ser.write(b'a')
                print((ser.readline()))
                ser.write(b'%02d\n'%collection_config.channel)
                print((ser.readline()))
            if firmware_config.modulate:
                l.debug('Starting modulated wave')
                ser.write(b'o')     # start modulated wave
                print((ser.readline()))
            else:
                l.debug('Starting continuous wave')
                ser.write(b'c')     # start continuous wave
               
        if collection_config.use_spike_of_target_freq != -1:
            l.debug('Setting spike frequency')
            ser.write(b'K')
            print((ser.readline()))
            ser.write(b'%d\n'%(int(collection_config.target_freq/(1000*1000)))) #converting Hz to MHz
            print("SETTING SPIKE FREQUENCY")
            print(b'%d\n'%(int(collection_config.target_freq/(1000*1000))))
            print((ser.readline()))
            print((ser.readline())) #ADDED
        
        l.debug('Entering test mode')
        ser.write(firmware_mode.mode_command.encode()) # enter test mode
        print((ser.readline()))
        print((ser.readline())) #ADDED

        if firmware_mode.repetition_command:
            l.debug('Setting trace repitions')
            ser.write(('n%d\r\n' % num_traces_per_point).encode())
            print((ser.readline()))
            print((ser.readline())) #ADDED

        if firmware_mode.have_keys and firmware_config.fixed_key:
            # The key never changes, so we can just set it once and for all.
            if not _send_key(ser, keys[0]):
                failed = True

        if firmware_config.fixed_plaintext:
            # The plaintext never changes, so we can just set it once and for all.
            if not _send_plaintext(ser, plaintexts[0]):
                failed = True

        if firmware_config.mode == 'maskaes' or firmware_config.mode == 'maskaes_slow':
            l.debug('Setting masking mode to %d', firmware_config.mask_mode)
            ser.write(('%d\r\n' % firmware_config.mask_mode).encode())
            print((ser.readline()))


        l.debug('Starting GNUradio')
        gnuradio = GNUradio(collection_config.target_freq,
                            collection_config.sampling_rate,
                            firmware_config.conventional,
                            collection_config.usrp_gain,
                            collection_config.hackrf_gain,
                            collection_config.hackrf_gain_if,
                            collection_config.hackrf_gain_bb,
                            collection_config.plutosdr_gain)
        # with click.progressbar(plaintexts) as bar:
            # for index, plaintext in enumerate(bar):
        with click.progressbar(list(range(num_points))) as bar:
            # for index, plaintext in enumerate(bar):
            for index in bar:
                failed = False
                if firmware_mode.have_keys and not firmware_config.fixed_key:
                    if not _send_key(ser, keys[index]):
                        failed = True

                if not firmware_config.fixed_plaintext:
                    if firmware_config.mode == "hwcrypto_keygen":
                        if not _send_init(ser, plaintexts[index]):
                            failed = True
                    else:
                        if not _send_plaintext(ser, plaintexts[index]):
                            failed = True

                gnuradio.start()
                time.sleep(0.03)

                if RADIO == Radio.USRP_B210_MIMO or RADIO == Radio.USRP_B210:
                    time.sleep(0.08)
                    # time.sleep(0.04)

                if firmware_mode.repetition_command:
                    # The test mode supports repeated actions.
                    l.debug('Start repetitions')
                    ser.write(firmware_mode.action_command.encode())
                    ser.readline() # wait until done
                else:
                    for _iteration in range(num_traces_per_point):
                        time.sleep(firmware_config.slow_mode_sleep_time)
                        ser.write(firmware_mode.action_command.encode()) # single action

                time.sleep(0.09)
                gnuradio.stop()
                gnuradio.wait()

                #trace = analyze.extract2(OUTFILE, collection_config, average_out, plot, target_path, saveplot, index, True)
                
                try:
                    with open(OUTFILE) as f:
                        trace = np.fromfile(f, dtype=np.complex64)
                
                except Exception as inst:
                    print(inst)
                    
                if not failed:
                    savedata = trace[int(collection_config.drop_start * collection_config.sampling_rate):max(0,len(trace)-int(collection_config.drop_end * collection_config.sampling_rate))]
        
                    np.save(os.path.join(target_path,"trace_%s_%d.npy"%(name,index)),savedata)
                    if raw:
                        save_raw(OUTFILE, target_path, index, name)
                    success_tracker = success_tracker + 1
                else:
                    failure_tracker = failure_tracker + 1
                gnuradio.reset_trace()
                
                print("DEBUG: failure tracker ", str(failure_tracker))
                print("DEBUG: success tracker ", str(success_tracker))
                time.sleep(0.8) #hardware seems to fail after several thousands of iterations. Adding cooloff timer between cycles

        ser.write(b'q')     # quit tiny_aes mode
        print((ser.readline()))
        ser.write(b'e')     # turn off continuous wave
        
        time.sleep(1)
        ser.close()

    
    
class GNUradio(gr.top_block):
    """GNUradio capture from SDR to file."""
    def __init__(self, frequency=2.464e9, sampling_rate=5e6, conventional=False,
                 usrp_gain=40, hackrf_gain=0, hackrf_gain_if=40, hackrf_gain_bb=44, plutosdr_gain=64):
        gr.top_block.__init__(self, "Top Block")

        if RADIO in (Radio.USRP, Radio.USRP_mini, Radio.USRP_B210):
            radio_block = uhd.usrp_source(
                ("addr=" + RADIO_ADDRESS.encode("ascii"))
                if RADIO == Radio.USRP else "",
                uhd.stream_args(cpu_format="fc32", channels=[0]))
            radio_block.set_center_freq(frequency)
            radio_block.set_samp_rate(sampling_rate)
            radio_block.set_gain(usrp_gain)
            radio_block.set_antenna("TX/RX")
        elif RADIO == Radio.USRP_B210_MIMO:
            radio_block = uhd.usrp_source(
        	",".join(('', "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=list(range(2)),
        	),
            )
            radio_block.set_samp_rate(sampling_rate)
            radio_block.set_center_freq(frequency, 0)
            radio_block.set_gain(usrp_gain, 0)
            radio_block.set_antenna('RX2', 0)
            radio_block.set_bandwidth(sampling_rate/2, 0)
            radio_block.set_center_freq(frequency, 1)
            radio_block.set_gain(usrp_gain, 1)
            radio_block.set_antenna('RX2', 1)
            radio_block.set_bandwidth(sampling_rate/2, 1)
 
        elif RADIO == Radio.HackRF or RADIO == Radio.bladeRF:
            mysdr = str(RADIO).split(".")[1].lower() #get "bladerf" or "hackrf"
            radio_block = osmosdr.source(args="numchan=1 "+mysdr+"=0")
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
            radio_block.set_bandwidth(3e6, 0)
            
        elif RADIO == Radio.PlutoSDR:
            bandwidth = 3e6
            radio_block = iio.pluto_source(RADIO_ADDRESS.encode("ascii"),
                                           int(frequency), int(sampling_rate),
                                           1 - 1, int(bandwidth), 0x8000, True,
                                           True, True, "manual", plutosdr_gain,
                                           '', True)
        else:
            raise Exception("Radio type %s is not supported" % RADIO)


        self._file_sink = blocks.file_sink(gr.sizeof_gr_complex, OUTFILE)
        print(radio_block)
        print(self._file_sink)
        self.connect((radio_block, 0), (self._file_sink, 0))

        if RADIO == Radio.USRP_B210_MIMO:
            self._file_sink_2 = blocks.file_sink(gr.sizeof_gr_complex,
            OUTFILE+"_2")
            self.connect((radio_block, 1), (self._file_sink_2, 0))


    def reset_trace(self):
        """
        Remove the current trace file and get ready for a new trace.
        """
        self._file_sink.open(OUTFILE)
        
        if RADIO == Radio.USRP_B210_MIMO:
            self._file_sink_2.open(OUTFILE+"_2")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()


if __name__ == "__main__":
    cli()
