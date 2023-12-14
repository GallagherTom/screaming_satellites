#!/usr/bin/python3
import numpy as np

import matplotlib
matplotlib.use( 'tkagg' )

from matplotlib import pyplot as plt
from matplotlib import mlab

from scipy import signal
from scipy.signal import butter, lfilter
import peakutils

plt.figure(figsize=(14,5))

#
# Filter creation functions taken from https://stackoverflow.com/a/12233959
#
def butter_bandstop(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='bandstop')
    return b, a

def butter_bandstop_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandstop(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def butter_highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def find_starts(config, data, target_path, index, triage_mode):
    """
    Find the starts of interesting activity in the signal.

    The result is a list of indices where interesting activity begins, as well
    as the trigger signal and its average.
    """
    
    trigger = butter_bandpass_filter(
        data, config.bandpass_lower, config.bandpass_upper,
        config.sampling_rate, 6)
    

    #Added for Triage Mode
    if(triage_mode):
        plt.plot(trigger)
        plt.savefig(target_path+"/"+str(index)+"_4-trigger-bandpass.png")
        plt.clf()

        
    trigger = np.absolute(trigger)
    trigger = butter_lowpass_filter(
        trigger, config.lowpass_freq,config.sampling_rate, 6)

    #Added for Triage Mode
    if(triage_mode):
        plt.plot(trigger)
        plt.savefig(target_path+"/"+str(index)+"_5-trigger-lowpass.png")
        plt.clf()


    # transient = 0.0005
    # start_idx = int(transient * config.sampling_rate)
    start_idx = 0
    average = np.average(trigger[start_idx:])
    maximum = np.max(trigger[start_idx:])
    minimum = np.min(trigger[start_idx:])
    middle = (np.max(trigger[start_idx:]) - min(trigger[start_idx:])) / 2
    if average < 1.1*middle:
        print("")
        print("Adjusting average to avg + (max - avg) / 2")
        average = average + (maximum - average) / 2
    offset = -int(config.trigger_offset * config.sampling_rate)

    if config.trigger_rising:
        trigger_fn = lambda x, y: x > y
    else:
        trigger_fn = lambda x, y: x < y

    # The cryptic numpy code below is equivalent to looping over the signal and
    # recording the indices where the trigger crosses the average value in the
    # direction specified by config.trigger_rising. It is faster than a Python
    # loop by a factor of ~1000, so we trade readability for speed.
    trigger_signal = trigger_fn(trigger, average)[start_idx:]
    starts = np.where((trigger_signal[1:] != trigger_signal[:-1])
                      * trigger_signal[1:])[0] + start_idx + offset + 1
    if trigger_signal[0]:
        starts = np.insert(starts, 0, start_idx + offset)

    #Added for Triage Mode
    if(triage_mode):
        plt.plot(trigger_signal)
        plt.savefig(target_path+"/"+str(index)+"_6-triggerstart.png")
        plt.clf()

    #Added for Triage Mode
    if(triage_mode):
        plt.plot(data)
        plt.plot(trigger*100)
        plt.axhline(y=average*100)
        plt.savefig(target_path+"/"+str(index)+"_7-data_w_trigger.png")
        plt.clf()

    return starts, trigger, average

# The part that uses a frequency component as trigger was initially
# inspired by https://github.com/bolek42/rsa-sdr
# The code below contains a few hacks to deal with all possible errors we
# encountered with different radios and setups. It is not very clean but it is
# quite stable.
def extract(capture_file, config, average_file_name=None, plot=False, target_path=None, savePlot=False, index=0, triage_mode=False):
    """
    Post-process a GNUradio capture to get a clean and well-aligned trace.

    The configuration is a reproduce.AnalysisConfig tuple. The optional
    average_file_name specifies the file to write the average to (i.e. the
    template candidate).
    """
    try:
        
        with open(capture_file) as f:
            data = np.fromfile(f, dtype=np.complex64)

        # assert len(data) != 0, "ERROR, empty data just after measuring"
        if len(data) == 0:
            print("Warning! empty data, replacing with zeros")
            template = np.load(config.template_name)
            return np.zeros(len(template))
    
        #Added for Triage Mode
        if(triage_mode):
            plt.plot(data)
            plt.savefig(target_path+"/"+str(index)+"_1-data.png")
            plt.clf()
        

        template = np.load(config.template_name) if config.template_name else None
        
        if template is not None and len(template) != int(
                config.signal_length * config.sampling_rate):
            print("WARNING: Template length doesn't match collection parameters. "
                  "Is this the right template?")

        # cut usless transients
        data = data[int(config.drop_start * config.sampling_rate):]
        data = data[0:max(0,len(data)-int(config.drop_end * config.sampling_rate))]

        #Added for Triage Mode
        if(triage_mode):
            plt.plot(data)
            plt.savefig(target_path+"/"+str(index)+"_2-data-trimmed.png")
            plt.clf()


        # assert len(data) != 0, "ERROR, empty data after drop_start"
        if len(data) == 0:
           print("Warning! empty data after drop start, replacing with zeros")
           template = np.load(config.template_name)
           return np.zeros(len(template))

   
        # polar discriminator
        # fdemod = data[1:] * np.conj(data[:-1])
        # fdemod = np.angle(fdemod)
        # plt.plot(fdemod)
        # plt.show()
        # return fdemod
        # data = fdemod

        data = np.absolute(data)

        #Added for Triage Mode
        if(triage_mode):
            plt.plot(data)
            plt.savefig(target_path+"/"+str(index)+"_3-data-absolute.png")
            plt.clf()


        #
        # extract/aling trace with trigger frequency + autocorrelation
        #
        trace_starts, trigger, trigger_avg = find_starts(config, data, target_path, index, triage_mode)
        
        # extract at trigger + autocorrelate with the first to align
        traces = []
        trace_length = int(config.signal_length * config.sampling_rate)
        for start in trace_starts:
            if len(traces) >= config.num_traces_per_point:
                break

            stop = start + trace_length

            if stop > len(data):
                break

            trace = data[start:stop]
            if template is None or len(template) == 0:
                template = trace
                continue

            trace_lpf = butter_lowpass_filter(trace, config.sampling_rate / 4,
                    config.sampling_rate)
            template_lpf = butter_lowpass_filter(template, config.sampling_rate / 4,
                    config.sampling_rate)
            correlation = signal.correlate(trace_lpf**2, template_lpf**2)
            # print max(correlation)
            if max(correlation) <= config.min_correlation:
                continue

            shift = np.argmax(correlation) - (len(template)-1)
            traces.append(data[start+shift:stop+shift])

        # Reject outliers for each point in time.
        #
        # We reject a fixed number; while basing the decision on the standard
        # deviation would be nicer, we can't implement it with efficient numpy
        # operations: discarding a variable number of elements per column would not
        # yield a proper matrix again, so we'd need a Python loop...
        #
        # It should be enough to discard high values and keep everything on the low
        # side because interference always increases the energy (assuming that our
        # alignment is correct).
        # histo, bins = np.histogram(traces, bins=50)
        # arg = np.array(peakutils.indexes(histo, thres=0))
        # binmax = bins[arg]
        # maxima = histo[arg]
        # mode = binmax[np.argmax(maxima)]
        # reject_low = np.min(traces)
        # reject_high = np.max(traces)
        # if len(maxima) > 1:
            # low = min(binmax)
            # # high = max(binmax)
            # if low < mode:
                # reject_low = (mode - low) / 2.2
            # # if high > mode:
                # # reject_high = (high - mode) / 2
        # print binmax
        # plt.plot(binmax, maxima, '*')
        # plt.plot(bins[:-1], histo);
        # plt.show()
        # plt.plot(np.asarray(traces).T, '*')
        # plt.axhline(y=mode)
        # plt.axhline(y=reject_low)
        # plt.axhline(y=reject_high)
        # plt.show()

        # num_reject = int(0.05 * len(traces))
        # points = np.asarray(traces).T
        # points.sort()
        # avg = points[:, num_reject:(len(traces) - num_reject)].mean(axis=1)
        avg = np.average(traces, axis=0)

        if np.shape(avg) == ():
            return np.zeros(len(template))

        if average_file_name:
            np.save(average_file_name, avg)

        if plot or savePlot:
            plot_results(config, data, trigger, trigger_avg, trace_starts, traces, target_path, plot, savePlot)

        std = np.std(traces,axis=0)

        print("Extracted ")
        print("Number = ",len(traces))
        print("avg[Max(std)] = %.2E"%avg[std.argmax()])
        print("Max(u) = Max(std) = %.2E"%(max(std)))
        print("Max(u_rel) = %.2E"%(100*max(std)/avg[std.argmax()]),"%")

        # plt.plot(avg, 'r')
        # plt.plot(template, 'b')
        # plt.show()

        if config.keep_all:
            return traces
        else:
            return avg

    except Exception as inst:
        print(inst)
        print("Error, returning zeros")
        template = np.load(config.template_name)
        return np.zeros(len(template))

def plot_results(config, data, trigger, trigger_average, starts, traces, target_path=None, plot=True, savePlot=False):
    
    
    plt.subplots_adjust(hspace = 0.6) 
    plt.subplot(5, 1, 1)
    plt.plot(data)
    plt.title("Time domain capture")
    plt.xlabel("time [s]")
    plt.ylabel("normalized amplitude")
    
    print(len(data))
    print(data.shape)
    
    for i in range(5):
        plt.subplot(5, 1, i+1)
        plt.plot(data[(i*50000):((i+1)*50000)])
        plt.title("Time domain capture")
        plt.xlabel("time [s]")
        plt.ylabel("normalized amplitude")
    
    plt.show()
    plt.clf()
    
    for i in range(5):
        plt.subplot(5, 1, i+1)
        i = i+5
        plt.plot(data[(i*50000):((i+1)*50000)])
        plt.title("Time domain capture")
        plt.xlabel("time [s]")
        plt.ylabel("normalized amplitude")
    
    plt.show()
    plt.clf()
    
    plt.subplots_adjust(hspace = 0.6) 
    plt.subplot(4, 1, 1)

    t = np.linspace(0,len(data) / config.sampling_rate, len(data))
    plt.plot(t, data)
    plt.title("Time domain capture")
    plt.xlabel("time [s]")
    plt.ylabel("normalized amplitude")
   
    plt.plot(t, trigger*100)
    plt.axhline(y=trigger_average*100, color='y')
    trace_length = int(config.signal_length * config.sampling_rate)
    for start in starts:
        stop = start + trace_length
        plt.axvline(x=start / config.sampling_rate, color='r', linestyle='--')
        plt.axvline(x=stop / config.sampling_rate, color='g', linestyle='--')

    plt.subplot(4, 1, 2)
    #np.set_printoptions(threshold=np.inf)
    #print(data)
    
    plt.specgram(
        data, NFFT=128, Fs=config.sampling_rate, Fc=0, detrend=mlab.detrend_none,
        window=mlab.window_hanning, noverlap=127, cmap=None, xextent=None,
        pad_to=None, sides='default', scale_by_freq=None, mode='default',
        scale='default')
    plt.title("Spectrogram")
    plt.xlabel("time [s]")
    plt.ylabel("frequency [Hz]")

    # plt.subplot(4, 1, 3)
    # plt.psd(
        # data, NFFT=1024, Fs=config.sampling_rate, Fc=0, detrend=mlab.detrend_none,
        # window=mlab.window_hanning, noverlap=0, pad_to=None,
        # sides='default', scale_by_freq=None, return_line=None)

    if(len(traces) == 0):
        print("WARNING: no encryption was extracted")
    else:
        t = np.linspace(0,len(traces[0]) / config.sampling_rate, len(traces[0]))
        plt.subplot(4, 1, 3)
        for trace in traces:
            plt.plot(t, trace / max(trace))
        plt.title("%d aligned traces" % config.num_traces_per_point)
        plt.xlabel("time [s]")
        plt.ylabel("normalized amplitude")

        plt.subplot(4,1,4)
        avg = np.average(traces, axis=0)
        plt.plot(t, avg / max(avg))
        plt.title("Average of %d traces" % config.num_traces_per_point)
        plt.xlabel("time [s]")
        plt.ylabel("normalized amplitude")

    if savePlot and target_path != None:
        plt.savefig(target_path + "/plot.png")
    if plot:
        plt.show()

if __name__ == "__main__":
    extract(True)
