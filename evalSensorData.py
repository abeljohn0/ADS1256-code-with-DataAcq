# -*- encoding: utf-8 -*-

import multiprocessing

import matplotlib.animation
from pylab import *

# enable čšž etc.
import sys

from argparse import ArgumentParser, FileType
from configparser import ConfigParser
from confluent_kafka import Consumer, OFFSET_BEGINNING


# importlib.reload(sys)
# sys.getdefaultencoding()


# opener = urllib.request.build_opener(SMBHandler)
# fh = opener.open('smb://raspberrypi/ads1256datafile')
# data = fh.read()
# fh.close()

def list_of_arrays(file):
    """ Splits multiple data inputs into separate arrays"""
    # file = open(path, 'r')
    list = []
    for line in file:
        list.append(str_to_flts(line))

    # Find borders between different modes (SE_MUX / DIFF_MUX / SE_CONT / DIFF_CONT)
    newModeLine = [0]
    # for x in range(0, len(list) - 1):
    #     if list[x + 1][0] < list[x][0]:
    #         newModeLine.append(x + 1)
    newModeLine.append(len(list))

    # Create numpy arrays (one 2d array for each mode)
    arrays = []
    for x in range(0, len(newModeLine) - 1):
        arrays.append(np.array(list[newModeLine[x]: newModeLine[x + 1]]))
    # file.truncate(0)
    file.close()
    return arrays


def str_to_flts(string):
    """ Convert string of values to floats. """
    return [float(num) for num in string.split()]


def clean_data(array):
    """ Delete data that is completely wrong. """  # modify for more than one channel
    size = np.size(array)

    # calculate average:excluding values that are out of 0 - 5 V range
    values = array[array[:, 0] < 5]
    return array


def std_dev(array):
    """ Calculate standard deviation of a sample. """
    vals = array[:, 0]
    avg = np.average(vals)
    size = len(array[:, 0])
    var = 0
    for val in vals:
        var += (val - avg) ** 2
    return math.sqrt(var / size)


def std_dev2(vals):
    """ Calculate standard deviation of a sample. """
    avg = np.average(vals)
    size = len(vals)
    var = 0
    for val in vals:
        var += (val - avg) ** 2
    return math.sqrt(var / size)


def fourier(array):
    """ Calculate fourier transform of a sample. """
    val = np.fft.fft(array[:, 0])
    abs_val = [math.sqrt(x) for x in val.real ** 2 + val.imag ** 2]
    return abs_val


def sampling(array):
    """ Calculate average sampling frequency. """
    time = array[len(array) - 1, 1] - array[0, 1]
    samples = len(array)
    return samples / time * 1e6  # time in microseconds



def rms_noise(array):
    """ Calculare RMS of noise. """
    values = array[:, 0]
    sumSqrs = 0
    for val in values:
        sumSqrs += val ** 2
    return math.sqrt(float(sumSqrs) / len(values))


def rms_noise_norm(array):
    """ Calculare normalized RMS of noise. """
    vals = array[:, 0]
    avg = np.average(vals)
    vals = vals - avg
    sumSqrs = 0
    for val in vals:
        sumSqrs += val ** 2
    return math.sqrt(float(sumSqrs) / len(vals))


def rms_sin(points, peak2peak, offset):
    """ Calculate RMS of discretisized sine wave. """
    pts = np.linspace(0, 2 * math.pi, points)
    sin = [(float(peak2peak) / 2.0 * math.sin(val) + offset) for val in pts]
    sumSqrs = 0
    for val in sin:
        sumSqrs += val ** 2
    return math.sqrt(float(sumSqrs) / len(sin))


def snr_norm(array, peak2peak, offset):
    """ Calculate SNR of a sine wave with offset and normalized RMS of noise.
	Return value is in dB. """

    rmsNoise = rms_noise_norm(array)
    rmsSin = math.sqrt(offset ** 2 + 0.5 * ((float(peak2peak) / 2.0) ** 2))
    return 20 * math.log10(float(rmsSin) / float(rmsNoise))


def snr_min(bitWidth):
    """ Calculate theoretical minimum SNR for sine wave. Return value is in dB. """
    return 6.02 * (bitWidth - 1) + 1.76


def snr_variance(array, peak2peak):
    """ Calculate signal-to-noise ratio for discretisized sine wave.
	Return value is in dB. """
    points = np.linspace(0, 2 * math.pi, 5000)
    sin = [float(peak2peak) / 2.0 * math.sin(val) for val in points]
    values = array[:, 0]
    avg_val = np.average(values)
    values = values - avg_val
    var_err = (std_dev2(values)) ** 2
    var_sig = (std_dev2(sin)) ** 2
    return (10 * math.log10(float(var_sig) / float(var_err)))


############################################
## read file with data + perform cleaning ##
############################################


##############################################################################
## displays data as fft and time-domian plot and prints relevant statistics ##
##############################################################################

def animate(i, q):
    """ visualize data as it is ported in"""
    while q.empty():
        return
    else:
        sample = q.get()
    arr_batch = []
    # convert array from strings to floats
    # note that the actual big data array is the first item in tuple, while the specific batch is the second item.
    for string in sample:
        arr_batch.append([float(num) for num in string.split(" ")])

    # arr_batch = [x.split(" ") for x in values[batch_count]]
    arr_batch = np.array(arr_batch)
    # arr_batch = [x.astype(float) for x in arr_batch]

    clean_data(arr_batch)

    deviation = []
    average = []
    deviation.append(std_dev(arr_batch))
    average.append(np.average(arr_batch[:, 0]))

###################################
## fourier transform calculation ##
###################################

    ft = []  # fourier transform values
    sps = []  # sampling speed values
    freq = []  # frequency vectors - for plotting
    ft.append(fourier(arr_batch))
    sps.append(sampling(arr_batch))
    freq.append(sps[0] / len(arr_batch) * np.linspace(0, len(arr_batch) - 1, len(arr_batch)))


    ######################
    ## plotting results ##
    ######################

    xlabel1 = "Time [ms]"
    xlabel2 = "Frequency [Hz]"
    ylabel1 = "Voltage [mV]"
    ylabel2 = "Noise amplitude"

    # plot voltage - time graph

    plt.subplot(2, 1, 1)
    # for arr in arrays:
    plt.plot([(time - arr_batch[0, 1])*1e-3 for time in arr_batch[:, 1]], [(val - average[0]) * 1e3 for val in arr_batch[:, 0]], linewidth=0.4)  # time in microseconds
    #      plt.plot([time*1e3-1000 for time in arr[:,2]], [(val-average[i])*1e3 for val in arr[:,1]], label = labels[i], linewidth = 0.4)  #time in seconds
    plt.xlabel(xlabel1)
    plt.ylabel(ylabel1)

    # plot freq_output - freq_input graph

    # approach: take a sample with 100k data points, track the max and min, and plot the ave frequency of t50 datapts
    # plt.subplot(2, 1, 1)
    # indexes = int(ceil(len(ft[0])/2))
    # freq_analysis = ft[0][:indexes]
    # ave_frequency_indexes = np.argpartition(freq_analysis, -6)[-6:][:-1]
    # ave_frequency = 0
    # max = np.max(ave_frequency_indexes)
    # min = np.min(ave_frequency_indexes)
    # for x in ave_frequency_indexes:
    #     ave_frequency += freq[0][x]
    # ave_frequency = ave_frequency/5
    # plt.errorbar(100, ave_frequency, yerr=freq[0][max] - freq[0][min])
    # print(f"wave frequency is: \n{ave_frequency}\n error is \n{freq[0][max]-freq[0][min]}")
    # print(f"max: \n {freq[0][max]} \n min \n {freq[0][min]}")
    # [val * 1e-6 for val in ft[1:len(ft)]]

    indexes = int(ceil(len(ft[0])/2))
    freq_analysis = ft[0][:indexes]

    # plot voltage - frequency graph

    plt.subplot(2, 1, 2)
    # 2/len(arr_batch[:,0])*np.abs(ft[0])
    for mode, f in zip(ft, freq):
        # fig = plt.figure(figsize=(3.5, 3))
        L = len(mode)
        mode = np.array(mode)
        normalized_mode = 2*(mode/L)
        plt.plot(f[1:len(f)], normalized_mode[1:len(normalized_mode)],
                 linewidth=0.4)  # time in microseconds
    # plt.plot(freq[0], [val * 1e-6 for val in ft[0]], linewidth=0.4)  # time in microseconds
    # plt.plot(f[1:len(f)], mode[1:len(mode)], label = labels[i], linewidth = 0.4) #time in seconds
    ave_frequency_indexes = np.argpartition(freq_analysis, -6)[-6:][:-1]
    plt.xlabel(xlabel2)
    plt.ylabel(ylabel2)

     # i += 1
    print("\nDeviation: ")  # unit: V
    for x in deviation: print(x)

    print("\nSNR: ")
    print(snr_norm(arr_batch, 5, 2.5))

    print("Theoretical minimum SNR of a 24 bit AD converter: %3.2f" % (snr_min(24)))


def animation_process(q):
    # will the parameter value update given it changes in data_extraction? Not convinced it will
    while q.empty():
        pass
    fig = plt.figure(figsize=(3.5, 3))
    ani = matplotlib.animation.FuncAnimation(fig, animate, interval=300, fargs=(q,))
    plt.show()


def data_extraction(q):
    # Parse the command line.
    parser = ArgumentParser()
    parser.add_argument('config_file', type=FileType('r'))
    parser.add_argument('--reset', action='store_true')
    args = parser.parse_args()

    # Parse the configuration.
    # See https://github.com/edenhill/librdkafka/blob/master/CONFIGURATION.md
    config_parser = ConfigParser()
    config_parser.read_file(args.config_file)
    config = dict(config_parser['default'])
    config.update(config_parser['consumer'])

    # Create Consumer instance
    consumer = Consumer(config)

    # Set up a callback to handle the '--reset' flag.
    def reset_offset(consumer, partitions):
        if args.reset:
            for p in partitions:
                p.offset = OFFSET_BEGINNING
            consumer.assign(partitions)

    # Subscribe to topic
    topic = "datatime"
    consumer.subscribe([topic], on_assign=reset_offset)

    # Poll for new messages from Kafka and print them.
    try:
        while True:
            msg = consumer.poll(0.5)
            if msg is None:
                # Initial message consumption may take up to
                # `session.timeout.ms` for the consumer group to
                # rebalance and start consuming
                print("Waiting...")
            elif msg.error():
                print("ERROR: %s".format(msg.error()))
            else:
                # Extract the (optional) key and value, and print.
                print("received")
                data_array = (msg.value().decode('utf-8').split("\n")[:-1])
                # print("Consumed event from topic {topic}: key = {key:12} value = {value:12}".format(
                #    topic=msg.topic(), key=msg.key().decode('utf-8'), value=msg.value().decode('utf-8')))
                # batch_count += 1
                q.put(data_array)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave group and commit final offsets
        consumer.close()


if __name__ == '__main__':
    """ Running two process to display graphs and process incoming data """
    q = multiprocessing.Queue()
    p1 = multiprocessing.Process(name="p1", target=data_extraction, args=(q,))
    p2 = multiprocessing.Process(name="p2", target=animation_process, args=(q,))
    p1.start()
    p2.start()
    p1.join()
    p2.join()

