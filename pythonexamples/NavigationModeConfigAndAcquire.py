from posixpath import split
from tracemalloc import stop
import matplotlib.cm as cm       # colourmap
import matplotlib.pyplot as plt
import socket
import struct
import numpy as np
import time
import datetime
import os

#########################################################################
### Note that this script requires radar firmware 3.0.0.131 or higher ###
#########################################################################



########################################################################
# The below settings need to be changed to match your setup
########################################################################
tcp_ip = '10.77.2.211'   # this is to be the source of raw radar data. it can be a real radar or the address where 
                         # the navtech colossusnetrecordplayback tool is running in playback mode
tcp_port = 6317          # this is the port that the radar is using, this generally will not need to be changed
########################################################################



########################################################################
# The below settings can be changed to alter navigation mode behaviour
########################################################################
max_range_of_interest = 200     # in metres: this value is used to discard points beyond a certain range
                                # usually, a customer will set the radar's operating range through the radar's web ui

bins = 5                        # number of bins to operate on (window size) for peakfinding

min_bins = 3                    # closest bin to consider within the peak finding.
                                # this allows for close-in radar returns to be ignored

nav_threshold = 55              # power threshold used to configure the operation of the 
                                # navigationmode. note that setting this value too low can 
                                # result in the processing effort required from the radar
                                # exceeding that which is available. this will result in
                                # points appearing to be "missing" from ranges of azimuths
                                # in the returned dataset

max_peaks = 10                  # the number of peaks per azimuth that navigationmode should report
########################################################################



########################################################################
# The below settings should not be changed
########################################################################
signature_length = 16
version_length = 1
message_type_length = 1
payload_length = 4
header_length = signature_length + version_length + message_type_length + payload_length
check_signature = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe'
config_read = False
version = 0
message_type = 0
payload_size = 0
azimuth = 0
encoder_size = 0
packet_rate = 0
azimuth_samples = 0
first_run = True
range_list = []
bearing_list = []
power_list = []
nav_message_count = 0
nav_config_read = False
timestamp_seconds_and_fractional_seconds = 0.0
first_timestamp_seconds_and_fractional_seconds = 0.0
first_message = True
rotation_speed = 4000
########################################################################



########################################################################
# The below section contains function definitions
########################################################################

# Send a Configuration Request Message - type 20
def send_config_request():
    global radar_socket
    global version
    stop_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(20,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(stop_data_message)
    print("----------- Requested config -----------")
    print("")

# Send a start navigation data request message - type 120
def start_nav_data():
    global radar_socket
    global version
    start_nav_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(120,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(start_nav_message)
    print("---------- Requested nav data ----------")
    print("")

# Send a stop navigation data request message - type 121
def stop_nav_data():
    global radar_socket
    global version
    stop_nav_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(121,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(stop_nav_message)
    print("----------- Stopped nav data -----------")
    print("")

# Send a set navigation threshold message - type 122 - message type not required for radars with firmware 3.0.0.131 or later.
#def set_nav_threshold(threshold_db):
#    global radar_socket
#    global version
#    set_nav_threshold_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(122,1, byteorder='big')  + int.to_bytes(2,4, byteorder='big')+ int.to_bytes(int(thresholddb*10),2, byteorder='big')
#    radar_socket.send(set_nav_threshold_message)
#    print("----------set nav threshold {}----------".format(threshold_db))
#    print("")

# Send a set navigation configuration message - type 205
def set_nav_config(bins_to_operate_on, minimum_bin, navigation_threshold, max_peaks_per_azimuth):
    global radar_socket
    global version
    set_nav_config_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(205,1, byteorder='big')  + int.to_bytes(12,4, byteorder='big')+ int.to_bytes(int(bins_to_operate_on),2, byteorder='big')+ int.to_bytes(int(minimum_bin),2, byteorder='big')+ struct.pack('>f',navigation_threshold*10) + int.to_bytes(int(max_peaks_per_azimuth),4, byteorder='big')
    radar_socket.send(set_nav_config_message)
    print("----------- Set nav config -----------")
    print("Bins      {}".format(bins_to_operate_on))
    print("Minbin    {}".format(minimum_bin))
    print("Threshold {}".format(navigation_threshold))
    print("Maxpeaks  {}".format(max_peaks_per_azimuth))
    print("")

# Send a get navigation configuration message - type 203
def get_nav_config():
    global radar_socket
    global version
    get_nav_config_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(203,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(get_nav_config_message)
    print("-------- Requested nav config ----------")
    print("")

# Read and process the message
def handle_received_message():
    global signature_length
    global header_length
    global check_signature
    global config_read
    global nav_config_read
    global radar_socket
    global version
    global payload_size
    global encoder_size
    global azimuth_samples
    global nav_message_count
    global timestamp_seconds_and_fractional_seconds
    global first_timestamp_seconds_and_fractional_seconds
    global first_message
    global rotation_speed
    global message_type
    # read header
    try:
        data = radar_socket.recv(signature_length)
        if (data == check_signature):
            data = radar_socket.recv(header_length-signature_length)
            version = data[0]
            message_type = data[1]
            payload_size = int.from_bytes(data[2:6], byteorder='big')
    except:
        print("error reading message header")

     # Read the rest of the message
    if (message_type == 10): # Configuration data from radar
        data = radar_socket.recv(payload_size)
        if (len(data) == payload_size):
            try:
                azimuth_samples = int.from_bytes(data[0:2], byteorder='big')
                range_resolution = int.from_bytes(data[2:4], byteorder='big')
                range_bins = int.from_bytes(data[4:6], byteorder='big')
                encoder_size = int.from_bytes(data[6:8], byteorder='big')
                rotation_speed = int.from_bytes(data[8:10], byteorder='big')
                packet_rate = int.from_bytes(data[10:12], byteorder='big')
                print("----------Config Message----------\n")
                print("Azimuth Samples:  {} samples/rotation".format(azimuth_samples))
                print("Range Resolution: {} mm/bin".format(range_resolution/10))
                print("Range:            {} bins".format(range_bins))
                print("Encoder Size:     {} counts/rotation".format(encoder_size))
                print("Rotation Speed:   {} mHz".format(rotation_speed))
                print("Packet Rate:      {} azimuths/second".format(packet_rate))
                config_read = True
                print("")
            except:
                print("Error reading config message")

    if (message_type == 204): # Navigation configuration data from radar
        data = radar_socket.recv(payload_size)
        if (len(data) == payload_size):
            try:
                bins = int.from_bytes(data[0:2], byteorder='big')
                min_bin = int.from_bytes(data[2:4], byteorder='big')
                [threshold] = struct.unpack('>f',(data[4:8]))
                max_peaks = int.from_bytes(data[8:12], byteorder='big')
                print("-- Get navigation config (message204) --")
                print("Bins      {}".format(bins))
                print("Minbin    {}".format(min_bin))
                print("Threshold {}".format(threshold/10))
                print("Maxpeaks  {}".format(max_peaks)) 
                print("") 
                nav_config_read = True
            except:
                print("Error reading navigation configuration message")
    elif (message_type == 123): # navigation data from the radar
        data = radar_socket.recv(payload_size)  
        if (len(data) == payload_size):
            try:
                azimuth = int.from_bytes(data[0:2], byteorder='big')
                bearing = 360*(azimuth/encoder_size)
                seconds = int.from_bytes(data[2:6], byteorder='big')
                split_seconds = int.from_bytes(data[6:10], byteorder='big')
                nav_data = list(data[10:])
                timestamp_seconds_and_fractional_seconds = seconds + (split_seconds/1000000000)
                for index in range (int(len(nav_data)/6)):
                    byte_offset = index * 6
                    peak_range = int.from_bytes(nav_data[0+byte_offset:4+byte_offset],byteorder='big')
                    peak_power = int.from_bytes(nav_data[4+byte_offset:6+byte_offset],byteorder='big')
                    peak_range_metres = peak_range/1000000
                    peak_power_db = peak_power/10
                    if (peak_range_metres<=max_range_of_interest)and(peak_range_metres>=3):
                        range_list.append(peak_range_metres)
                        bearing_list.append((bearing/360)*2 * np.pi)
                        power_list.append(peak_power_db) 
                if firstmessage == True:
                    first_timestamp_seconds_and_fractional_seconds = timestamp_seconds_and_fractional_seconds
                    firstmessage = False
                nav_message_count += 1                       
            except:
                print("Error reading nav message")
    else:
        ("Unhandled message type: {}".format(message_type))
######################################################################################



########################################################################
# The below section contains the main program code
########################################################################

# Try to connect to the radar
try:
    print("Connecting to radar at {} on port {}".format(tcp_ip, tcp_port))
    radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    radar_socket.connect((tcp_ip, tcp_port))
except:
    print("Unable to connect to radar at {} on port {}".format(tcp_ip, tcp_port))
    exit()

timestamp=datetime.datetime.now()
file_path=(timestamp.strftime('radardata/%y/%m/%d/'))	# the folder path that contains the log file
if (not os.path.exists(file_path)):	# if the folder isn't there
	os.makedirs (file_path)			# ... then create it
file_name=(file_path+(timestamp.strftime('navigationmodedata_%h-%m-%s.csv'))) # filename is made up of fixed string and day-number in month 

# Try (for 5 seconds) to get a configuration message from the radar
print("Reading configuration message from radar")
timeout = time.time() + 5
while config_read == False:
    send_config_request()           # send a request configuration message to the radar
    time.sleep(1)           # just to be sure!
    handle_received_message()         # read and interpret message response
    if time.time() > timeout:
        print("Unable to read configuration message from radar")
        exit()

set_nav_config(bins, min_bins, nav_threshold, max_peaks)   # send the required navigation mode configuration to the radar

# Try (for 5 seconds) to get the current navigation mode settings from the radar
print("Reading configuration message from radar")
timeout = time.time() + 5
while nav_config_read == False:
    get_nav_config()             # send a request navigationmode configuration message to the radar
    time.sleep(1)              # just to be sure!
    handle_received_message()            # read and interpret message response
    if time.time() > timeout:
        print("Unable to read configuration message from radar")
        exit()

start_nav_data()                 # request navigation data messages from the radar

#for raw fft data we know to expect a certain number of messages per rotation (defined by rotation-speed and packet-rate. for navigation mode, the number of messages per rotation cannot be defined (as it is dependent on whether peaks are identified or not). the loop below is set up to execute for messages covering at least the time taken for one rotation of the radar's antenna. 
while (timestamp_seconds_and_fractional_seconds - first_timestamp_seconds_and_fractional_seconds)<(1000/rotation_speed):
    handle_received_message()
stop_nav_data()                   # instruct the radar to stop sending navigation mode data

target = open(file_name,'w')		# ...then open it for writing ...
target.write("date,{}\n".format(timestamp.strftime('%y/%m/%d')))
target.write("time,{}\n".format(timestamp.strftime('%h:%m:%s')))
target.write("threshold,{}\n".format(nav_threshold))
target.write("binstooperateon,{}\n".format(bins))
target.write("minbin,{}\n".format(min_bins))
target.write("peakstoreturn,{}\n".format(max_peaks))
target.write("notes\n")
target.write("range(m),bearing(rad),power\n")
for i in range (len(power_list)):
    target.write("{},{},{}\n".format(range_list[i],bearing_list[i],power_list[i]))
target.close

print ("Peaks indentified: {}".format(len(power_list)))
fig = plt.figure("One rotation of navigation mode data")
fig.set_size_inches(9.,10.)
ax = fig.add_subplot(projection='polar')
ax.set_ylim(0,max_range_of_interest)
c = ax.scatter(bearing_list, range_list, s=18, c=power_list, cmap=cm.rainbow,vmin=min(power_list), vmax=max(power_list))
ax.set_title('Navigation mode data. binstooperateon={}, minbin={}, threshold={}, maxpeaks={}'.format(bins, min_bins, nav_threshold, max_peaks))
ax.set_theta_direction(-1)
ax.set_theta_offset(np.pi / 2.0)
plt.tight_layout()
plt.savefig(file_path + (timestamp.strftime('navigationmodedata_%h-%m-%s.pdf')), dpi=2000)
plt.show()
########################################################################