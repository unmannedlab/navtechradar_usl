import socket
import matplotlib.pyplot as plt
import time
import datetime
import numpy as np
import cv2

########################################################################
# Below settings need to be changed to match your setup
########################################################################
tcp_ip = '192.168.0.1'   # This is to be the source of raw radar data. It can be a real radar or the address where 
                         # the Navtech ColossusNetrecordPlayback tool is running in playback mode
tcp_port = 6317          # This is the port that the radar is using, this generally will not need to be changed
########################################################################



########################################################################
# The below settings should not be changed
########################################################################
signature_length = 16
version_length = 1
message_type_length = 1
payload_length = 4
payload_size = 0
header_length = signature_length + version_length + message_type_length + payload_length
check_signature = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe'
config_read = False
data_read = False
encoder_size = 0
version = 0
power = []
bearing = 0
timestamp_with_nanoseconds = ''
bit_depth = 0
read_first_fft_message = False
azimuth = 0
starting_azimuth = 0
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
    print("--------Sent Config Request---------")
    print("")

# Send a start FFT Data Request Message - type 21
def start_fft_data():
    global radar_socket
    global version
    start_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(21,1, byteorder='big') + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(start_data_message)
    print("----------Started FFT Data----------")
    print("")

# Send a stop FFT Data Request Message - type 22
def stop_fft_data():
    global radar_socket
    global version
    stop_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(22,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(stop_data_message)
    print("----------Stopped FFT Data----------")
    print("")

# Read and process the response message
def handle_received_message():
    global signature_length
    global header_length
    global check_signature
    global payload_size
    global config_read
    global data_read
    global radar_socket
    global version
    global encoder_size
    global power
    global bearing
    global timestamp_with_nanoseconds
    global bit_depth
    global message_type
    global read_first_fft_message
    global azimuth
    global starting_azimuth
    global azimuth_samples
    global range_bins
    global range_resolution

    # Read the message header
    message_type = 999
    try:
        data = radar_socket.recv(signature_length)
        if (data == check_signature):
            data = radar_socket.recv(header_length - signature_length)
            version = data[0]
            message_type = data[1]
            payload_size = int.from_bytes(data[2:6], byteorder='big')
    except:
        print("Error reading message header")

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
    elif (message_type == 31 or message_type == 30): # Message type 31 is HighRes (16Bit) FFT Data 
                                                   # and message type 30 is 8bit data from the radar
        data = radar_socket.recv(payload_size)
        if (len(data) == payload_size):
            try:
                counter = int.from_bytes(data[2:4], byteorder='big')
                azimuth = int.from_bytes(data[4:6], byteorder='big')
                bearing = 360*(azimuth/encoder_size)
                seconds = int.from_bytes(data[6:10], byteorder='little')
                split_seconds = int.from_bytes(data[10:14], byteorder='little')
                ts = datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')
                timestamp_with_nanoseconds="{}.{}".format(ts, str(int(round(split_seconds/10000,0))).rjust(5,'0'))
                fft_data_bytes = data[14:]
                if (message_type == 31):
                    for index in range (int(len(fft_data_bytes)/2)):
                        power.append(int.from_bytes(fft_data_bytes[index:index+2],byteorder='big'))
                    data_read = True
                    bit_depth = 16
                else:
                    for index in range ((len(fft_data_bytes))):
                        power.append((int(fft_data_bytes[index])))
                    data_read = True
                    bit_depth=8
                if not read_first_fft_message:
                    starting_azimuth = int(azimuth / encoder_size * azimuth_samples)
                    read_first_fft_message = True
            except:
                print("Error reading FFT message")
    else:
        print("Unhandled message type: {}".format(message_type))
########################################################################



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

# Send a message to the radar to instruct it to start sending FFT data
print("Starting FFT data")
start_fft_data()

# Try to get an entire rotation of FFT data messages from the radar
print("Reading one rotation of FFT data from radar")
fft_data_captured = 0
data_array = np.zeros((azimuth_samples, range_bins, 3))
handle_received_message()             # Read the first FFT data
data_array[int(azimuth / encoder_size * azimuth_samples), :len(power), 1] = power
fft_data_captured += 1
power.clear()
timeout = time.time() + 5
while int(azimuth / encoder_size * azimuth_samples) != starting_azimuth or fft_data_captured < azimuth_samples:
    time.sleep(0.01)            # just in case we end up looping here
    handle_received_message()             # Read the rest of the FFT data to make a complete rotation
    data_array[int(azimuth / encoder_size * azimuth_samples), :len(power), 1] = power
    fft_data_captured += 1
    power.clear()
    if time.time() > timeout:
        print("Unable to read FFT data from radar")
        exit()

# Send a message to the radar to instruct it to stop sending FFT data
print("Stopping FFT data")
stop_fft_data()

# Now show the captured rotation as a b-scan style image
data_array = np.interp(data_array, (data_array.min(), data_array.max()), (0, 255))
data_array = data_array.astype(np.uint8)
bScanPlot = plt.imshow(data_array)
plt.title('B-Scan style image of complete rotation of FFT data')
plt.ylabel('Azimuth', fontsize=12)
plt.xlabel('Bin', fontsize=12)
plt.tight_layout()
print("Close b-scan plot to see polar plot")
plt.show()

# Now show the captured rotation as a polar plot style image
data_array_scaled = cv2.resize(data_array, ((800,800)), cv2.INTER_AREA)
polar_data_array =  cv2.linearPolar(data_array_scaled, (int(data_array_scaled.shape[1] / 2), int(data_array_scaled.shape[0] / 2)), int(data_array_scaled.shape[0] / 2), cv2.WARP_FILL_OUTLIERS + cv2.WARP_INVERSE_MAP)
polar_data_array = cv2.rotate(polar_data_array, cv2.ROTATE_90_COUNTERCLOCKWISE) # Rotate to make zeroth azimuth point north/up
polarPlot = plt.imshow(polar_data_array)
plt.title('Polar plot style image of complete rotation of FFT data')
plt.ylabel('Range (Metres)', fontsize=12)
range_metres = range_resolution * range_bins / 10000
plt.xticks(np.arange(0, data_array_scaled.shape[0], 100), np.arange(-(range_metres), range_metres, range_metres / 4, dtype = int))
plt.xlabel('Range (Metres)', fontsize=12)
plt.yticks(np.arange(0, data_array_scaled.shape[0], 100), np.arange(range_metres, -(range_metres), -(range_metres / 4), dtype = int))
plt.tight_layout()
print("Close polar plot to exit")
plt.show()

########################################################################