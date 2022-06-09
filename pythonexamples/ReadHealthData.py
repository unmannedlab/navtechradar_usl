import socket
import time
import datetime
import health_pb2

########################################################################
# Below settings need to be changed to match your setup
########################################################################
tcp_ip = '192.168.0.1'   # This is to be the source of raw radar data. This will only work with a physical radar 
                         # as the Navtech ColossusNetrecordPlayback tool does not generate health messages
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
check_signature = [x for x in check_signature]
config_read = False
data_read = False
version = 0
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

# Send a start Health Data Request Message - type 23
def start_health_data():
    global radar_socket
    global version
    start_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(23,1, byteorder='big') + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(start_data_message)
    print("----------Started Health Data----------")
    print("")

# Send a stop health Request Message - type 24
def stop_health_data():
    global radar_socket
    global version
    stop_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(24,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(stop_data_message)
    print("----------Stopped Health Data----------")
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
    global message_type
    global data

    # Read the message header
    message_type = 0
    try:
        data = []
        while (len(data) < signature_length):
            if time.time() > timeout:
                raise ValueError('Data read timeout exceeded')
            data += radar_socket.recv(1)
        if (data == check_signature):
            data = []
            while (len(data) < header_length - signature_length):
                if time.time() > timeout:
                    raise ValueError('Data read timeout exceeded')
                data += radar_socket.recv(1)
            version = data[0]
            message_type = data[1]
            payload_size = int.from_bytes(data[2:6], byteorder='big')
    except:
        print("Error reading message header")

    # Read the rest of the message
    if (message_type == 10): # Configuration data from radar
        try:
            data = []
            while (len(data) < payload_size):
                data += radar_socket.recv(1)
            if (len(data) == payload_size):
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

    elif (message_type == 40): # Health data from radar
        try:
            data = []
            while (len(data) < payload_size):
                data += radar_socket.recv(1)
            if (len(data) == payload_size):
                message=health_pb2.Health()
                message.ParseFromString(bytearray (data))

                # Print the entire health message
                print("Entire health message:")
                print(message)

                # Print a single value from the health message
                print("Health message die temperature:")
                print(message.dietemperature.value)
                print()

                data_read = True
        except:
            print("Error reading health message")
    else:
        print("Unhandled message type: {}".format(message_type))
########################################################################



########################################################################
# The below section contains the main program code
########################################################################

# Try (for 5 seconds) to connect to the radar
try:
    print("Connecting to radar at {} on port {}".format(tcp_ip, tcp_port))
    radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    radar_socket.settimeout(5)
    radar_socket.connect((tcp_ip, tcp_port))
except:
    print("Unable to connect to radar at {} on port {}".format(tcp_ip, tcp_port))
    exit()

# Try (for 5 seconds) to get a configuration message from the radar
timeout = time.time() + 5
radar_socket.settimeout(5)
try:
    print("Reading configuration message from radar")
    while config_read == False and time.time() < timeout:
        send_config_request()           # send a request configuration message to the radar
        time.sleep(0.5)
        handle_received_message()
except:
    print("Unable to read configuration message from radar")
    exit()

# Send a message to the radar to instruct it to start sending health data
print("Starting health data")
start_health_data()

# Try (for 10 seconds) to get a health message from the radar
# Default for radar sending health messages is every 10 seconds
timeout = time.time() + 10
radar_socket.settimeout(10)
try:
    print("Reading health message from radar")
    while data_read == False and time.time() < timeout:
        handle_received_message()
except:
    print("Unable to read health message from radar")
    exit()

# Send a message to the radar to instruct it to stop sending health data
print("Stopping health data")
stop_health_data()

########################################################################
