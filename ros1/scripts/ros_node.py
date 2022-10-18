#!/usr/bin/env python
from email.header import Header
from itertools import count
import rospy
import socket
import matplotlib.pyplot as plt
import time
import datetime
from nav_ross.msg import HighPrecisionFFTData
import numpy as np



class RadarNode:
    def __init__(self):
        # Start ros publisher
        self.radar_pub = rospy.Publisher("/Navtech/FFTData", HighPrecisionFFTData, queue_size = 1)

    
        ########################################################################
        # Below settings need to be changed to match your setup
        ########################################################################
        self.tcp_ip = '192.168.0.1'   # This is to be the source of raw radar data. It can be a real radar or the address where 
                                # the Navtech ColossusNetrecordPlayback tool is running in playback mode
        self.tcp_port = 6317          # This is the port that the radar is using, this generally will not need to be changed

        self.radar_frame = "navtech"
        ########################################################################



        ########################################################################
        # The below settings should not be changed
        ########################################################################
        self.signature_length = 16
        self.version_length = 1
        self.message_type_length = 1
        self.payload_length = 4
        self.payload_size = 0
        self.header_length = self.signature_length + self.version_length + self.message_type_length + self.payload_length
        self.check_signature = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe'
        self.check_signature = [x for x in self.check_signature]
        self.config_read = False
        self.data_read = False
        self.encoder_size = 0
        self.version = 0
        self.power = []
        self.bearing = 0
        self.timestamp_with_nanoseconds = ''
        self.bit_depth = 0
        self.radar_socket = None
        ########################################################################

        ########################################################################
        # The below section contains the main program code
        ########################################################################

        # Try (for 5 seconds) to connect to the radar
        try:
            rospy.loginfo("Connecting to radar at {} on port {}".format(self.tcp_ip, self.tcp_port))
            self.radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.radar_socket.settimeout(5)
            self.radar_socket.connect((self.tcp_ip, self.tcp_port))
        except:
            rospy.loginfo("Unable to connect to radar at {} on port {}".format(self.tcp_ip, self.tcp_port))
            exit()

        # Try (for 5 seconds) to get a configuration message from the radar
        timeout = time.time() + 5
        self.radar_socket.settimeout(5)
        try:
            rospy.loginfo("Reading configuration message from radar")
            while self.config_read == False and time.time() < timeout:
                self.send_config_request()           # send a request configuration message to the radar
                time.sleep(0.5)
                self.handle_received_message()
        except:
            rospy.loginfo("Unable to read configuration message from radar")
            exit()

        # Send a message to the radar to instruct it to start sending FFT data
        rospy.loginfo("Starting FFT data")
        self.start_fft_data()

        # Try (for 5 seconds) to get an FFT message from the radar
        timeout = time.time() + 5
        self.radar_socket.settimeout(5)
        #try:
        rospy.loginfo("Reading azimuths of FFT data from radar")
        # Timeout if no message for 5 seconds
        while time.time() < timeout: 
            self.handle_received_message()
            timeout = time.time() + 5
        #except:
        #    rospy.loginfo("Unable to read FFT data from radar")
        #    exit()

        ########################################################################

    ########################################################################
    # The below section contains function definitions
    ########################################################################

    # Send a Configuration Request Message - type 20
    def send_config_request(self):
        stop_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(self.version,1, byteorder='big') + int.to_bytes(20,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
        self.radar_socket.send(stop_data_message)
        rospy.loginfo("--------Sent Config Request---------")
        rospy.loginfo("")

    # Send a start FFT Data Request Message - type 21
    def start_fft_data(self):
        start_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(self.version,1, byteorder='big') + int.to_bytes(21,1, byteorder='big') + int.to_bytes(0,4, byteorder='big')
        self.radar_socket.send(start_data_message)
        rospy.loginfo("----------Started FFT Data----------")
        rospy.loginfo("")
        

    # Send a stop FFT Data Request Message - type 22
    def stop_fft_data(self):
        stop_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(self.version,1, byteorder='big') + int.to_bytes(22,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
        self.radar_socket.send(stop_data_message)
        rospy.loginfo("----------Stopped FFT Data----------")
        rospy.loginfo("")

    # Read and process the response message and publish ros message 
    def handle_received_message(self):

        # Read the message header
        self.message_type = 0
        #try:
        data = []
        while (len(data) < self.signature_length):
            data += self.radar_socket.recv(1)
        if (data == self.check_signature):
            data = []
            while (len(data) < self.header_length - self.signature_length):
                data += self.radar_socket.recv(1)
            self.version = data[0]
            self.message_type = data[1]
            self.payload_size = int.from_bytes(data[2:6], byteorder='big')
        #except:
         #   rospy.loginfo("Error reading message header")

        # Read the rest of the message
        if (self.message_type == 10): # Configuration data from radar
            try:
                data = []
                while (len(data) < self.payload_size):
                    data += self.radar_socket.recv(1)
                #print("config")
                #print(data)
                if (len(data) == self.payload_size):
                        self.azimuth_samples = int.from_bytes(data[0:2], byteorder='big')
                        self.range_resolution = int.from_bytes(data[2:4], byteorder='big')
                        self.range_bins = int.from_bytes(data[4:6], byteorder='big')
                        self.encoder_size = int.from_bytes(data[6:8], byteorder='big')
                        self.rotation_speed = int.from_bytes(data[8:10], byteorder='big')
                        self.packet_rate = int.from_bytes(data[10:12], byteorder='big')
                        rospy.loginfo("----------Config Message----------\n")
                        rospy.loginfo("Azimuth Samples:  {} samples/rotation".format(self.azimuth_samples))
                        rospy.loginfo("Range Resolution: {} mm/bin".format(self.range_resolution/10))
                        rospy.loginfo("Range:            {} bins".format(self.range_bins))
                        rospy.loginfo("Encoder Size:     {} counts/rotation".format(self.encoder_size))
                        rospy.loginfo("Rotation Speed:   {} mHz".format(self.rotation_speed))
                        rospy.loginfo("Packet Rate:      {} azimuths/second".format(self.packet_rate))
                        self.config_read = True
                        rospy.loginfo("")
            except:
                rospy.loginfo("Error reading config message")

        elif (self.message_type == 31 or self.message_type == 30): # Message type 31 is HighRes (16Bit) FFT Data 
                                                                   # and message type 30 is 8bit data from the radar
            #try:
            data = []
            while (len(data) < self.payload_size):
                data += self.radar_socket.recv(1)
            if (len(data) == self.payload_size):
                counter = int.from_bytes(data[2:4], byteorder='big')
                azimuth = int.from_bytes(data[4:6], byteorder='big')
                bearing=360*(azimuth/self.encoder_size)
                seconds = int.from_bytes(data[6:10], byteorder='little')
                split_seconds = int.from_bytes(data[10:14], byteorder='little')
                #if (self.message_type == 31):  #This is the colossus message type for 16 bit FFT data
                    #rospy.loginfo("    FFT: 16Bit (HighRes)")
                #else: # If the data isn't 16 bit, but we are here in the code, we must have 8 bit data
                #    rospy.loginfo("    FFT: 8Bit (Standard)")            
                #rospy.loginfo("Counter: {}".format(counter))
                #rospy.loginfo("Azimuth: {}".format(azimuth))
                #rospy.loginfo("Bearing: {}".format(round(bearing,2)))
                #rospy.loginfo("Seconds: {}".format(seconds))
                #ts = datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')
                #rospy.loginfo("NanoSec: {}".format(round(split_seconds,5)))
                #timestamp_with_nanoseconds = "{}.{}".format(ts, str(int(round(split_seconds/10000,0))).rjust(5,'0'))
                #rospy.loginfo("Day/Tim: " + timestamp_with_nanoseconds)
                #rospy.loginfo ("\n")
                fft_data_bytes = data[14:]
                power = []
                if (self.message_type == 31):
                    for index in range (int(len(fft_data_bytes)/2)):
                        power.append(int.from_bytes(fft_data_bytes[index:index+2],byteorder='big'))
                        #power = np.array(self.power, np.uint16)
                    data_read = True
                    bit_depth = 16
                else:
                    for index in range ((len(fft_data_bytes))):
                        power.append((int(fft_data_bytes[index])))
                        #power = np.array(self.power, np.uint16)
                    data_read = True
                    bit_depth = 8
                
                # Make ros message
                out_data = HighPrecisionFFTData()
                out_data.header.stamp = rospy.Time(seconds,split_seconds)
                out_data.header.frame_id = self.radar_frame
                out_data.angle = bearing
                out_data.azimuth = azimuth
                out_data.sweepCounter = counter
                out_data.size = self.payload_size
                out_data.data = power

                self.radar_pub.publish(out_data)
                
            #except:
            #    rospy.logwarn("Error reading FFT message")
        else:
            rospy.logwarn("Unhandled message type: {}".format(self.message_type))
    ########################################################################

    def on_shutdown(self):
        # Send a message to the radar to instruct it to stop sending FFT data
        rospy.loginfo("Stopping FFT data")
        self.stop_fft_data()

if __name__ == '__main__':
    rospy.init_node('radar_pub')

    node = RadarNode()

    while not rospy.is_shutdown():
        rospy.spin()

    rospy.on_shutdown(node.on_shutdown)