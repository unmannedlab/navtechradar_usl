# Imports
import os.path
import sqlite3
import struct
import datetime
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
#from interfaces.msg import FftDataMessage
from datetime import timezone
import math

# Globals
bagFilePath = '/mnt/c/Local/Development/iasdk/ros2/recordings/lpu_test/lpu_test_0.db3'
previousTimestamp = datetime.datetime(1970, 1, 1, 0, 0, 0, 0, tzinfo=timezone.utc)
firstTimestamp = True

# Main
if (not os.path.isfile(bagFilePath)):
    print("File does not exist")
    quit()

conn = sqlite3.connect(bagFilePath)
cursor = conn.cursor()

topics_data = cursor.execute("SELECT id, name, type FROM topics").fetchall()
print("Topic data")
for topic_data in topics_data:
    print(topic_data)
print()

csv_file = open("output_data/timestamp_data.csv", "w")
csv_file.write("Azimuth,Sweep Counter,Timestamp,Time Offset\n")
biggest_time_offset = float('-inf')
smallest_time_offset = float('inf')
last_sweep_counter = 0

print("Messages info")
messages_data = cursor.execute("SELECT id, topic_id, timestamp, data FROM messages WHERE topic_id = 1;").fetchall()
for message_data in messages_data:

    #print('Message Length: {}'.format(len(message_data[3])))
    azimuth =  struct.unpack("<H", message_data[3][8:10])[0]
    sweep_counter =  struct.unpack("<H", message_data[3][10:12])[0]
    ntp_seconds = message_data[3][12:16]
    ntp_split_seconds = message_data[3][16:20]

    epoch_time = datetime.datetime(1970, 1, 1, 0, 0, 0, 0, tzinfo=timezone.utc)
    date_time = epoch_time + datetime.timedelta(0, struct.unpack(">L", ntp_seconds)[0])
    date_time = date_time + datetime.timedelta(0, (struct.unpack(">L", ntp_split_seconds)[0] / 1000000000.0))
    
    #print('ntp_seconds: {} {}'.format(ntp_seconds, struct.unpack(">L", ntp_seconds)[0]))
    #print('ntp_split_seconds: {} {}'.format(ntp_split_seconds, struct.unpack(">L", ntp_split_seconds)[0]))
    #print('date_time: {}'.format(date_time))
    total_offset_seconds = 0
    if (not firstTimestamp):
        total_offset_seconds = (date_time - previousTimestamp).total_seconds()
        if (total_offset_seconds < smallest_time_offset):
            smallest_time_offset = total_offset_seconds
        if (total_offset_seconds > biggest_time_offset):
            biggest_time_offset = total_offset_seconds
        if (abs(last_sweep_counter - sweep_counter) != 1):
            if (last_sweep_counter != 65535 and sweep_counter != 0):
                print('Messages in wrong order: {} {}'.format(last_sweep_counter, sweep_counter))
    csv_file.write(str(azimuth)+","+str(sweep_counter)+","+str(date_time)+","+str(total_offset_seconds))
    csv_file.write("\n")
    firstTimestamp = False
    previousTimestamp = date_time
    last_sweep_counter = sweep_counter
    #print()

print('smallest_time_offset: {}'.format(smallest_time_offset))
print('biggest_time_offset: {}'.format(biggest_time_offset))
csv_file.close()
conn.close()