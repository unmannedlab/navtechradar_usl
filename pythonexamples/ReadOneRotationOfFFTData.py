from operator import truediv
import socket
import matplotlib.pyplot as plt
import time
import datetime
import numpy as np
import cv2

########################################################################
# Below settings need to be changed to match your setup
########################################################################
tcpIp = '10.77.2.211'   # This is to be the source of raw radar data. It can be a real radar or the address where 
                        # the Navtech ColossusNetrecordPlayback tool is running in playback mode
tcpPort = 6317          # This is the port that the radar is using, this generally will not need to be changed
########################################################################



########################################################################
# The below settings should not be changed
########################################################################
signatureLength = 16
versionLength = 1
messageTypeLength = 1
payloadLength = 4
payloadSize = 0
headerLength = signatureLength + versionLength + messageTypeLength + payloadLength
checkSignature = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe'
configRead = False
dataRead = False
encoderSize=0
version = 0
Power=[]
bearing=0
TimeStampWithNanoseconds=''
BitDepth=0
readFirstFFTMessage = False
azimuth = 0
startingAzimuth = 0
########################################################################



########################################################################
# The below section contains function definitions
########################################################################

# Send a Configuration Request Message - type 20
def configRequest():
    global socket
    global version
    stopDataMessage = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(20,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    socket.send(stopDataMessage)
    print("--------Sent Config Request---------")
    print("")

# Send a start FFT Data Request Message - type 21
def startFFTData():
    global socket
    global version
    startDataMessage = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(21,1, byteorder='big') + int.to_bytes(0,4, byteorder='big')
    socket.send(startDataMessage)
    print("----------Started FFT Data----------")
    print("")

# Send a stop FFT Data Request Message - type 22
def stopFFTData():
    global socket
    global version
    stopDataMessage = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(22,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    socket.send(stopDataMessage)
    print("----------Stopped FFT Data----------")
    print("")

# Read and process the response message
def handleMessage():
    global signatureLength
    global headerLength
    global checkSignature
    global payloadSize
    global configRead
    global dataRead
    global socket
    global version
    global encoderSize
    global Power
    global bearing
    global TimeStampWithNanoseconds
    global BitDepth
    global messageType
    global readFirstFFTMessage
    global azimuth
    global startingAzimuth
    global azimuthSamples
    global rangeBins

    # Read the message header
    messageType = 999
    try:
        data = socket.recv(signatureLength)
        if (data == checkSignature):
            data = socket.recv(headerLength-signatureLength)
            version = data[0]
            messageType = data[1]
            payloadSize = int.from_bytes(data[2:6], byteorder='big')
    except:
        print("Error reading message header")

    # Read the rest of the message
    if (messageType == 10): # Configuration data from radar
        data = socket.recv(payloadSize)
        if (len(data) == payloadSize):
            try:
                azimuthSamples = int.from_bytes(data[0:2], byteorder='big')
                rangeResolution = int.from_bytes(data[2:4], byteorder='big')
                rangeBins = int.from_bytes(data[4:6], byteorder='big')
                encoderSize = int.from_bytes(data[6:8], byteorder='big')
                rotationSpeed = int.from_bytes(data[8:10], byteorder='big')
                packetRate = int.from_bytes(data[10:12], byteorder='big')
                print("----------Config Message----------\n")
                print("Azimuth Samples:  {} samples/rotation".format(azimuthSamples))
                print("Range Resolution: {} mm/bin".format(rangeResolution/10))
                print("Range:            {} bins".format(rangeBins))
                print("Encoder Size:     {} counts/rotation".format(encoderSize))
                print("Rotation Speed:   {} mHz".format(rotationSpeed))
                print("Packet Rate:      {} azimuths/second".format(packetRate))
                configRead = True
                print("")
            except:
                print("Error reading config message")
    elif (messageType == 31 or messageType == 30): # Message type 31 is HighRes (16Bit) FFT Data 
                                                   # and message type 30 is 8bit data from the radar
        data = socket.recv(payloadSize)
        if (len(data) == payloadSize):
            try:
                counter = int.from_bytes(data[2:4], byteorder='big')
                azimuth = int.from_bytes(data[4:6], byteorder='big')
                bearing=360*(azimuth/encoderSize)
                seconds = int.from_bytes(data[6:10], byteorder='little')
                splitSeconds = int.from_bytes(data[10:14], byteorder='little')
                ts = datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')
                TimeStampWithNanoseconds="{}.{}".format(ts, str(int(round(splitSeconds/10000,0))).rjust(5,'0'))
                FFTDataBytes=data[14:]
                if (messageType ==31):
                    for index in range (int(len(FFTDataBytes)/2)):
                        Power.append(int.from_bytes(FFTDataBytes[index:index+2],byteorder='big'))
                    dataRead = True
                    BitDepth=16
                else:
                    for index in range ((len(FFTDataBytes))):
                        Power.append((int(FFTDataBytes[index])))
                    dataRead = True
                    BitDepth=8
                if not readFirstFFTMessage:
                    startingAzimuth = int(azimuth / encoderSize * azimuthSamples)
                    readFirstFFTMessage = True
            except:
                print("Error reading FFT message")
    else:
        print("Unhandled message type: {}".format(messageType))
########################################################################



########################################################################
# The below section contains the main program code
########################################################################

# Try to connect to the radar
try:
    print("Connecting to radar at {} on port {}".format(tcpIp, tcpPort))
    socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket.connect((tcpIp, tcpPort))
except:
    print("Unable to connect to radar at {} on port {}".format(tcpIp, tcpPort))
    exit()

# Try (for 5 seconds) to get a configuration message from the radar
print("Reading configuration message from radar")
timeout = time.time() + 5
while configRead == False:
    configRequest           # send a request configuration message to the radar
    time.sleep(1)           # just to be sure!
    handleMessage()         # read and interpret message response
    if time.time() > timeout:
        print("Unable to read configuration message from radar")
        exit()

# Send a message to the radar to instruct it to start sending FFT data
print("Starting FFT data")
startFFTData()

# Try to get an entire rotation of FFT data messages from the radar
print("Reading one rotation of FFT data from radar")
fftDataCaptured = 0
dataArray = np.zeros((azimuthSamples, rangeBins, 3))
handleMessage()             # Read the first FFT data
dataArray[int(azimuth / encoderSize * azimuthSamples), :len(Power), 1] = Power
fftDataCaptured += 1
Power.clear()
timeout = time.time() + 5
while int(azimuth / encoderSize * azimuthSamples) != startingAzimuth or fftDataCaptured < azimuthSamples:
    time.sleep(0.01)            # just in case we end up looping here
    handleMessage()             # Read the rest of the FFT data to make a complete rotation
    dataArray[int(azimuth / encoderSize * azimuthSamples), :len(Power), 1] = Power
    fftDataCaptured += 1
    Power.clear()
    if time.time() > timeout:
        print("Unable to read FFT data from radar")
        exit()

# Send a message to the radar to instruct it to stop sending FFT data
print("Stopping FFT data")
stopFFTData()

# Now show the captured rotation as a b-scan style image
dataArray = np.interp(dataArray, (dataArray.min(), dataArray.max()), (0, 255))
dataArray = dataArray.astype(np.uint8)
bScanPlot = plt.imshow(dataArray)
plt.title('B-Scan style image of complete rotation of FFT data')
plt.ylabel('Azimuth', fontsize=12)
plt.xlabel('Bin', fontsize=12)
plt.tight_layout()
print("Close b-scan plot to see polar plot")
plt.show()

# Now show the captured rotation as a polar plot style image
dataArrayScaled = cv2.resize(dataArray, ((400,400)), cv2.INTER_AREA)
polarDataArray =  cv2.linearPolar(dataArrayScaled, (int(dataArrayScaled.shape[1] / 2), int(dataArrayScaled.shape[0] / 2)), int(dataArrayScaled.shape[0] / 2), cv2.WARP_FILL_OUTLIERS + cv2.WARP_INVERSE_MAP)
polarPlot = plt.imshow(polarDataArray)
plt.title('Polar plot style image of complete rotation of FFT data')
plt.tight_layout()
print("Close polar plot to exit")
plt.show()

########################################################################