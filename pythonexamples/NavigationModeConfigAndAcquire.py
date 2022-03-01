import matplotlib.cm as cm       # colourmap
import matplotlib.pyplot as plt
import socket
import struct
import numpy as np
import time
import datetime
import os

#########################################################################
### NOTE that this script requires radar firmware 3.0.0.131 or higher ###
#########################################################################



########################################################################
tcpIp = '192.168.0.1' #this is to be the address of a /real/ radar  
                      # The Navtech Radar data replay tool does not 
                      # produce NavigationMode output so cannot be used
                      # as the data source for this example.
tcpPort = 6317
########################################################################

    

maxRangeOfInterest=200# in metres: this value is used to discard points beyond a certain range
                      # usually, a customer will set the radar's operating range through the radar's Web UI

Bins=5                # number of bins to operate on (window size) for peakfinding

MinBins=3             # closest bin to consider within the peak finding.
                      # this allows for close-in radar returns to be ignored

NavThreshold=55       # power threshold used to configure the operation of the 
                      # NavigationMode. Note that setting this value too low can 
                      # result in the processing effort required from the radar
                      # exceeding that which is available. This will result in
                      # points appearing to be "missing" from ranges of azimuths
                      # in the returned dataset

MaxPeaks=10            # the number of peaks per azimuth that NavigationMode should report

########################################################################
signatureLength = 16
versionLength = 1
messageTypeLength = 1
payloadLength = 4
headerLength = signatureLength + versionLength + messageTypeLength + payloadLength
checkSignature = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe'
configRead = False
version = 0
messageType = 0
payloadSize = 0
azimuth = 0
encoderSize = 0
packetRate = 0
azimuthSamples = 0
firstRun = True
rangeList=[]
rangeList=[]
bearingList=[]
powerList=[]
NavMessageCount=0
NavConfigRead=False
TimeStampSecondsAndFractionalSeconds=0.0
FirstTimeStampSecondsAndFractionalSeconds=0.0
FirstMessage=True
rotationSpeed=4000

def configRequest():    # Send a Configuration Request Message - type 20
    global socket
    global version
    stopDataMessage = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(20,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    socket.send(stopDataMessage)
    print("----------- Requested Config -----------")
    print("")

def startNavData():     # Send a start Navigation Data Request Message - type 120
    global socket
    global version
    startNavMessage = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(120,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    socket.send(startNavMessage)
    print("---------- Requested Nav Data ----------")
    print("")

def stopNavData():      # Send a stop Navigation Data Request Message - type 121
    global socket
    global version
    stopNavMessage = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(121,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    socket.send(stopNavMessage)
    print("----------- Stopped Nav Data -----------")
    print("")

#def setNavThreshold(thresholddB):     # Send a set Navigation threshold Message - type 122 - message type not required for radars with firmware 3.0.0.131 or later.
#    global socket
#    global version
#    setNavThresholdMessage = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(122,1, byteorder='big')  + int.to_bytes(2,4, byteorder='big')+ int.to_bytes(int(thresholddB*10),2, byteorder='big')
#    socket.send(setNavThresholdMessage)
#    print("----------Set Nav Threshold {}----------".format(thresholddB))
#    print("")

# Send a set Navigation configuration Message - type 205
def setNavConfig(BinsToOperateOn,MinimumBin,NavigationThreshold,MaxPeaksPerAzimuth):
    global socket
    global version
    setNavConfigMessage = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(205,1, byteorder='big')  + int.to_bytes(12,4, byteorder='big')+ int.to_bytes(int(BinsToOperateOn),2, byteorder='big')+ int.to_bytes(int(MinimumBin),2, byteorder='big')+ struct.pack('>f',NavigationThreshold*10) + int.to_bytes(int(MaxPeaksPerAzimuth),4, byteorder='big')
    socket.send(setNavConfigMessage)
    print("------ Set Nav Config (Message205) -----")
    print("     Bins      {}".format(BinsToOperateOn))
    print("     MinBin    {}".format(MinimumBin))
    print("     Threshold {}".format(NavigationThreshold))
    print("     MaxPeaks  {}".format(MaxPeaksPerAzimuth))
    print("")

def getNavConfig():     # Send a get Navigation configuration Message - type 203
    global socket
    global version
    getNavConfigMessage = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(203,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    socket.send(getNavConfigMessage)
    print("-------- Requested Nav Config ----------")
    print("")

def handleMessage():    # Read and process the message
    global signatureLength
    global headerLength
    global checkSignature
    global configRead
    global NavConfigRead
    global socket
    global version
    global payloadSize
    global encoderSize
    global azimuthSamples
    global NavMessageCount
    global TimeStampSecondsAndFractionalSeconds
    global FirstTimeStampSecondsAndFractionalSeconds
    global FirstMessage
    global rotationSpeed
    global messageType
    # Read header
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
                print("------------ Config Message ------------")
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

    if (messageType == 204): # Navigation Configuration data from radar
        data = socket.recv(payloadSize)
        if (len(data) == payloadSize):
            try:
                Bins = int.from_bytes(data[0:2], byteorder='big')
                MinBin = int.from_bytes(data[2:4], byteorder='big')
                [Threshold] = struct.unpack('>f',(data[4:8]))
                MaxPeaks = int.from_bytes(data[8:12], byteorder='big')
                print("-- Get Navigation Config (Message204) --")
                print("     Bins      {}".format(Bins))
                print("     MinBin    {}".format(MinBin))
                print("     Threshold {}".format(Threshold/10))
                print("     MaxPeaks  {}".format(MaxPeaks)) 
                print("") 
                NavConfigRead = True
            except:
                print("Error reading navigation configuration message")
    elif (messageType == 123): # Navigation Data from the radar
        data = socket.recv(payloadSize)  
        if (len(data) == payloadSize):
            try:
                azimuth = int.from_bytes(data[0:2], byteorder='big')
                bearing=360*(azimuth/encoderSize)
                seconds = int.from_bytes(data[2:6], byteorder='big')
                splitSeconds = int.from_bytes(data[6:10], byteorder='big')
                navData = list(data[10:])
                TimeStampSecondsAndFractionalSeconds=seconds+(splitSeconds/1000000000)
                for index in range (int(len(navData)/6)):
                    byteOffset=index*6
                    peakRange =int.from_bytes(navData[0+byteOffset:4+byteOffset],byteorder='big')
                    peakPower = int.from_bytes(navData[4+byteOffset:6+byteOffset],byteorder='big')
                    peakRangeMetres=peakRange/1000000
                    peakPowerdB=peakPower/10
                    if (peakRangeMetres<=maxRangeOfInterest)and(peakRangeMetres>=3):
                        rangeList.append(peakRangeMetres)
                        bearingList.append((bearing/360)*2 * np.pi)
                        powerList.append(peakPowerdB) 
                if FirstMessage==True :
                    FirstTimeStampSecondsAndFractionalSeconds = TimeStampSecondsAndFractionalSeconds
                    FirstMessage=False
                NavMessageCount+=1                       
            except:
                print("Error reading nav message")
    else:
        ("Unhandled message type: {}".format(messageType))

######################################################################################

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect((tcpIp, tcpPort))
TimeStamp=datetime.datetime.now()
filepath=(TimeStamp.strftime('RadarData/%Y/%m/%d/'))	# the folder path that contains the log file
if (not os.path.exists(filepath)):	# if the folder isn't there
	
	os.makedirs (filepath)			# ... then create it

filename=(filepath+(TimeStamp.strftime('NavigationModeData_%H-%M-%S.csv'))) # filename is made up of fixed string and Day-number in month 

# Request and read the radar configuration
while configRead == False:     # this is a flag to note whether we have recieved the radar's config
    configRequest              # send a request configuration message to the radar
    time.sleep(1)              # just to be sure!
    handleMessage()            # read and interpret message response

setNavConfig(Bins,MinBins,NavThreshold,MaxPeaks)   # send the required navigation mode configuration to the radar

# read the current set navigation mode settings back from the radar
while NavConfigRead == False:  # this is a flag to note whether we have recieved the radar's config
    getNavConfig()             # send a request NavigationMode configuration message to the radar
    time.sleep(1)              # just to be sure!
    handleMessage()            # read and interpret message response

startNavData()                 # Request Navigation Data messages from the radar

#for raw FFT data we know to expect a certain number of messages per rotation (defined by rotation-speed and packet-rate. For Navigation mode, the number of messages per rotation cannot be defined (as it is dependent on whether peaks are identified or not). THe loop below is set up to execute for messages covering at least the time taken for one rotation of the radar's antenna. 
while (TimeStampSecondsAndFractionalSeconds-FirstTimeStampSecondsAndFractionalSeconds)<(1000/rotationSpeed):
    handleMessage()
stopNavData()                   # Instruct the radar to stop sending Navigation Mode data


target = open(filename,'w')		# ...then open it for writing ...
target.write("Date,{}\n".format(TimeStamp.strftime('%Y/%m/%d')))
target.write("Time,{}\n".format(TimeStamp.strftime('%H:%M:%S')))
target.write("Threshold,{}\n".format(NavThreshold))
target.write("BinsToOperateOn,{}\n".format(Bins))
target.write("MinBin,{}\n".format(MinBins))
target.write("PeaksToReturn,{}\n".format(MaxPeaks))
target.write("Notes\n")
target.write("Range(m),Bearing(rad),Power\n")
for i in range (len(powerList)):
    target.write("{},{},{}\n".format(rangeList[i],bearingList[i],powerList[i]))
target.close

print ("     Peaks Indentified: {}".format(len(powerList)))
fig = plt.figure("One Rotation of Navigation Mode Data")
fig.set_size_inches(9.,10.)
ax = fig.add_subplot(projection='polar')
ax.set_ylim(0,maxRangeOfInterest)
c = ax.scatter(bearingList, rangeList, s=18, c=powerList, cmap=cm.rainbow,vmin=min(powerList), vmax=max(powerList))
ax.set_title('Navigation Mode Data. BinsToOperateOn={}, MinBin={}, Threshold={}, MaxPeaks={}'.format(Bins, MinBins,NavThreshold,MaxPeaks))
ax.set_theta_direction(-1)
ax.set_theta_offset(np.pi / 2.0)
plt.tight_layout()
plt.savefig(filepath+(TimeStamp.strftime('NavigationModeData_%H-%M-%S.pdf')),dpi=2000)
plt.show()
