# Imports
import cv2
import os.path
from pathlib import Path
import numpy as np

# Globals
currentDirectory = os.getcwd()
videoDirectory = Path(currentDirectory).parents[1]
cameraVideoPath = str(videoDirectory) + '/output_videos/camera_output.avi'
radarVideoPath = str(videoDirectory) + '/output_videos/radar_output.avi'
outputVideoPath = str(videoDirectory) + '/output_videos/combined_output.avi'
outputVideoSize = (800, 400)
framesProcessed = 0

# Main
print("Current Directory: {0}".format(currentDirectory))
print("Video Directory: {0}".format(videoDirectory))
if (not os.path.isfile(cameraVideoPath)):
    print("Camera video file does not exist")
    quit()
if (not os.path.isfile(radarVideoPath)):
    print("Radar video file does not exist")
    quit()

cameraCapture = cv2.VideoCapture(cameraVideoPath)
radarCapture = cv2.VideoCapture(radarVideoPath)
outputVideo = cv2.VideoWriter(outputVideoPath, cv2.VideoWriter_fourcc('M','J','P','G'), 4, (outputVideoSize[0], outputVideoSize[1]))

while(cameraCapture.isOpened() and radarCapture.isOpened()):
    retCamera, frameCamera = cameraCapture.read()
    retRadar, frameRadar = radarCapture.read()
    
    if retCamera == True and retRadar == True:
        frameCameraResized = cv2.resize(frameCamera, (int(outputVideoSize[0]/2), outputVideoSize[1]), cv2.INTER_NEAREST)
        frameRadarResized = cv2.resize(frameRadar, (int(outputVideoSize[0]/2), outputVideoSize[1]), cv2.INTER_NEAREST)
        combinedFrame = np.hstack([frameCameraResized, frameRadarResized])
        outputVideo.write(combinedFrame)
        framesProcessed += 1
        if (framesProcessed % 240 == 0):
            print("Processed: {0} minute(s) of video".format(framesProcessed / 240))

    else:
        cameraCapture.release()
        radarCapture.release()
        outputVideo.release()
        print("Done processing video")
        break