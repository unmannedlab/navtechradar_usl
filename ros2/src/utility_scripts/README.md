# utility_scripts

The utility_scripts folder contains some **helper scripts**, used to manipulate radar and camera output videos, and to part analyse radar data. The following utility scripts are included:

## join_radar_camera_videos

Contains a bash cript which uises FFMPEG to join together the radar video output and the camera video output, for synchronised viewing in a side-by-side mannor.

Please see README.md in the ROS2 folder for FFMPEG installation instructions

## join_radar_camera_videos.py

Contains a Python cript which uises OpenCV to join together the radar video output and the camera video output, for synchronised viewing in a side-by-side mannor.

Please see README.md in the ROS2 folder for Python and OpenCV installation instructions


## show_colossus_timestamps.py

Contains a Python script to open a ROS bag file, and examine the timestamps on the radar FFT data messages.