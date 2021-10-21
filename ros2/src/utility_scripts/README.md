# utility_scripts

The utility_scripts folder contains some *helper scripts*, used to manipulate radar and camera output videos, and to part analyse radar data. The following utility scripts are included:

*Note - these utility scripts must be manually edited, they contain no settings files*

## join_radar_camera_videos

Contains a bash script which uises Ffmpeg to join together the radar video output and the camera video output, for synchronised, side-by-side viewing.

Please see README.md in the ROS2 folder for Ffmpeg installation instructions

Example script usage:

	sudo chmod +x join_radar_camera_videos

	./join_radar_camera_videos

## join_radar_camera_videos.py

Contains a Python script which uises OpenCV to join together radar video output and the camera video output, for synchronised, side-by-side viewing.

Please see README.md in the ROS2 folder for Python and OpenCV installation instructions

Example script usage:

	python3 join_radar_camera_videos.py

## show_colossus_timestamps.py

Contains a Python script to open a ROS bag file and examine the timestamps on the radar FFT data messages.

Example script usage:

	python3 show_colossus_timestamps.py