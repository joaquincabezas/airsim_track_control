# airsim_track_control
Controlling AirSim vehicles with webcam and object tracking

Goal: Control the movement of the AirSim car with a camera and a tracked object

Purpose: Play with my kids

Installation:
 1. Install Airsim scenario (Neighborhood has ready to use binaries, see below)
 2. Install airsim python package (https://pypi.org/project/airsim/)
 3. Install OpenCV2 (https://pypi.org/project/opencv-python/)

Instructions:
 1. Open an AirSim scenario
      1.1 Edit run.bat to "start AirSimNH -windowed -ResX=640 -ResY=480")
 2. Select the color of the object in the code, in the module variables
 3. Execute this script and check the camera is properly working
 4. Use a object to control the car:
      4.1 Centered object: No movement
      4.2 Vertical axis: Throttle (up makes the car go forward, down goes backwards)
      4.3 Horizontal axis: Steer to the right or left

Tested Neighborhood binary for Windows:
  https://github.com/microsoft/AirSim/releases/download/v1.3.0-Windows/Neighborhood.zip
  
Object Tracking based on https://github.com/jonathanfoster/ball-tracking/

Airsim controls based on https://microsoft.github.io/AirSim/apis/#hello-car
