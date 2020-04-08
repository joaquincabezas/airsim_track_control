#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
  airsim_track_control.py: Controls AirSim car simulation with an object tracked in camera.
  Author: Joaquin Cabezas github.com/joaquincabezas
  Date: 08/04/2020

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
"""

import time
import cv2
import airsim

# The HSV colors can be found in https://alloyui.com/examples/color-picker/hsv.html
# Toy Lemon
COLOR_LOWER = (20, 100, 100)
COLOR_UPPER = (30, 255, 255)

# Toy Lettuce
#COLOR_LOWER = (40, 40, 6)
#COLOR_UPPER = (70, 255, 255)

# Dimensions of the webcam capture screen (in pixels)
DIM_X = 640
DIM_Y = 480

# Smallest size of the tracked object (in pixels)
THRESHOLD_CONTOUR = 10

# FORWARD_RATIO tells how many times forward is faster than backwards
FORWARD_RATIO = 2

# Needed difference in throttle or steering to update the controls
CONTROL_THROTTLE_THRESHOLD = 0.1
CONTROL_STEERING_THRESHOLD = 0.1

# Rate in seconds
REFRESH_RATE = 0.1

EXIT_KEY = 'x'

def connect_airsim():
    """Connect to to the airsim simulator.

    The airsim simulator must be running prior to the execution of this script,
    otherwise it will throw an error. Also, it must be in "car" mode.
    Download airsim from: https://github.com/microsoft/AirSim/releases

    Returns:
        client: AirSim object representing the connection to the simulator
        car_controls: AirSim object to control the car


    """
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    car_controls = airsim.CarControls()
    car_controls.is_manual_gear = False

    return client, car_controls

def coordinates_to_controls(coordinate_x, coordinate_y):
    """ Transform tracking coordinates to control commands of AirSim.

    Args:
    x (int): x coordinate of the tracked object.
    y (int): y coordinate of the tracked object.

    """

    # The simplest transformation based on location
    throttle = -(coordinate_y-(DIM_Y/2))/DIM_Y
    steering = -(coordinate_x-(DIM_X/2))/DIM_X

    # TODO: Homography for perspective (after calibration) # pylint: disable=fixme

    # Throttle is FORWARD_RATIO times faster when going forward (with regard to backward)
    if throttle > 0:
        throttle = FORWARD_RATIO*throttle

    return throttle, steering

def send_controls(client, car_controls, status_controls, throttle, steering):
    """ Send controls to the AirSim simulator

    Args:
    client (int): AirSim client object
    car_controls (int): AirSim object to control the car
    throttle (int): 0 means stopped. Positive forward. Negative backwards
    steering (int): Positive right steering. Negative left steering

    """
    # To avoid flooding the simulator, we only send commands with a certain
    # difference to the current actions (ie hysteresis)

    diff_throttle = abs(status_controls['throttle']-throttle) > CONTROL_THROTTLE_THRESHOLD
    diff_steering = abs(status_controls['steering']-steering) > CONTROL_STEERING_THRESHOLD

    if diff_throttle or diff_steering:

        car_controls.throttle = throttle
        car_controls.steering = steering

        if throttle < 0:
            car_controls.is_manual_gear = True
            car_controls.manual_gear = -1
        else:
            car_controls.is_manual_gear = False

        client.setCarControls(car_controls)
        status_controls['throttle'] = throttle
        status_controls['steering'] = steering

    return status_controls



def main():
    """ Main function for the track_control script

    The flow of the script is the following:
        · Connecto to AirSim
        · Connect to webcam
        · Track in loop the position of an object based in the color
        · Send control actions depending upon the tracking on the object
        · Waits for 'x' to be terminated

    """

    # Connect to the AirSim simulator
    client, car_controls = connect_airsim()

    # Connect to the first available camera
    camera = cv2.VideoCapture(0)

    # We initialize the controls
    status_controls = {}
    status_controls['throttle'] = 0
    status_controls['steering'] = 0

    # Once we have camera and simulator, we start the loop
    while True:
        # We read the camera image
        (_grabbed, frame) = camera.read()

        # We convert the image to HSW, which makes easier color range identification
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # We create a mask for just the range color we want to track
        mask = cv2.inRange(hsv, COLOR_LOWER, COLOR_UPPER)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # We fin a number of contours on the mask
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(contours) > 0:
            # We pick the biggest contour and find its center
            contour = max(contours, key=cv2.contourArea)
            ((coordinate_x, coordinate_y), radius) = cv2.minEnclosingCircle(contour)

            # Given the contour is significative
            if radius > THRESHOLD_CONTOUR:
                cv2.circle(frame, (int(coordinate_x), int(coordinate_y)),
                           int(radius), (0, 255, 255), 2)

                # Now, using the coordinates of the tracked object, we set the controls of the car
                throttle, steering = coordinates_to_controls(coordinate_x, coordinate_y)
                status_controls = send_controls(client, car_controls, status_controls,
                                                throttle, steering)
        else:
            # If no object is tracked, we stop the car
            status_controls = send_controls(client, car_controls, status_controls, 0, 0)

        # To ease self identification, we flip the image so it acts as a mirror
        frame = cv2.flip(frame, 1)

        cv2.imshow("Frame", frame)

        # Finish execution by pressing the EXIT_KEY
        if (cv2.waitKey(1) & 0xFF) == ord(EXIT_KEY):
            break

        # Sample the camera every REFRESH_RATE seconds
        time.sleep(REFRESH_RATE)

    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
