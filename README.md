# Psyonic Hand ROS

This repository contains a collection of packages to operate the PSYONIC Ability Hand in ROS.

## Usage

The hand can be controller via a serial connection or BLE.
BLE has a limited interface; only position control is possible and the control loop runs at 20 Hz.
Via serial connection, position, velocity, torque and voltage control are available, and the control loop runs at 63 Hz.

It is possible to connect both via serial and BLE at the same time.
However, the BLE connection can slow down the serial connection, so this is not recommended.

### Serial connection

Start the hand driver:

```
roslaunch psyonic_hand_driver hand_control.launch
```

By default, the node connects to the hand on the serial port `/dev/ttyUSB0`.
You may either specify a different port with the `port` argument, or use the `id` argument to specify the device ID as listed in `/dev/serial/by-id`.
This is useful if you have multiple serial devices, which may not always use the same port.

### BLE

```
roslaunch psyonic_hand_driver hand_control.launch connect_serial:=false connect_ble:=true
```

## Control GUI

Run rqt and start the plugin at Plugins -> Robot Tools -> Psyonic Hand.
Here, you can switch control and reply modes, and manually send joint commands to the hand.

## Mimic Hand Demo

We have a demo to let the hand mimic positions captured from a camera stream.
The node uses mediapipe to capture hand landmarks.
Our launch file assumes you have a virtual environment called `mediapipe` with mediapipe installed set up, and the [trained model](https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task) for hand landmarker detection (the default path is the folder "models" in the hand_mimicker package).
To test mimicking a hand from a webcam, run:

```
roslaunch hand_mimicker mimic_from_webcam.launch
```

For a different setup, you can modify the launch file.
For more information on setting up mediapipe, check out the [documentation](https://ai.google.dev/edge/mediapipe/solutions/guide), especially the [hand landmarker guide](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker).

You may visualize the detected landmarks by opening an instance of the image view plugin in rqt and subscribe to the `/mimic_hand/visualization` topic.
In the psyonic hand rqt plugin, set the control mode to "Position" to start mimicking the detected hand.