# ARUCO_Client

This package is sought to work as a client for the work as written in some documentation. And the detection. It is part of the ROS-implementation of the [numerical simulation](https://github.com/NicoMandel/Numerical-Sim-Semantic)
This package co-exists with the [knowledgebase package]() and is sought to handle image-based issues.

## Dependencies
* The [ml_detector package](https://github.com/qutas/marker_localization) from qutas github
* `cv_bridge` - good luck
* This package is used by the knowledgebase for detection
* Px4 Firmware
* A proprietary camera model for the UAV to use (not included)
* Numpy and Pandas

## Known issues
* the camera model of px4 is not recommended.
* python2 and python3 issues - change shebangs where necessary
