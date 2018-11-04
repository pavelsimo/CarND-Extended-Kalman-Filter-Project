# CardND - Extended Kalman Filter Project

The goal of this project is to utilize extended kalman filters to estimate the state of a moving 
object of interest with noisy lidar and radar measurements. 

[//]: # (Image References)
[img1]: ./data/ekf_project.png

My project includes the following files:

* ```kalman_filter.cpp``` extended kalman filter implementation
* ```FusionEKF.cpp``` class for running the kalman filter
* ```tools.cpp``` utility functions (e.g. CalculateRMSE)

![EKF Project][img1]

## Dependencies

* [Udacity Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

