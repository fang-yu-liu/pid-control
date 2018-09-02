# Autonomous Driving using PID Controller
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This project is part of the [Udacity Self-Driving Car Nanodegree](https://www.udacity.com/drive) program, and some of the code are leveraged from the lecture materials.

---
## Objective
This project uses a PID controller to control the steering angle and throttle of a vehicle in a simulator. Cross track error (CTE) is provided by the simulator and the PID controller is responsible of controlling the vehicle to drive smoothly along the track in the simulator.

## PID Controller Overview
A proportional-integral-derivative (PID) controller is a widely used control mechanism. It continuously calculates the difference between the desired setpoint and the measured value and applies correction based on proportional, integral and derivative terms.

* P (proportional) - Apply correction proportional to the current error value. System with large proportional gain is more responsive and sensitive but might become unstable if the gain is too large. System with small proportional gain will be less sensitive but more vulnerable to disturbances.
* I (Integral) - Apply correction to account for the all past accumulated errors. The integral term can help with eliminating the residual steady-state error but will also introduce overshoot and instability.
* D (Derivative) - Apply correction to account for the future trend of the error based on the current change rate. The derivative term helps damping the system and reducing overshoot.

## Parameter Tuning
The parameters are tuned manually. The tuning process are described below:
1. Kp = 0, Ki = 0, Kd = 0 - The vehicle is unable to follow the track and drive out of the lane line on the first corner.
2. Slowly increasing Kp, Ki = 0, Kd = 0 - When Kp reaches 0.1, the vehicle is able to follow the track but will sometimes overshoot and drive out of the lane line. Increasing Kp even more doesn't seem to help with the performance.
3. Kp = 0.1, slowly increasing Kd, Ki = 0 - With larger Kd, the overshoot behavior is improved a lot with Kd = 12, the vehicle is able to stay within the two lane lines all around the track.
4. Kp = 0.1, Kd = 12, slowly increasing Ki - Finally, adding a small integral gain to help eliminating steady-state error to make the vehicle driving around the center line of the track.

The PID controller for the throttle is tuned using similar process.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.
