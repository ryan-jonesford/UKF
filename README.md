# Unscented Kalman Filter
Self-Driving Car Engineer Nanodegree Program

This project utilizes a Kalman filter to estimate the state of a moving object of interest with noisy Lidar and radar measurements. 

*This project requires the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)*

*This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.*

*Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.*

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./KF [OPTIONS]

---

## Other Important Dependencies

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

# Running the program
### SYNOPSIS

       KF [OPTIONS]... 

### DESCRIPTION
       Runs a Kalman filter to be used in the Udacity Term2 Simulator, runs unscented Kalman filter by default if no options are given

       -v
              Verbose mode. Prints out an obnoxious amount of debug statements to the console

       -e
              Run an extended kalman filter

       -r
              Use Radar only

       -l
              Use Laser only

       -f
              Print to log file ./out_log.csv

       -n
              Print NIS info to console

       -hunt 
              Used for the catch the runaway car simulation

# Program Evaluation
## Accuracy
The RSME values for px,py,vx,vy vs ground truth settle out to less than 0.07,0.09,0.3 and 0.3 respectively. The percentage of NIS values for  radar below 7.81 is almost 96% and for Laser that are below 5.99 is almost 98%.  

## Program flow
The program follows the flow as follows for the Unscented Kalman Filter:
The first measurement of the position is recorded as reported to initialize the state vector, converting polar coordinates to Cartesian if the first measurement came from Radar, and on subsequent time steps making predictions using the previous values and timestep. The covarience matricies are also created when the first measurement comes in and are different if it is a laser or a radar measurement. After the prediction is made the update step uses different functions depending on the sensor type. Since they see the world differently, the measurements need to be handled differently.

The Lidar measurement uses a linear update method whereas the radar uses a non-linear one, necessitating the need for creating sigma points.

## Code Efficiency
The code strives to be clean and readable, and to avoid loops, and creating unnecessary complexity whether through the use of loops or control flow checks. 

## Catching the runnaway car
There are many strategies to do this. I chose a simple one to implement. It takes the first place that the car was detected and sits there and waits for it to come back around. Once the car gets close enough it drives towards it and catches it. It is able to opperate with a large amount of error in the measurements when using the UKF and is also successful with the EKF (to run with EKF run './Kf -hunt -e')

## Comparisons
The UKF finds itself to be more accurate than the EKF, in fact using the laser values only, the UKF is as accurate as the EKF. 

The following table shows the results (gt = ground truth):

| Filter        | px vs gt  | py vs gt | vx vs gt | vy vs gt |
| ------------- |:---------:| --------:| --------:| --------:|
| UKF           | .065      |   .083   |   .277   |   .277   |
| EKF           | .096      |   .085   |   .415   |   .432   |
| UKF Laser only| .096      |   .097   |   .462   |   .288   |
| UKF Radar only| .156      |   .224   |   .425   |   .676   |


Note: *Italicized* sections and the "Other Important Dependencies" section were written by Udacity. The majority of code in main.cpp was written by Udacity.
