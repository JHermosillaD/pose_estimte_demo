# ROS package for human detection and position estimation
 
![Static Badge](https://img.shields.io/badge/ros%20-%20noetic%20-blue) ![GitHub repo size](https://img.shields.io/github/repo-size/jhermosillad/pose_estimate_demo)
 ![GitHub top language](https://img.shields.io/github/languages/top/jhermosillad/pose_estimate_demo) 

ROS package for the detection of persons using histograms of oriented gradients and Haar cascades and the estimation of their position by transforming the coordinates of their bounding box into xyz coordinates with respect to the base_link reference frame.

## Set-up
### Requirements
- [x] ROS melodic or higher
- [x] OpenCV
- [x] [HOG detector package](https://github.com/JHermosillaD/hog)
- [x] [Haar cascade detector package](https://github.com/JHermosillaD/haar_cascade)
- [x] [Bounding Box position estimator package](https://github.com/JHermosillaD/bbox_position)
- [x] [Marker package](https://github.com/JHermosillaD/pedestrian_space)
      
### Installation
Clone the repository to the workspace source path.
```
user@hostname:~/workspace/src$ git clone https://github.com/JHermosillaD/pose_estimate_demo.git
```
Compile the package.
```
user@hostname:~/workspace$ catkin_make
```
## Usage

Run the launcher.
```
user@hostname:~/workspace$ roslaunch pose_estimate_demo demo.launch
```
## Visualization
The image containing the bounding box, the marker and coordinate frames can be displayed in Rviz, the coordinates through topic command line.

<img width="480" height="360" src="/test.gif">
