# CLM ROS WRAPPER

ROS wrapper for the [Cambridge face tracker (aka CLM framework)](https://github.com/TadasBaltrusaitis/CLM-framework). 
It works on catkin and it targets [ROS indigo](http://wiki.ros.org/indigo).

## 1. Dependencies

### 1.1 Prerequisites

```
sudo apt-get -y install libopencv-dev build-essential cmake git libgtk2.0-dev pkg-config python-dev python-numpy libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libtiff4-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils unzip
```

### 1.2 ROS `indigo`

[This guide](http://alecive.github.io/blog/2015/11/12/ROS-naive-installation/) is what I usually recommend, but also [this link](http://wiki.ros.org/indigo/Installation/Source) is equally useful (plus, it's the official guide).

### 1.3 OpenCV > 3.0.0

#### 1.3.1 Download OpenCV

Download OpenCV from [opencv.org](http://opencv.org/). I used the 3.1.0 version available [here](https://github.com/Itseez/opencv/archive/3.1.0.zip).

#### 1.3.2 Install OpenCV

 * Unzip the archive in a suitable directory (we will be using `~/src/`)
 * `cd ~/src/opencv-3.1.0`
 * `mkdir build`
 * `cd build`
 * `cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_QT=ON -D WITH_OPENGL=ON ..`
 * `make -j7`

#### 1.3.3 Finish installation

To get OpenCV working properly, you need to edit your `~/.bashrc` file by adding a proper `OpenCV_DIR`, to let the system know where OpenCV 3.* is compiled. Just add this line at the end of said file:

```
export OpenCV_DIR=~/src/opencv-3.1.0/build
```

## 2. Download and configure

### 2.1 Download the repo in a suitable catkin workspace

```
cd /your_ws/src
git clone https://github.com/sociallyassistiverobotics/clm_ros_wrapper.git
```

## 2.2 Compilation and Installation

If everything is ok , simply `cd` in the catkin workspace, and type:

```
cd /your_ws/
catkin_make
rospack profile
```

## 3. Usage

The default usage is pretty simple. Just be sure that there is a `roscore` instance in your local network, and then launch the node with `roslaunch`, provided that a `roscore` instance is already running:

 Terminal 1 | Terminal 2                                           
------------|-------------------------------------------------------
 `roscore`  | ` roslaunch clm_ros_wrapper clm_ros_wrapper.launch ` 

