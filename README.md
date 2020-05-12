# SC-LeGO-LOAM-BOR

This is a fork of [LeGO-LOAM-BOR](https://github.com/facontidavide/LeGO-LOAM-BOR), that is itself a fork of the original [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM). The purpose of this new fork is to add the Scan Context loop detector implemented in [SC-LeGO-LOAM](https://github.com/irapkaist/SC-LeGO-LOAM). Hence, the advantages of SC-LeGO-LOAM (second loop closure detector) and LeGO-LOAM-BOR (determinism, computing effectiveness) are combined in a single package SC-LeGO-LOAM-BOR.

# About SC-LeGO-LOAM

## Real-time LiDAR SLAM: Scan Context (18 IROS) + LeGO-LOAM (18 IROS)
- This repository is an example use-case of Scan Context C++ , the LiDAR place recognition method, for LiDAR SLAM applications.
- For more details for each algorithm please refer to:
    - Scan Context https://github.com/irapkaist/scancontext
    - LeGO LOAM https://github.com/RobustFieldAutonomyLab/LeGO-LOAM
- Just include Scancontext.h. For details see the file mapOptmization.cpp.
- This example is integrated with LOAM, but our simple module (i.e., Scancontext.h) can be easily integrated with any other key-frame-based odometry (e.g., wheel odometry or ICP-based odometry).
- Current version: April, 2020.

## Features
- Light-weight: a single header and cpp file named "Scancontext.h" and "Scancontext.cpp"
	- Our module has KDtree and we used nanoflann. nanoflann is an also single-header-program 			and that file is in our directory.
- Easy to use: A user just remembers and uses only two API functions: 	  makeAndSaveScancontextAndKeys and detectLoopClosureID.
- Fast: The loop detector runs at 10-15Hz (for 20 x 60 size, 10 candidates)

## Scan Context integration
- For implementation details, see the mapOptmization.cpp; all other files are same as the original LeGO-LOAM.
- Some detail comments
     - We use non-conservative threshold for Scan Context's nearest distance, so expect to maximise true-positive loop factors, while the number of false-positive increases.
     - To prevent the wrong map correction, we used Cauchy (but DCS can be used) kernel for loop factor. See mapOptmization.cpp for details. (the original LeGO-LOAM used non-robust kernel). We found that Cauchy is emprically enough.
     - We use both two-type of loop factor additions (i.e., radius search (RS)-based as already implemented in the original LeGO-LOAM and Scan context (SC)-based global revisit detection). See mapOptmization.cpp for details. SC is good for correcting large drifts and RS is good for fine-stitching.
     - Originally, Scan Context supports reverse-loop closure (i.e., revisit a place in a reversed direction) and examples in here (py-icp slam) . Our Scancontext.cpp module contains this feature. However, we did not use this for closing a loop in this repository because we found PCL's ICP with non-eye initial is brittle.


# About LeGO-LOAM-BOR
This is a "friendly fork", in other words, we will be happy to work with the original authors to merge
these improvements with the original LeGO-LOAM... if they want to!

The original author deserves all the credits, we just use good software engineering practices to
make the code more readable and efficient.

The purpose of this fork is:

- To improve the quality of the code, making it more readable, consistent and easier to understand and modify.
- To remove hard-coded values and use proper configuration files to describe the hardware.
- To improve performance, in terms of amount of CPU used to calculate the same result.
- To convert a multi-process application into a single-process / multi-threading one; this makes the algorithm
  more deterministic and slightly faster.
- To make it easier and faster to work with rosbags: processing a rosbag should be done at maximum
  speed allowed by the CPU and in a deterministic way (usual speed improvement in the order of 5X-10X).
- As a consequence of the previous point, creating unit and regression tests will be easier.

The purpose of this fork (for the time being) is **not** to modify and/or improve the original algorithm.

Please do not submit to this repository any issue related to the algorithm, since we are focusing on the
software implementation.

# About the original LeGO-LOAM
This repository contains code for a lightweight and ground optimized lidar odometry and mapping (LeGO-LOAM) system for ROS compatible UGVs.
The system takes in point cloud  from a Velodyne VLP-16 Lidar (palced horizontal) and optional IMU data as inputs.
It outputs 6D pose estimation in real-time. A demonstration of the system can be found here -> https://www.youtube.com/watch?v=O3tz_ftHV48
<!--
[![Watch the video](/LeGO-LOAM/launch/demo.gif)](https://www.youtube.com/watch?v=O3tz_ftHV48)
-->
<p align='center'>
    <img src="/LeGO-LOAM/launch/demo.gif" alt="drawing" width="800"/>
</p>

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with indigo and kinetic)
- [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)
  ```
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.0-alpha2/
  mkdir build && cd build
  cmake ..
  sudo make install
  ```

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/robot54/SC-LeGO-LOAM-BOR.git
cd ..
catkin_make
```

## The system

LeGO-LOAM is speficifally optimized for a horizontally placed lidar on a ground vehicle. It assumes there is always a ground plane in the scan. The UGV we are using is Clearpath Jackal.

<p align='center'>
    <img src="/LeGO-LOAM/launch/jackal-label.jpg" alt="drawing" width="400"/>
</p>

The package performs segmentation before feature extraction.

<p align='center'>
    <img src="/LeGO-LOAM/launch/seg-total.jpg" alt="drawing" width="400"/>
</p>

Lidar odometry performs two-step Levenberg Marquardt optimization to get 6D transformation.

<p align='center'>
    <img src="/LeGO-LOAM/launch/odometry.jpg" alt="drawing" width="400"/>
</p>

## New sensor and configuration

To customize the behavior of the algorithm or to use a lidar different from VLP-16, edit the file **config/loam_config.yaml**.

One important thing to keep in mind is that our current implementation for range image projection is only suitable for sensors that have evenly distributed channels.
If you want to use our algorithm with Velodyne VLP-32c or HDL-64e, you need to write your own implementation for such projection.

If the point cloud is not projected properly, you will lose many points and performance.

**The IMU has been remove from the original code.** Deal with it.

## Run the package

You may process a rosbag using the following command:

```
roslaunch lego_loam_bor run.launch rosbag:=/path/to/your/rosbag lidar_topic:=/velodyne_points
```

Change the parameters `rosbag`, `lidar_topic` as needed.


Some sample bags can be downloaded from [here](https://github.com/RobustFieldAutonomyLab/jackal_dataset_20170608).

## New data-set

This dataset, [Stevens data-set](https://github.com/TixiaoShan/Stevens-VLP16-Dataset), is captured using a Velodyne VLP-16, which is mounted on an UGV - Clearpath Jackal, on Stevens Institute of Technology campus.
The VLP-16 rotation rate is set to 10Hz. This data-set features over 20K scans and many loop-closures.

<p align='center'>
    <img src="/LeGO-LOAM/launch/dataset-demo.gif" alt="drawing" width="600"/>
</p>
<p align='center'>
    <img src="/LeGO-LOAM/launch/google-earth.png" alt="drawing" width="600"/>  
</p>

## Cite *SC-LeGO-LOAM-BOR*
```
@INPROCEEDINGS { gkim-2018-iros,
  author = {Kim, Giseop and Kim, Ayoung},
  title = { Scan Context: Egocentric Spatial Descriptor for Place Recognition within {3D} Point Cloud Map },
  booktitle = { Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems },
  year = { 2018 },
  month = { Oct. },
  address = { Madrid }
}

```

and

```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Tixiao Shan and Brendan Englot},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```
