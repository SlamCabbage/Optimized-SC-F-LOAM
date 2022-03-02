**This method is different from the method of directly combining scan context and f-loam in the following two points:**
**1. The method uses an adaptive threshold to further judge the loop closure detection results, reducing false loop closure detections;**
**2. This method uses feature point-based matching to calculate the constraints between a pair of loop closure frame point clouds, so that this method only needs 30% of the time consumption of SC-A-LOAM in the process of constructing loopback frame constraints.**

**<u> Note: This method has been submitted to IEEE Robotics and automation letters && IROS. Therefore, you cannot use this method as an innovation point of your paper. </u>**

# Optimized-SC-F-LOAM
This work combines F-LOAM with Scan-Context and optimizes it to reduce the time required to compute pose constraints between a pair of loop closure point clouds to 28% of Simple-SC-F-LOAM.(Simple-SC-F-LOAM is a way to directly combine F-LOAM with Scan-Context)

contributor: Lizhou Liao, Chongqing University, Chongqing, China

# 1. Mapping result
![效果图3](https://user-images.githubusercontent.com/95751923/155124889-934ea649-3b03-4e8d-84af-608753f34c93.png)

# 2. UGV platform
![image](https://user-images.githubusercontent.com/95751923/155125896-55f90f9d-4c04-4cc0-a605-095c7d05709d.png)

UGV equipped with a 16-beam LiDAR
# 3. Evaluation

All tests were done based on the Robot Operating System (ROS) that was installed on a laptop with an AMD R5-5600H processor, a 16 GB RAM and the Ubuntu platform.

## 3.1. Results on KITTI Sequence 00 and Sequence 05
![image](https://user-images.githubusercontent.com/95751923/155125294-980e6a3d-6e76-4a23-9771-493ba278677e.png)


## 3.2. Comparison of trajectories on KITTI dataset
![image](https://user-images.githubusercontent.com/95751923/155125478-a361762f-f18e-4161-b892-6f5080f5681f.png)


## 3.3. Experiment environment of the UGV platform
![image](https://user-images.githubusercontent.com/95751923/155126033-faae2e47-ca13-4fa5-b8ea-653a1a03cc2d.png)


# 4. Prerequisites
## 4.1. Ubuntu and ROS
    Ubuntu 64-bit 20.04.
    
    ROS noetic. ROS Installation: http://wiki.ros.org/noetic/Installation/Ubuntu.

## 4.2. Ceres Solver
    Follow Ceres Installation: 
    http://www.ceres-solver.org/installation.html.

## 4.3. PCL
    Follow PCL Installation: 
    https://pointclouds.org/downloads/#linux.

## 4.4. GTSAM
    Follow GTSAM Installation: 
    https://gtsam.org/get_started/.

## 4.4. Trajectory visualization
For visualization purpose, this package uses hector trajectory sever, you may install the package by

    sudo apt-get install ros-noetic-hector-trajectory-server

# 5. Build
## 5.1. Clone repository:
    cd ~/catkin_ws/src
    git clone https://github.com/SlamCabbage/Optimized-SC-F-LOAM.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash

## 5.2. Create message file:

In this folder, Ground Truth information, optimized pose information, F-LOAM pose information and time information are stored：

```
mkdir ~/message
```

## 5.3. Launch ROS

    roslaunch optimized_sc_f_loam optimized_sc_f_loam_mapping.launch

# 6. Test on KITTI sequence
You can download the sequence 00 and 05 datasets from the KITTI official website and convert them into bag files using the kitti2bag open source method.

Nr.     Sequence name     Start   End
---------------------------------------
00: 2011_10_03_drive_0027 000000 004540
05: 2011_09_30_drive_0018 000000 002760

See the link: https://github.com/ethz-asl/kitti_to_rosbag


# 7. Acknowledgements
Thanks for SC-A-LOAM(Scan context: Egocentric spatial descriptor for place recognition within 3d point cloud map) and F-LOAM(F-LOAM : Fast LiDAR Odometry and Mapping).

# 8. citation

waiting
