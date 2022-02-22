# Optimized-SC-F-LOAM
This work combines F-LOAM with Scan-Context and optimizes it to reduce the time required to compute pose constraints between a pair of loop frames to 28% of Simple-SC-F-LOAM.
(Simple-SC-F-LOAM is a way to directly combine F-LOAM with Scan-Context)
contributor: Lizhou Liao, Chongqing University, Chongqing, China

# 1. Mapping result
![效果图3](https://user-images.githubusercontent.com/95751923/155124889-934ea649-3b03-4e8d-84af-608753f34c93.png)

# 2. UGV platform
![image](https://user-images.githubusercontent.com/95751923/155125896-55f90f9d-4c04-4cc0-a605-095c7d05709d.png)

UGV equipped with a 16-beam LiDAR
# 3. Evaluation

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
Follow Ceres Installation: http://www.ceres-solver.org/installation.html.

## 4.3. PCL
Follow PCL Installation: https://pointclouds.org/downloads/#linux.

## 4.4. GTSAM
Follow GTSAM Installation: https://gtsam.org/get_started/.

## 4.4. Trajectory visualization
For visualization purpose, this package uses hector trajectory sever, you may install the package by

sudo apt-get install ros-melodic-hector-trajectory-server
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

4. Build
4.1 Clone repository:
    cd ~/catkin_ws/src
    git clone https://github.com/wh200720041/floam.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
4.2 Download test rosbag
Download KITTI sequence 05 or KITTI sequence 07

Unzip compressed file 2011_09_30_0018.zip. If your system does not have unzip. please install unzip by

sudo apt-get install unzip 
And this may take a few minutes to unzip the file

	cd ~/Downloads
	unzip ~/Downloads/2011_09_30_0018.zip
4.3 Launch ROS
    roslaunch floam floam.launch
if you would like to create the map at the same time, you can run (more cpu cost)

    roslaunch floam floam_mapping.launch
If the mapping process is slow, you may wish to change the rosbag speed by replacing "--clock -r 0.5" with "--clock -r 0.2" in your launch file, or you can change the map publish frequency manually (default is 10 Hz)

5. Test on other sequence
To generate rosbag file of kitti dataset, you may use the tools provided by kitti_to_rosbag or kitti2bag

6. Test on Velodyne VLP-16 or HDL-32
You may wish to test FLOAM on your own platform and sensor such as VLP-16 You can install the velodyne sensor driver by

sudo apt-get install ros-melodic-velodyne-pointcloud
launch floam for your own velodyne sensor

    roslaunch floam floam_velodyne.launch
If you are using HDL-32 or other sensor, please change the scan_line in the launch file

7.Acknowledgements
Thanks for A-LOAM and LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and LOAM_NOTED.

8. Citation
If you use this work for your research, you may want to cite

@inproceedings{wang2021,
  author={H. {Wang} and C. {Wang} and C. {Chen} and L. {Xie}},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={F-LOAM : Fast LiDAR Odometry and Mapping}, 
  year={2020},
  volume={},
  number={}
}
