//
// Created by llz on 2021/9/13.
//

#pragma once

#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "scancontext/common.h"
#include "scancontext/tic_toc.h"
#include "lidar.h"
#include "scancontext/Scancontext.h"
#include "lidarOptimization.h"

class laserLoopOptimizationClass {
public:
    laserLoopOptimizationClass();
    //process_pg
    //将nav_msgs::Odometry以及nav_msgs::Odometry信息转化为Pose6D信息
    static Pose6D getOdom(const nav_msgs::Odometry::ConstPtr& _odom);
    static Pose6D getOdom(nav_msgs::Odometry  &_odom);
    //计算两个位姿之间的距离
    static Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2);
    bool isaNowKeyFrame(const Pose6D& odom_pose_prev, const Pose6D& odom_pose_curr);
    void save_and_update_PGnode(const pcl::PointCloud<PointType>::Ptr& thisKeyFrame, pcl::PointCloud<PointType>::Ptr thisKeyFrame_surf, pcl::PointCloud<PointType>::Ptr thisKeyFrame_edge, Pose6D& pose_curr, Pose6D& pose_gt_curr, double& timeLaserOdometry, double& timeLaser, double& timeGroud_truth);
    static gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p);
    void initNoises();
    std::string padZeros(int val, int num_digits = 6);

    void performSCLoopClosure();

    //process_icp
    bool doloopclosure(ros::Publisher& pubLoopScanLocal, ros::Publisher& pubLoopSubmapLocal);
    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf);
    void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx);
    void loopFindNearKeyframesCloud_surf( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& submap_size, const int& root_idx);
    void loopFindNearKeyframesCloud_edge( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& submap_size, const int& root_idx);
    void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes_edge,  pcl::PointCloud<PointType>::Ptr& nearKeyframes_surf,  pcl::PointCloud<PointType>::Ptr& Keyframes_edge, pcl::PointCloud<PointType>::Ptr& Keyframes_surf, const int& submap_size, const int& root_idx, const int& curr_idx);


    void GetPoints(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po);
    void doCeres(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in,  gtsam::Pose3& curr2loop);
    std::optional<gtsam::Pose3> doVirtualRelative( const int& _loop_kf_idx, const int& _curr_kf_idx ,ros::Publisher& pubLoopScanLocal, ros::Publisher& pubLoopSubmapLocal, bool& correct);
    std::optional<gtsam::Pose3> doVirtualRelative_1( const int& _loop_kf_idx, const int& _curr_kf_idx ,ros::Publisher& pubLoopScanLocal, ros::Publisher& pubLoopSubmapLocal, bool& correct);


    //process_isam
    void saveOptimizedVerticesKITTIformat(const gtsam::Values& _estimates, std::string _filename);
    void saveOdometryVerticesKITTIformat(const std::string& _filename);
    void savegtVerticesKITTIformat(const std::string& _filename);
    void updatePoses();
    void runISAM2opt();
    void doIsam2();

    //process_viz_map
    void pubMap(const pcl::PointCloud<PointType>::Ptr& laserCloudMapPGO);

    //process_viz_path
    void pubPath(ros::Publisher& pubOdomAftPGO, ros::Publisher& pubPathAftPGO);

public:
    gtsam::NonlinearFactorGraph gtSAMgraph;
    std::mutex mKF;
    std::mutex mutex_lock;
    std::mutex mtxICP;
    std::mutex mtxPosegraph;
    std::mutex mtxRecentPose;

    //储存查找到的回环信息
    std::queue<std::pair<int, int> > scLoopICPBuf;
    std::queue<std::pair<int, int> > scLoopBuf;

    //图优化噪声模型
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odomNoise;
    gtsam::noiseModel::Base::shared_ptr robustLoopNoise;

    //定义一个图优化对象
    bool gtSAMgraphMade = false;//判断添加图优化的节点是否为第一个
    gtsam::Values initialEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;


    pcl::VoxelGrid<PointType> downSizeFilterICP;

    //判断是否为关键帧的变量
    double translationAccumulated = 1000000.0; // large value means must add the first given frame.大值意味着必须添加第一个给定的帧。
    double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.
    double keyframeMeterGap;
    double keyframeDegGap, keyframeRadGap;
    bool isNowKeyFrame = false;

    //更新PG节点的变量
    pcl::VoxelGrid<PointType> downSizeFilterScancontext;
    SCManager scManager;
    double scDistThres, scMaximumRadius;

    //关键帧信息变量
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds;
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds_edge;
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds_surf;
    std::vector<Pose6D> keyframePoses;
    std::vector<Pose6D> keyframegtPoses;
    std::vector<Pose6D> keyframePosesUpdated;
    std::vector<double> keyframeTimes;
    int recentIdxUpdated = 0;

    //储存数据的变量
    std::string save_directory;
    std::string pgKITTIformat, pgScansDirectory;
    std::string odomKITTIformat;
    std::string gt_odomKITTIformat;
    std::fstream pgTimeSaveStream;
    std::fstream pg_gtTimeSaveStream;

    //pubmap
    pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
    bool laserCloudMapPGORedraw = true;

    //ceres
    int optimization_count;
    Eigen::Isometry3d odom;
private:
    //optimization variable
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_local_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
    Eigen::Map<Eigen::Vector3d> t_local_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

    //kd-tree
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgesubMap;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfsubMap;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap;
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud_surf;
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud_edge;



    void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem, ceres::LossFunction *loss_function);
    void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);

};


