//include pcl
#include <fstream>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

//include pcl
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

//include ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

//include gtsam
#include <gtsam/nonlinear/ISAM2.h>

//include mine
#include "scancontext/common.h"
#include "scancontext/tic_toc.h"
#include "lidar.h"
#include "scancontext/Scancontext.h"
#include "laserLoopOptimizationClass.h"

using namespace gtsam;

lidar::Lidar lidar_param;
laserLoopOptimizationClass laserLoopOptimization;


using std::cout;
using std::endl;

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init
Pose6D odom_pose_prev_temp {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero

nav_msgs::PathPtr curGT;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<nav_msgs::Odometry> ground_truth;
std::queue<sensor_msgs::NavSatFix::ConstPtr> gpsBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mutex_lock;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;
double timeGroud_truth = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());


SCManager scManager;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapPGO_edge(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapPGO_surf(new pcl::PointCloud<PointType>());

pcl::VoxelGrid<PointType> downSizeFilterMapPGO;

ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubPathAftPGO;
ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal;


void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    //laserCloudMsg???????????????
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
    mutex_lock.lock();
    odometryBuf.push(_laserOdometry);
    mutex_lock.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes)
{
    mutex_lock.lock();
    fullResBuf.push(_laserCloudFullRes);
    mutex_lock.unlock();
} // laserCloudFullResHandler


void GroundTruthHandler(const nav_msgs::Path::ConstPtr &_GroundTruth)
{
    mutex_lock.lock();
    if(_GroundTruth->poses.size()-10 > 0)
    {
        for (int j = _GroundTruth->poses.size()-10; j < _GroundTruth->poses.size(); ++j)
        {
            nav_msgs::Odometry gtOdometry;
            gtOdometry.header = _GroundTruth->poses[j].header;
            gtOdometry.pose.pose = _GroundTruth->poses[j].pose;
            ground_truth.push(gtOdometry);
        }
    }else{
        for (int j = 0; j < _GroundTruth->poses.size(); ++j)
        {
            nav_msgs::Odometry gtOdometry;
            gtOdometry.header = _GroundTruth->poses[j].header;
            gtOdometry.pose.pose = _GroundTruth->poses[j].pose;
            ground_truth.push(gtOdometry);
        }
    }

    mutex_lock.unlock();
}

//??????poseGraph???edge??????Vertexs
void process_pg()
{
    while(1)
    {
        //??????????????????????????????????????????gt??????
        while ( !odometryBuf.empty() && !fullResBuf.empty() && !ground_truth.empty() && !pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty())
        {
            //
            // pop and check keyframe is or not  ????????????????????????????????????
            //
            mutex_lock.lock();
            //????????????odometryBuf??????????????????????????????
            while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < fullResBuf.front()->header.stamp.toSec())
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mutex_lock.unlock();
                break;
            }

            //???gt?????????odometry???????????????
            while (!ground_truth.empty() && ground_truth.front().header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec())
                ground_truth.pop();
            if (ground_truth.empty())
            {
                mutex_lock.unlock();
                break;
            }
            //edge???odom????????????
            while (!pointCloudSurfBuf.empty() && pointCloudSurfBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec())
                pointCloudSurfBuf.pop();
            if (pointCloudSurfBuf.empty())
            {
                mutex_lock.unlock();
                break;
            }

            //surf???odom????????????
            while (!pointCloudEdgeBuf.empty() && pointCloudEdgeBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec())
                pointCloudEdgeBuf.pop();
            if (pointCloudEdgeBuf.empty())
            {
                mutex_lock.unlock();
                break;
            }

            //edge???surf????????????
            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;
            }
            //???????????????????????????????????????????????????????????????????????????????????????????????????????????? - 0.5*??????
            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;
            }

            // Time equal check
            timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
            timeLaser = fullResBuf.front()->header.stamp.toSec();
            timeGroud_truth = ground_truth.front().header.stamp.toSec();
//            ??????????????????????????????????????????
//            std::cout << std::fixed << std::setprecision(7) << pointCloudEdgeBuf.front()->header.stamp.toSec() << " and " << pointCloudSurfBuf.front()->header.stamp.toSec() << " and " << pointCloudSurfBuf.front()->header.stamp.toSec() << std::endl;

            // TODO

//            laserCloudFullRes->clear();
            //??????edge+surf
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            //??????????????????????????????*thisKeyFrame???
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            //??????edge
            pcl::PointCloud<PointType>::Ptr thisKeyFrame_edge(new pcl::PointCloud<PointType>());
            //??????????????????????????????*thisKeyFrame???
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *thisKeyFrame_edge);
            pointCloudEdgeBuf.pop();

            //??????surf
            pcl::PointCloud<PointType>::Ptr thisKeyFrame_surf(new pcl::PointCloud<PointType>());
            //??????????????????????????????*thisKeyFrame???
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *thisKeyFrame_surf);
            pointCloudSurfBuf.pop();



            Pose6D pose_curr = laserLoopOptimization.getOdom(odometryBuf.front());
            odometryBuf.pop();

            //??????gt??????
            Pose6D pose_gt_curr = laserLoopOptimization.getOdom(ground_truth.front());
            ground_truth.pop();

            mutex_lock.unlock();

            //??????????????????????????????????????????????????????????????????
//            if( ! laserLoopOptimization.gtSAMgraphMade /* prior node */) {
//                odom_pose_curr = pose_curr;
//            }else{
//                odom_pose_curr = pose_curr;
//            }
            odom_pose_prev_temp = odom_pose_curr;
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;


            //??????????????????????????????
//            ///todo:???????????????????????????????????????????????????????????????
            if(!laserLoopOptimization.isaNowKeyFrame(odom_pose_prev_temp, odom_pose_curr)){
                //?????????????????????????????????????????????
                continue;
            }
            //??????????????????????????????poseGraph???
//            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
//            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
//
//            pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
//            laserLoopOptimization.downSizeFilterICP.setInputCloud(thisKeyFrame_edge);
//            laserLoopOptimization.downSizeFilterICP.filter(*downsampledEdgeCloud);
//            laserLoopOptimization.downSizeFilterICP.setInputCloud(thisKeyFrame_surf);
//            laserLoopOptimization.downSizeFilterICP.filter(*downsampledSurfCloud);

//            std::cout << "?????????????????????" << thisKeyFrame_surf->size() << std::endl;
//            std::cout << "?????????????????????" << downsampledSurfCloud->size() << std::endl;
            laserLoopOptimization.save_and_update_PGnode(thisKeyFrame, thisKeyFrame_surf, thisKeyFrame_edge, pose_curr, pose_gt_curr, timeLaserOdometry, timeLaser, timeGroud_truth);


            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");
            // laserLoopOptimization.gtSAMgraph.print("\nFactor Graph:\n");

        }

        // ps.
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);//??????2ms
        std::this_thread::sleep_for(dura);
    }
} // process_pg


//??????????????????
void process_lcd()
{
    float loopClosureFrequency = 1.0;//1.0; // can change ?????????????????????
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok())
    {
        rate.sleep();
        laserLoopOptimization.performSCLoopClosure();
    }
} // process_lcd

double total_time1 =0;
int total_frame1=0;
void process()
{
    while(1)
    {
        while ( !laserLoopOptimization.scLoopBuf.empty() )
        {
            if( laserLoopOptimization.scLoopBuf.size() > 30 ) {
                ROS_WARN("??????????????????????????????????????????process_lcd???????????????");
            }
            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            bool correct = laserLoopOptimization.doloopclosure(pubLoopScanLocal, pubLoopSubmapLocal);

            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            if(correct){
                total_frame1++;
                float time_temp1 = elapsed_seconds.count() * 1000;
                total_time1+=time_temp1;
                ROS_INFO("average find_relation time %f ms \n \n", total_time1/total_frame1);
            }
        }
        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process

void process_viz_path()
{
    float hz = 10.0;
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        laserLoopOptimization.pubPath(pubOdomAftPGO, pubPathAftPGO);
    }
}

double total_time =0;
int total_frame=0;
void process_isam()
{
    float hz = 1; //?????????????????????
    ros::Rate rate(hz);//rate???hz?????????????????????????????????rate.sleep????????????rate.sleep?????????????????????????????????
    while (ros::ok()) {
        rate.sleep();
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();

        laserLoopOptimization.doIsam2();

        end = std::chrono::system_clock::now();
        std::chrono::duration<float> elapsed_seconds = end - start;
        total_frame++;
        float time_temp = elapsed_seconds.count() * 1000;
        total_time+=time_temp;
        ROS_INFO("average ISAM time %f ms \n \n", total_time/total_frame);
    }
}

void process_viz_map()
{
    float vizmapFrequency = 0.1; // 0.1 means run onces every 10s
    ros::Rate rate(vizmapFrequency);
    while (ros::ok()) {
        rate.sleep();
        laserCloudMapPGO = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
        laserLoopOptimization.pubMap(laserCloudMapPGO);

        downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
        downSizeFilterMapPGO.filter(*laserCloudMapPGO);

	pcl::io::savePCDFileBinary(laserLoopOptimization.pgScansDirectory + "map.pcd", *laserCloudMapPGO);

        sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
        pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
        laserCloudMapPGOMsg.header.frame_id = "map";
        pubMapAftPGO.publish(laserCloudMapPGOMsg);
    }
} // pointcloud_viz


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sc_floam_laserLO");
    ros::NodeHandle nh;

    laserLoopOptimization.save_directory = "/home/llz/message/";
    laserLoopOptimization.pgKITTIformat = laserLoopOptimization.save_directory + "optimized_poses.txt";
    laserLoopOptimization.odomKITTIformat = laserLoopOptimization.save_directory + "odom_poses.txt";
    laserLoopOptimization.gt_odomKITTIformat = laserLoopOptimization.save_directory + "gt_poses.txt";
    laserLoopOptimization.pg_gtTimeSaveStream = std::fstream(laserLoopOptimization.save_directory + "gt_times.txt", std::fstream::out);
    laserLoopOptimization.pgTimeSaveStream = std::fstream(laserLoopOptimization.save_directory + "times.txt", std::fstream::out);
    laserLoopOptimization.pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);
    laserLoopOptimization.pgScansDirectory = laserLoopOptimization.save_directory + "Scans/";

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period);
    nh.getParam("/vertical_angle", vertical_angle);
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    nh.param<double>("keyframe_meter_gap", laserLoopOptimization.keyframeMeterGap, 2); // pose assignment every k m move
    nh.param<double>("keyframe_deg_gap", laserLoopOptimization.keyframeDegGap, 10); // pose assignment every k deg rot
    laserLoopOptimization.keyframeRadGap = deg2rad(laserLoopOptimization.keyframeDegGap);

    nh.param<double>("sc_dist_thres", laserLoopOptimization.scDistThres, 0.4);
    nh.param<double>("sc_max_radius", laserLoopOptimization.scMaximumRadius, 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    laserLoopOptimization.isam = new ISAM2(parameters);
    laserLoopOptimization.initNoises();

    //scManager???????????????????????????????????????????????????
    scManager.setSCdistThres(laserLoopOptimization.scDistThres);
    scManager.setMaximumRadius(laserLoopOptimization.scMaximumRadius);

    //??????????????????????????????
    float filter_size = 0.4;
    laserLoopOptimization.downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    laserLoopOptimization.downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize;
    nh.param<double>("mapviz_filter_size", mapVizFilterSize, 0.6); // pose assignment every k frames
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

    //????????????
    //????????????????????????????????????fullResBuf,?????????????????? ?????????????????????????????????pcl???????????? ?????????
    //???????????????????????????????????????????????????
    ///velodyne_points_filtered ??????edge???surf
    ///velodyne_points?????????local????????????
//    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100, laserCloudFullResHandler);
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudFullResHandler);
    //???????????????????????????????????????odometryBuf
    //????????????????????????????????????????????????
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, laserOdometryHandler);
    //??????ground truth
    ros::Subscriber subGroundTruth = nh.subscribe<nav_msgs::Path>("/gt/trajectory", 100, GroundTruthHandler);
    //??????edge???surf
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);


    pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom", 100);
    pubPathAftPGO = nh.advertise<nav_msgs::Path>("/aft_pgo_path", 100);
    pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);

    pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
    pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);


    //6?????????????????????
    std::thread posegraph_slam {process_pg}; // pose graph construction
    std::thread lc_detection {process_lcd}; // loop closure detection
    std::thread relationPose_calculation {process}; // loop constraint calculation

    std::thread isam_update {process_isam}; // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added.

    std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)
    std::thread viz_path {process_viz_path}; // visualization - path (high frequency)

    ros::spin();

    return 0;
}
