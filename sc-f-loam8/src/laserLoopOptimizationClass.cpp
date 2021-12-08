//
// Created by llz on 2021/9/13.
//

#include <laserLoopOptimizationClass.h>

using std::cout;
using std::endl;


Pose6D laserLoopOptimizationClass::getOdom(const nav_msgs::Odometry::ConstPtr& _odom)
{
    //从_odom（nav_msgs::Odometry::ConstPtr）中提取出xyz以及roll，pitch，yaw
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw};
} // getOdom

//重载Pose6D 传入非ptr
Pose6D laserLoopOptimizationClass::getOdom(nav_msgs::Odometry  &_odom)
{
    //从_odom（nav_msgs::Odometry::ConstPtr）中提取出xyz以及roll，pitch，yaw
    auto tx = _odom.pose.pose.position.x;
    auto ty = _odom.pose.pose.position.y;
    auto tz = _odom.pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom.pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw};
} // getOdom



Pose6D laserLoopOptimizationClass::diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
//     std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

bool laserLoopOptimizationClass::isaNowKeyFrame(const Pose6D& odom_pose_prev, const Pose6D& odom_pose_curr)
{
    //diffTransformation函数：传入上一位姿与当前位姿；输出两个位姿之间的差，即prev2curr的位姿变换帧
    Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

    double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value.
    translationAccumulated += delta_translation;
    rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.

    if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
        isNowKeyFrame = true;
        translationAccumulated = 0.0; // reset
        rotaionAccumulated = 0.0; // reset
    } else {
        isNowKeyFrame = false;
    }
    //如果不是当前的关键帧则continue
    return isNowKeyFrame;


}

gtsam::Pose3 laserLoopOptimizationClass::Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void laserLoopOptimizationClass::initNoises()
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
} // initNoises


void laserLoopOptimizationClass::save_and_update_PGnode(const pcl::PointCloud<PointType>::Ptr& thisKeyFrame, pcl::PointCloud<PointType>::Ptr thisKeyFrame_surf, pcl::PointCloud<PointType>::Ptr thisKeyFrame_edge, Pose6D& pose_curr, Pose6D& pose_gt_curr, double& timeLaserOdometry, double& timeLaser, double& timeGroud_truth)
{
    //
    // Save data and Add consecutive node
    //
    pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
    downSizeFilterScancontext.setInputCloud(thisKeyFrame);//输入当前点云数据
    downSizeFilterScancontext.filter(*thisKeyFrameDS);//进行降采样之后，将点云数据储存到 *thisKeyFrameDS
    pcl::PointCloud<PointType>::Ptr thisKeyFrame_edgeDS(new pcl::PointCloud<PointType>());
    downSizeFilterScancontext.setInputCloud(thisKeyFrame_edge);//输入当前点云数据
    downSizeFilterScancontext.filter(*thisKeyFrame_edgeDS);//进行降采样之后，将点云数据储存到 *thisKeyFrameDS
    pcl::PointCloud<PointType>::Ptr thisKeyFrame_surfDS(new pcl::PointCloud<PointType>());
    downSizeFilterScancontext.setInputCloud(thisKeyFrame_surf);//输入当前点云数据
    downSizeFilterScancontext.filter(*thisKeyFrame_surfDS);//进行降采样之后，将点云数据储存到 *thisKeyFrameDS
    /*
     * 操作关键帧的几个值：keyframeLaserClouds、keyframePoses、keyframePosesUpdated、keyframeTimes
     * 以及调用scManager.makeAndSaveScancontextAndKeys函数
     * 将线程锁住，以防一个数据操作失效
     */
    mKF.lock();
    //todo 加入边界点和平面点，用与后续的计算位姿变换
    keyframeLaserClouds.push_back(thisKeyFrameDS);
    keyframeLaserClouds_edge.push_back(thisKeyFrame_edgeDS);
    keyframeLaserClouds_surf.push_back(thisKeyFrame_surfDS);
//    keyframeLaserClouds_edge.push_back(thisKeyFrame_edge);
//    keyframeLaserClouds_surf.push_back(thisKeyFrame_surf);
    keyframePoses.push_back(pose_curr);
    keyframegtPoses.push_back(pose_gt_curr);
    keyframePosesUpdated.push_back(pose_curr); // init
    keyframeTimes.push_back(timeLaserOdometry);

    //这里调用scManager.makeAndSaveScancontextAndKeys()函数，保存每一帧数据sc信息以及key值，用于icp中的初值
    scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

    laserCloudMapPGORedraw = true;
    mKF.unlock();

    //将当前关键帧的索引设置为当前关键字容器的最后一个索引，这个索引值会被添加到图优化节点构建中，同时这个索引也用于sc信息的读取
    const int prev_node_idx = keyframePoses.size() - 2;
    const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
    if( ! gtSAMgraphMade /* prior node */) {

        const int init_node_idx = 0;
        gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
        // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
        //操作gtSAMgraph，需要锁定线程
        mtxPosegraph.lock();
        {
            // prior factor
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
            initialEstimate.insert(init_node_idx, poseOrigin);
            // runISAM2opt();
        }
        mtxPosegraph.unlock();

        gtSAMgraphMade = true;

        cout << "posegraph prior node " << init_node_idx << " added" << endl;
    } else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1
        gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
        gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

        mtxPosegraph.lock();
        {
            // odom factor
            // 这里添加的是相邻关键帧之间的位姿变换关系
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));
            initialEstimate.insert(curr_node_idx, poseTo);
            // runISAM2opt();
        }
        mtxPosegraph.unlock();

        if(curr_node_idx % 100 == 0)
            cout << "posegraph odom node " << curr_node_idx << " added." << endl;
//        // save utility
//        std::string curr_node_idx_str = padZeros(curr_node_idx);
//        pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan
//        pgTimeSaveStream << timeLaserOdometry << std::endl; // path
//        pg_gtTimeSaveStream << std::fixed << std::setprecision(7) << timeGroud_truth << std::endl;
    }
}

std::string laserLoopOptimizationClass::padZeros(int val, int num_digits)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

void laserLoopOptimizationClass::performSCLoopClosure()
{
    //当查找到的关键帧数量小于scManager.NUM_EXCLUDE_RECENT（30），则不进行回环检测
    if( int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT) // do not try too early
        return;

    //scManager.detectLoopClosureID()返回一对数值 pair<int, float>，int是最近节点的索引，float是相差的偏航角
    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) {
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mutex_lock.lock();
        //将查找到的回环帧放入scLoopBuf，用于另一个icp线程的处理；这里scLoopBuf同时会被多个线程使用，为避免内存冲突，需要lock
        scLoopBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mutex_lock.unlock();
    }
} // performSCLoopClosure

pcl::PointCloud<PointType>::Ptr laserLoopOptimizationClass::local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    int numberOfCores = 12;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void laserLoopOptimizationClass::loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = root_idx + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock();
        if(key == root_idx){//当查找当前关键帧点云时时采用当前关键帧的位姿
            *nearKeyframes += * local2global(keyframeLaserClouds[key], keyframePosesUpdated[key]);
        }else{//查找对应帧附件submap点云数据时，采用以下方法
            *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        }

//        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud

void laserLoopOptimizationClass::loopFindNearKeyframesCloud_surf( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = root_idx + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock();
        *nearKeyframes += * local2global(keyframeLaserClouds_surf[keyNear], keyframePosesUpdated[keyNear]);
//        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud_surf

void laserLoopOptimizationClass::loopFindNearKeyframesCloud_edge( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = root_idx + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock();
        *nearKeyframes += *local2global(keyframeLaserClouds_edge[keyNear], keyframePosesUpdated[keyNear]);

//        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud_edge

void laserLoopOptimizationClass::GetPoints(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_local_curr * point_curr + t_local_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

void laserLoopOptimizationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function)
{
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        //取出一帧点云，暂存到point_temp
        GetPoints(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        /*
         * 在调用addEdgeCostFactor函数之前，kdtreeEdgeMap已经将edge_map添加进去了
         * 利用kdtree查找与point_temp最近的5个点
         * 计算最近5个点的均值（center）和协方差（covMat）
        */
        kdtreeEdgesubMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            //5个最近点的均值和协方差
            center = center / 5.0;
            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);//SelfAdjointEigenSolver计算特征向量和特征值

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);//雷达坐标系下
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])//当最大的特征值大于3倍第二大的特征值，则认为是一个直线
            {
                Eigen::Vector3d point_on_line = center;
                //point_a, point_b是直线上取的两个点
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;
                //point_a 和 point_b组成一条线段
                //EdgeAnalyticCostFunction构建costfunction代价函数
                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  //传入已知参数，构建costfunction
                problem.AddResidualBlock(cost_function, loss_function, parameters);//添加残差块，其中parameters是代优化的参数
                corner_num++;
            }
        }
    }
//    if(corner_num<10){
//        cout <<"not enough correct points" << endl;
//    }

}

void laserLoopOptimizationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        GetPoints(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfsubMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
        {

            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }

    }
//    if(surf_num<20){
//        cout <<"not enough correct points" << endl;
//    }

}

void laserLoopOptimizationClass::doCeres(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in, gtsam::Pose3& curr2loop){
    optimization_count = 2;
    //初始化parameters
//    parameters[0] = curr2loop.x();
//    parameters[1] = curr2loop.y();
//    parameters[2] = curr2loop.z();
//    Eigen::Quaterniond q(curr2loop.rotation().matrix());
//    parameters[3] = q.w();
//    parameters[4] = q.x();
//    parameters[5] = q.y();
//    parameters[6] = q.z();
    parameters[0] = 0;
    parameters[1] = 0;
    parameters[2] = 0;
    parameters[3] = 1;
    parameters[4] = 0;
    parameters[5] = 0;
    parameters[6] = 0;
    //初始化kdtree
    kdtreeEdgesubMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeSurfsubMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());


//    cout << laserCloudCornerMap->points.size() << " and " << laserCloudSurfMap->points.size() << " and "<< edge_in->points.size()<<endl;
    //下面两行代码的意思是：将MAP的edge和surf添加到kdtree，用于后续的查找最近点

    kdtreeEdgesubMap->setInputCloud(laserCloudCornerMap);
    kdtreeSurfsubMap->setInputCloud(laserCloudSurfMap);
    for (int iterCount = 0; iterCount < optimization_count; iterCount++){
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        //在创建的problem中添加优化问题的优化变量，parameters是优化变量
        problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());

        //ceres添加残差块通过 AddResidualBlock()完成
        //这里的edge_in与map_edge_in均是在世界坐标系下
        addEdgeCostFactor(edge_in, laserCloudCornerMap,problem,loss_function);
        addSurfCostFactor(surf_in, laserCloudSurfMap,problem,loss_function);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 100;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-6;
        ceres::Solver::Summary summary;

        ceres::Solve(options, &problem, &summary);//优化了参数parameters

    }

    //更新odom参数
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_local_curr.toRotationMatrix();
    odom.translation() = t_local_curr;
}


std::optional<gtsam::Pose3> laserLoopOptimizationClass::doVirtualRelative( const int& _loop_kf_idx, const int& _curr_kf_idx, ros::Publisher& pubLoopScanLocal, ros::Publisher& pubLoopSubmapLocal, bool& correct)
{
    mKF.lock();
    gtsam::Pose3 curr_gtPose3 = Pose6DtoGTSAMPose3(keyframePosesUpdated[_curr_kf_idx]);
    gtsam::Pose3 loop_gtPose3 = Pose6DtoGTSAMPose3(keyframePosesUpdated[_loop_kf_idx]);
    gtsam::Pose3 curr2loop = curr_gtPose3.between(loop_gtPose3);
    mKF.unlock();
    double odom_distance = sqrt((curr2loop.x()) * (curr2loop.x()) + (curr2loop.y()) * (curr2loop.y()) + (curr2loop.z()) * (curr2loop.z()));

    gtsam::Pose3 poseFrom;
    gtsam::Pose3 poseTo;
    Eigen::Vector3d trans = odom.translation();

    if(odom_distance > 20){
        correct = false;
        poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
        poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
        return poseFrom.between(poseTo);
    }


    const int submap_size = 25;
    // parse pointclouds
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
//    int historyKeyframeSearchNum = 0; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    cureKeyframeCloud_surf = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    cureKeyframeCloud_edge = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    //取出点云数据,scan --> submap
    //submap_size = 17 ape = 0.85 rmse =
    //submap_size = 13 ape = 0.86 rmse = 1.03
    //submap_size = 25 ape = 0.86 rmse = 1.00
    //
//    loopFindNearKeyframesCloud_edge(cureKeyframeCloud_edge, 0, _curr_kf_idx);
    loopFindNearKeyframesCloud_edge(cureKeyframeCloud_edge, 0, _loop_kf_idx);
    loopFindNearKeyframesCloud_edge(laserCloudCornerMap, submap_size, _loop_kf_idx);
//    loopFindNearKeyframesCloud_surf(cureKeyframeCloud_surf, 0, _curr_kf_idx);
    loopFindNearKeyframesCloud_surf(cureKeyframeCloud_surf, 0, _loop_kf_idx);
    loopFindNearKeyframesCloud_surf(laserCloudSurfMap, submap_size, _loop_kf_idx);

//    // loop verification
//    sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
//    pcl::toROSMsg(*cureKeyframeCloud_surf + *cureKeyframeCloud_edge, cureKeyframeCloudMsg);
//    cureKeyframeCloudMsg.header.frame_id = "map";
//    pubLoopScanLocal.publish(cureKeyframeCloudMsg);
//
//    sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
//    pcl::toROSMsg(*laserCloudSurfMap + *laserCloudCornerMap, targetKeyframeCloudMsg);
//    targetKeyframeCloudMsg.header.frame_id = "map";
//    pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);
    ///todo:计算检测到的两帧数据之间的初始位姿


    doCeres(cureKeyframeCloud_edge, cureKeyframeCloud_surf, curr2loop);


    //判断是否是回环

//    double distance = sqrt(trans.transpose() * trans);
//    if(distance > odom_distance + 15 || distance < odom_distance - 15  || distance == 0){
//        ROS_INFO("index %d and index %d loop detect filed, distance is %f m, odom_distanced is %f m  \n", _loop_kf_idx, _curr_kf_idx, distance, odom_distance);
//        poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
//        poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
//    }else{
//        ROS_INFO("index %d and index %d loop detect successed, distance is %f m, odom_distanced is %f m  \n", _loop_kf_idx, _curr_kf_idx, distance, odom_distance);
//        correct = true;
//        Eigen::Matrix3d rota_matrix = odom.rotation();
//        Eigen::Vector3d eulerAngle = rota_matrix.eulerAngles(2,1,0);
//        poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(eulerAngle[0], eulerAngle[1], eulerAngle[2]), gtsam::Point3(trans[0], trans[1], trans[2]));
//        poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
//    }
    correct = true;
    Eigen::Matrix3d rota_matrix = odom.rotation();
    Eigen::Vector3d eulerAngle = rota_matrix.eulerAngles(2,1,0);
    poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(eulerAngle[0], eulerAngle[1], eulerAngle[2]), gtsam::Point3(trans[0], trans[1], trans[2]));
    poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

    //这个poseFrom.between(poseTo)得到的是poseTo相对于poseFrom的姿态变化，即以poseFrom为基准坐标，poseTo在poseFrom基准坐标下的坐标
    //这里的意思就是将cur --> loop的位姿变换转化为loop --> cur的位姿变换
//    std::cout << "初值：" << curr2loop.x() << "," << curr2loop.y() << "," << curr2loop.z() << std::endl;
//    std::cout << "结果：" << poseFrom.x() << "," << poseFrom.y() << "," << poseFrom.z() << std::endl;
//    std::cout << std::endl;
    return poseFrom.between(poseTo);
}


int interval=30;
bool laserLoopOptimizationClass::doloopclosure(ros::Publisher &pubLoopScanLocal, ros::Publisher &pubLoopSubmapLocal)
{
    mutex_lock.lock();
    //scLoopBuf是一个队列queue< std::pair<int, int> > 储存的是两个满足回环检测的节点值
    std::pair<int, int> loop_idx_pair = scLoopBuf.front();
    scLoopBuf.pop();
    mutex_lock.unlock();
    //利用一个滞后与curr的节点与其自身周围的节点进行计算

    //取出构成回环的两个关键帧索引
    const int prev_node_idx = loop_idx_pair.first;
    const int curr_node_idx = loop_idx_pair.second;


    //doVirtualRelative用来计算两帧数据之间的位姿变换
    bool correct = false;
    auto relative_pose_optional = doVirtualRelative(prev_node_idx, curr_node_idx, pubLoopScanLocal, pubLoopSubmapLocal, correct);
    if(correct)
    {
        gtsam::Pose3 relative_pose = relative_pose_optional.value();
        mtxPosegraph.lock();
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
//        cout << "添加回环节点成功" << endl;
        mtxPosegraph.unlock();

        // loop verification
        sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
        pcl::toROSMsg(*cureKeyframeCloud_surf + *cureKeyframeCloud_edge, cureKeyframeCloudMsg);
        cureKeyframeCloudMsg.header.frame_id = "map";
        pubLoopScanLocal.publish(cureKeyframeCloudMsg);

        sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
        pcl::toROSMsg(*laserCloudSurfMap + *laserCloudCornerMap, targetKeyframeCloudMsg);
        targetKeyframeCloudMsg.header.frame_id = "map";
        pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);
        return correct;
    }
    return correct;
//    if (curr_node_idx - interval >= 25) {
//
//        auto relative_pose_optional = doVirtualRelative(interval, interval, pubLoopScanLocal,
//                                                        pubLoopSubmapLocal, correct);
//
//        if(correct) {
//            gtsam::Pose3 relative_pose = relative_pose_optional.value();
//            mtxPosegraph.lock();
//            gtSAMgraph.add(
//                    gtsam::BetweenFactor<gtsam::Pose3>(interval, interval, relative_pose, robustLoopNoise));
////        cout << "添加回环节点成功" << endl;
//            mtxPosegraph.unlock();
//
//            // loop verification
//            sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
//            pcl::toROSMsg(*cureKeyframeCloud_surf + *cureKeyframeCloud_edge, cureKeyframeCloudMsg);
//            cureKeyframeCloudMsg.header.frame_id = "map";
//            pubLoopScanLocal.publish(cureKeyframeCloudMsg);
//
//            sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
//            pcl::toROSMsg(*laserCloudSurfMap + *laserCloudCornerMap, targetKeyframeCloudMsg);
//            targetKeyframeCloudMsg.header.frame_id = "map";
//            pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);
//
//            interval = curr_node_idx;
//            return;
//        }
//    }
}


void laserLoopOptimizationClass::updatePoses()
{
    mKF.lock();
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p = keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
//    recentOptimizedX = lastOptimizedPose.translation().x();
//    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void laserLoopOptimizationClass::runISAM2opt()
{
    // called when a variable added
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}//runISAM2opt

void laserLoopOptimizationClass::saveOptimizedVerticesKITTIformat(const gtsam::Values& _estimates, std::string _filename)
{
    using namespace gtsam;
    std::cout << "优化后位姿的长度: " << _estimates.size() << std::endl;

    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }

}//saveOptimizedVerticesKITTIformat

void laserLoopOptimizationClass::saveOdometryVerticesKITTIformat(const std::string& _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::cout << "优化前位姿的长度： " << keyframePoses.size() << std::endl;
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        gtsam::Point3 t = pose.translation();
        gtsam::Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3
        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}//saveOdometryVerticesKITTIformat

void laserLoopOptimizationClass::savegtVerticesKITTIformat(const std::string& _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::cout << "gt位姿的长度： " << keyframegtPoses.size() << std::endl;
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframegtPoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        gtsam::Point3 t = pose.translation();
        gtsam::Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}//savegtVerticesKITTIformat

void laserLoopOptimizationClass::doIsam2() {
    if( gtSAMgraphMade ) {
        mtxPosegraph.lock();
        runISAM2opt();
        cout << "正在执行isam2优化 ..." << endl;
        mtxPosegraph.unlock();

        saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
        saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
        savegtVerticesKITTIformat(gt_odomKITTIformat); // gt_pose
    }
}

void laserLoopOptimizationClass::pubMap(const pcl::PointCloud<PointType>::Ptr& laserCloudMapPGO) {
    if(recentIdxUpdated > 1) {
        int SKIP_FRAMES = 2; // sparse map visulalization to save computations
        int counter = 0;

        laserCloudMapPGO->clear();

        mKF.lock();
        // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
        for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
            if(counter % SKIP_FRAMES == 0) {
                *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
            }
            counter++;
        }
        mKF.unlock();

//        downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
//        downSizeFilterMapPGO.filter(*laserCloudMapPGO);
//
//        sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
//        pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
//        laserCloudMapPGOMsg.header.frame_id = "map";
//        pubMapAftPGO.publish(laserCloudMapPGOMsg);
    }
}

void laserLoopOptimizationClass::pubPath(ros::Publisher& pubOdomAftPGO, ros::Publisher& pubPathAftPGO)
{
    if(recentIdxUpdated > 1) {
        // pub odom and path
        nav_msgs::Odometry odomAftPGO;
        nav_msgs::Path pathAftPGO;
        pathAftPGO.header.frame_id = "map";
        mKF.lock();
        // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
        for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
        {
            const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
            // const gtsam::Pose3& pose_est = isamCurrentEstimate.at<gtsam::Pose3>(node_idx);

            nav_msgs::Odometry odomAftPGOthis;
            odomAftPGOthis.header.frame_id = "map";
            odomAftPGOthis.child_frame_id = "aft_pgo_odom";
            odomAftPGOthis.header.stamp = ros::Time().fromSec(keyframeTimes.at(node_idx));
            odomAftPGOthis.pose.pose.position.x = pose_est.x;
            odomAftPGOthis.pose.pose.position.y = pose_est.y;
            odomAftPGOthis.pose.pose.position.z = pose_est.z;
            odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);
            odomAftPGO = odomAftPGOthis;

            geometry_msgs::PoseStamped poseStampAftPGO;
            poseStampAftPGO.header = odomAftPGOthis.header;
            poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

            pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
            pathAftPGO.header.frame_id = "map";
            pathAftPGO.poses.push_back(poseStampAftPGO);
        }
        mKF.unlock();
        pubOdomAftPGO.publish(odomAftPGO); // last pose
        pubPathAftPGO.publish(pathAftPGO); // poses

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, odomAftPGO.pose.pose.position.z));
        q.setW(odomAftPGO.pose.pose.orientation.w);
        q.setX(odomAftPGO.pose.pose.orientation.x);
        q.setY(odomAftPGO.pose.pose.orientation.y);
        q.setZ(odomAftPGO.pose.pose.orientation.z);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, odomAftPGO.header.stamp, "map", "aft_pgo_odom"));
    }
}

laserLoopOptimizationClass::laserLoopOptimizationClass()
{

}





