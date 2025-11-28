#include "optimization.h"

optimization::optimization(ros::NodeHandle &nh)
{
    // gtsam::ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.1;
    // parameters.relinearizeSkip = 1;
    // isam = std::make_unique<gtsam::ISAM2>(parameters);
    common_lib_ = std::make_unique<common_lib>();
    
    nh.param<std::string>("gps/optimization_mode", optimization_mode_, "online");
    nh.param<std::string>("gps/opt_tum_output_path", opt_tum_output_path_, "opt_trajectory.txt");
    nh.param<std::string>("gps/gps_tum_output_path", gps_tum_output_path_, "gps_trajectory.txt");
    nh.param<std::string>("gps/global_map_pcd_path", global_map_pcd_path_, "optimized_map.pcd");
    nh.param<std::string>("gps/save_pcd_directory", pcd_save_directory_, "/home/nomai/ws_fastlivo2_rtk/src/FAST-LIVO2/scan_pcd");
    nh.param<string>("gps/gps_topic", gps_topic, "/ublox_driver/receiver_lla");
    nh.param<float>("gps/gps_offset", gps_offset, 0.0);
    nh.param<bool>("gps/gps_en", gps_en, false);
    nh.param<vector<double>>("gps/extrinsic_T", gps_extrinT, vector<double>());
    nh.param<float>("gps/poseCovThreshold", poseCovThreshold, 25.0);
    nh.param<float>("gps/gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<float>("gps/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
    nh.param<float>("gps/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);

    nh.param<bool>("loop/loop_en", loopClosureEnable_, false);
    nh.param<float>("loop/searchRadius", loopClosureSearchRadius_, 10.0);
    nh.param<float>("loop/timeThreshold", loopClosureTimeThreshold_, 30.0); // 至少 30s 才能算回环
    nh.param<float>("loop/fitnessScore", loopClosureFitnessScore_, 0.5);
    nh.param<float>("loop/voxelLeafSize", loopVoxelLeafSize_, 0.4);

    nh.param<bool>("opt/load_data", load_data, false);
    nh.param<double>("opt/rtk_cov", rtk_cov, 0.02);
    nh.param<double>("opt/livo2_RPY_cov", livo2_RPY_cov, 1e-4);
    nh.param<double>("opt/livo2_XYZ_cov", livo2_XYZ_cov, 1e-4);
    nh.param<double>("opt/loop_cov", loop_cov, 1e-4);

    nh.param<std::string>("gps/imu_topic", imu_topic, "/livox/imu");

    subImu = nh.subscribe(imu_topic, 2000, &optimization::imuHandler, this, ros::TransportHints().tcpNoDelay());
    // subGPS = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 2000, &optimization::gpsHandler, this);
    subGPS_pvt = nh.subscribe<gnss_comm::GnssPVTSolnMsg>("/ublox_driver/receiver_pvt", 2000, &optimization::gpsHandler, this);
    // subGPS_lla = nh.subscribe<sensor_msgs::NavSatFix>("/ublox_driver/receiver_lla", 2000, &optimization::gpsHandler, this);
    // subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/synced_cloud", 2000, &optimization::cloudHandler, this, ros::TransportHints().tcpNoDelay());
    // subOdom = nh.subscribe<nav_msgs::Odometry>("/odometry/fast_livo2", 2000, &optimization::odomHandler, this, ros::TransportHints().tcpNoDelay());
    pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/optimization/key_poses", 1);
    pubPath = nh.advertise<nav_msgs::Path>("/optimization/path", 1);
    pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>("/optimization/recent_key_frame", 1);
    pubGpsOdom = nh.advertise<nav_msgs::Odometry> ("/optimization/gps_odom", 1);
    pubGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("/optimization/global_map", 1);
    pubLoopLocalMap = nh.advertise<sensor_msgs::PointCloud2>("/optimization/loop_local_map_cloud", 1);
    pubLoopCurrentScan = nh.advertise<sensor_msgs::PointCloud2>("/optimization/loop_current_scan", 1);

    Eigen::Vector3d gps_lever_arm_; 
    gps_lever_arm_(0) = gps_extrinT[0]; 
    gps_lever_arm_(1) = gps_extrinT[1]; 
    gps_lever_arm_(2) = gps_extrinT[2]; 
    T_imu_rtk = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(gps_lever_arm_));

    subOdom_.subscribe(nh, "/odometry/fast_livo2", 2000);
    subCloud_.subscribe(nh, "/synced_cloud", 2000);
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), subOdom_, subCloud_);
    sync_->registerCallback(boost::bind(&optimization::syncedCallback, this, _1, _2));

    laserCloudSurfLastDS.reset(new PointCloudXYZRGB());
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses_.reset(new pcl::KdTreeFLANN<PointType>());

    icpLoop_.setMaxCorrespondenceDistance(10.0);
    icpLoop_.setMaximumIterations(100);
    icpLoop_.setTransformationEpsilon(1e-6);
    icpLoop_.setEuclideanFitnessEpsilon(1e-6);
    icpLoop_.setRANSACIterations(0); 
    downSizeFilterICP_.setLeafSize(loopVoxelLeafSize_, loopVoxelLeafSize_, loopVoxelLeafSize_);

    loadVoxelConfig(nh, voxel_config_);
    poseCovariance = gtsam::Matrix66::Identity();
    for (int i = 0; i < 6; ++i){
        transformTobeMapped[i] = 0;
    }

    ROS_INFO("load_data: %d", load_data);
    if(load_data)
    {
        std::string data_path = "/home/nomai/ws_fastlivo2_rtk/src/FAST-LIVO2/DATA/";
        loadData(data_path);
    }

    ROS_INFO("Optimization mode: [OFFLINE]. Accumulating factors for batch optimization.");
    optimization_thread_ = std::thread(&optimization::offlineOptimizationTask, this);

}

optimization::~optimization() 
{
}

void optimization::offlineOptimizationTask() {
    std::cin.get();
    std::cout << "[Offline Optimization] Starting batch optimization..." << std::endl;
    
    initialAlign();
    saveOptimizedGlobalMap();
    std::cout << "[Offline Optimization] Initial alignment done." << std::endl;
    std::cout << "[Offline Optimization] Initial global map saved." << std::endl;
    //propagateGps();
    std::cout << "[Offline Optimization] GPS propagation done." << std::endl;
    gtsam::LevenbergMarquardtParams params;

    params.orderingType = gtsam::Ordering::METIS;
    params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
    params.setMaxIterations(500);
    params.setRelativeErrorTol(1e-9);
    params.setAbsoluteErrorTol(1e-9);
    
    if(gps_en)
    {
    addloopfactor_ = false;
    addrtkfactor_ = true;
    buildBatchGraph();
    gtsam::LevenbergMarquardtOptimizer optimizer(gtSAMgraph, initialEstimate);
    gtsam::Values result = optimizer.optimize();
    initialEstimate = result;
    std::cout << "[Offline Optimization] First optimization pass done." <<  std::endl;
    }

    if(loopClosureEnable_)
    {
    addloopfactor_ = true;
    addrtkfactor_ = false;
    buildBatchGraph();
    gtsam::LevenbergMarquardtOptimizer optimizer_pass2(gtSAMgraph, initialEstimate);
    gtsam::Values result_pass2 = optimizer_pass2.optimize();
    initialEstimate = result_pass2;
    std::cout << "[Offline Optimization] Second optimization pass done." <<  std::endl;
    }

    is_optimized = true;

    int numPoses = initialEstimate.size();

    for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 rtk_optimizedPose = initialEstimate.at<gtsam::Pose3>(i);
        cloudKeyPoses6D->points[i].x = rtk_optimizedPose.translation().x();
        cloudKeyPoses6D->points[i].y = rtk_optimizedPose.translation().y();
        cloudKeyPoses6D->points[i].z = rtk_optimizedPose.translation().z();
        cloudKeyPoses6D->points[i].roll  = rtk_optimizedPose.rotation().roll();
        cloudKeyPoses6D->points[i].pitch = rtk_optimizedPose.rotation().pitch();
        cloudKeyPoses6D->points[i].yaw   = rtk_optimizedPose.rotation().yaw();
    }

    writeOptimizedTumTrajectory();
    std::cout << "[Offline Optimization] Optimized TUM trajectory saved "  << std::endl;
    writeRtkTumTrajectory();
    std::cout << "[Offline Optimization] RTK TUM trajectory saved " << std::endl;

    for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 rtk_optimizedPose = initialEstimate.at<gtsam::Pose3>(i);
        gtsam::Pose3 imu_optimizedPose = rtk_optimizedPose.compose((T_imu_rtk).inverse());
        cloudKeyPoses6D->points[i].x = imu_optimizedPose.translation().x();
        cloudKeyPoses6D->points[i].y = imu_optimizedPose.translation().y();
        cloudKeyPoses6D->points[i].z = imu_optimizedPose.translation().z();
        cloudKeyPoses6D->points[i].roll  = imu_optimizedPose.rotation().roll();
        cloudKeyPoses6D->points[i].pitch = imu_optimizedPose.rotation().pitch();
        cloudKeyPoses6D->points[i].yaw   = imu_optimizedPose.rotation().yaw();
    }

    std::cout << "[Offline Optimization] Saving maps and trajectories..." << std::endl;

    saveOptimizedGlobalMap();
    std::cout << "[Offline Optimization] Global map saved " << std::endl;

    std::cout << "[Offline Optimization] Finished." << std::endl;
}

bool optimization::saveFrame()
{
    if(cloudKeyPoses3D->points.empty())
    return true;

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
        abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
        return false;

    return true;  
}


void optimization::loadData(const std::string& data_dir)
{
    ROS_INFO("Loading data from directory: %s", data_dir.c_str());

    //  加载 IMU 数据 
    std::string imu_path = data_dir + "/imu.txt"; 
    std::ifstream imu_file(imu_path);
    
    if (imu_file.is_open()) {
        std::string line;
        int imu_count = 0;
        imuQueue.clear(); 

        while (std::getline(imu_file, line)) {
            if (line.empty() || line[0] == '#') continue; 

            std::stringstream ss(line);
            double t, ax, ay, az, gx, gy, gz;

            ss >> t >> ax >> ay >> az >> gx >> gy >> gz;

            imudata imu_msg(t, Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az));
            imuQueue.push_back(imu_msg);
            imu_count++;
        }
        imu_file.close();
        ROS_INFO("Loaded %d IMU measurements.", imu_count);
    } else {
        ROS_WARN("Failed to open IMU file: %s", imu_path.c_str());
    }

    // 加载 RTK/GPS 数据 
    std::string rtk_path = data_dir + "/rtk.txt"; 
    std::ifstream rtk_file(rtk_path);

    if (rtk_file.is_open()) {
        std::string line;
        int rtk_count = 0;
        gpsQueue.clear(); 

        while (std::getline(rtk_file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ss(line);
            double t, x, y, z, vx, vy, vz, h_acc, v_acc;
            ss >> t >> x >> y >> z >> vx >> vy >> vz >> h_acc >> v_acc;
            nav_msgs::Odometry gps_msg;
            gps_msg.header.stamp = ros::Time().fromSec(t) - ros::Duration(gps_offset);
            gps_msg.header.frame_id = "map"; 

            gps_msg.pose.pose.position.x = x;
            gps_msg.pose.pose.position.y = y;
            gps_msg.pose.pose.position.z = z;

            gps_msg.pose.pose.orientation.w = 1.0;

            gps_msg.twist.twist.linear.x = vx;
            gps_msg.twist.twist.linear.y = vy;
            gps_msg.twist.twist.linear.z = vz;

            for(int i=0; i<36; i++) gps_msg.pose.covariance[i] = 0.0;
            gps_msg.pose.covariance[0] = h_acc;  // x variance
            gps_msg.pose.covariance[7] = h_acc;  // y variance
            gps_msg.pose.covariance[14] = v_acc; // z variance

            gpsQueue.push_back(gps_msg);
            rtk_count++;
        }
        rtk_file.close();
        ROS_INFO("Loaded %d RTK measurements.", rtk_count);
    } else {
        ROS_WARN("Failed to open RTK file: %s", rtk_path.c_str());
    }

    // 加载 KeyFrames (Odom + Cov + PCD) 
    std::string odom_path = data_dir + "/odom.txt";
    std::string cov_path  = data_dir + "/cov.txt";
    std::string pcd_dir   = data_dir + "pcd/";

    std::ifstream odom_file(odom_path);
    std::ifstream cov_file(cov_path);

    if (!odom_file.is_open() || !cov_file.is_open()) {
        ROS_ERROR("Failed to open trajectory files. Check path: %s", data_dir.c_str());
        return;
    }

    std::string line_odom, line_cov;
    int processed_count = 0;

    while (std::getline(odom_file, line_odom) && std::getline(cov_file, line_cov)) {
        
        if (line_odom.empty() || line_odom[0] == '#') continue;
        if (line_cov.empty() || line_cov[0] == '#') continue; 

        std::stringstream ss_odom(line_odom);
        std::stringstream ss_cov(line_cov);

        double t_odom, t_cov;
        
        ss_odom >> t_odom;
        ss_cov >> t_cov;

        if (std::abs(t_odom - t_cov) > 1e-5) {
            ROS_ERROR("Critical Error: Timestamp mismatch at line %d!", processed_count + 1);
            ROS_ERROR("Odom Time: %.6f, Cov Time: %.6f", t_odom, t_cov);
            break; 
        }

        //  Odom 
        double tx, ty, tz, qx, qy, qz, qw;
        ss_odom >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        timeLaserInfoCur = t_odom;
        timeLaserInoStamp = ros::Time().fromSec(t_odom);

        tf::Quaternion orientation(qx, qy, qz, qw);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        
        transformTobeMapped[0] = roll;
        transformTobeMapped[1] = pitch;
        transformTobeMapped[2] = yaw;
        transformTobeMapped[3] = tx;
        transformTobeMapped[4] = ty;
        transformTobeMapped[5] = tz;

        //  Cov 
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                double val;
                ss_cov >> val;
                odomCovariance(i, j) = val;
            }
        }

        //  PCD
        std::stringstream ss_filename;
        ss_filename << std::setw(6) << std::setfill('0') << processed_count;
        std::string pcd_path = pcd_dir + ss_filename.str() + ".pcd";

        laserCloudSurfLastDS->clear();
        if (pcl::io::loadPCDFile(pcd_path, *laserCloudSurfLastDS) == -1) {
            ROS_ERROR("Missing PCD file for timestamp %.6f: %s", t_odom, pcd_path.c_str());
            continue; 
        }

        saveKeyFramesAndFactor();
        
        processed_count++;

    }
    odom_file.close();
    cov_file.close();
    
    ROS_INFO("Load complete. Total frames: %d", processed_count);
}

void optimization::saveKeyFramesAndFactor()
{
    std::lock_guard<std::mutex> lock(mutex);
    //存储位姿
    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 currentPose = trans2gtsamPose(transformTobeMapped);
    initialEstimate.insert(cloudKeyPoses6D->size(), currentPose);

    thisPose3D.x = currentPose.translation().x();
    thisPose3D.y = currentPose.translation().y();
    thisPose3D.z = currentPose.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size();
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll  = currentPose.rotation().roll();
    thisPose6D.pitch = currentPose.rotation().pitch();
    thisPose6D.yaw   = currentPose.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);
    //存储点云
    PointCloudXYZRGB::Ptr thisSurfKeyFrame(new PointCloudXYZRGB());
    pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    //存储协方差
    keyFrameCovariances.push_back(odomCovariance);
}

double calculateDtwTimeOffset(
    const std::vector<std::vector<double>>& gpsdata,
    const std::vector<std::vector<double>>& slamdata)
{
    const double MIN_EFFECTIVE_VELOCITY = 0.5; // m/s
    const int IDX_TIME = 0;
    const int IDX_X = 1;
    const int IDX_Y = 2;
    const int IDX_Z = 3;

    auto process_data = [=](const std::vector<std::vector<double>>& data_in) 
        -> std::pair<std::vector<double>, std::vector<double>>
    {
        double duration = 50.0;
        const double t_start = data_in[0][IDX_TIME];
        const double t_end   = t_start + duration;

        std::vector<double> times;
        std::vector<double> norms;
        
        bool is_moving = false;
        Eigen::Vector3d origin_pos(0, 0, 0); 

        for (size_t i = 1; i < data_in.size(); ++i) {
            const auto& row_i = data_in[i];
            const auto& row_prev = data_in[i-1];

            const double current_time = row_i[IDX_TIME];
            if (current_time > t_end) {
                break; 
            }

            if (!is_moving) {
                double dt = row_i[IDX_TIME] - row_prev[IDX_TIME];
                if (dt < 1e-6) continue; 
                double dx = row_i[IDX_X] - row_prev[IDX_X];
                double dy = row_i[IDX_Y] - row_prev[IDX_Y];
                double dz = row_i[IDX_Z] - row_prev[IDX_Z];               
                double vel_norm = std::sqrt(dx*dx + dy*dy + dz*dz) / dt;

                double pos_norm = std::sqrt(row_i[IDX_X]*row_i[IDX_X] + row_i[IDX_Y]*row_i[IDX_Y] + row_i[IDX_Z]*row_i[IDX_Z]);
                if (vel_norm > MIN_EFFECTIVE_VELOCITY && pos_norm > MIN_EFFECTIVE_VELOCITY) {
                    is_moving = true;
                } else {
                    continue; 
                }
            } else {

                Eigen::Vector3d current_pos(row_i[IDX_X], row_i[IDX_Y], row_i[IDX_Z]);
                double pos_norm = current_pos.norm();

                times.push_back(row_i[IDX_TIME]);
                norms.push_back(pos_norm);
            }
        }
        
        return {times, norms};
    };

    auto [gps_pos_times, gps_pos_norms] = process_data(gpsdata);
    auto [slam_pos_times, slam_pos_norms] = process_data(slamdata);

    sample1 = &(slam_pos_norms[0]);
    sample2 = &(gps_pos_norms[0]);

    std::vector<std::pair<int, int>> dtw_path;
    DTW(slam_pos_norms, gps_pos_norms, dtw_path);


    double total_time_diff = 0.0;
    int valid_pairs = 0;

    for (const auto& pair : dtw_path) {

        if (pair.first < slam_pos_times.size() && pair.second < gps_pos_times.size()) {
            total_time_diff += (slam_pos_times[pair.first] - gps_pos_times[pair.second]);
            valid_pairs++;
        }
    }

    if (valid_pairs == 0) {

        return 0.0;
    }

    double avg_time_diff = total_time_diff / valid_pairs;

    std::ofstream slam_file("/home/nomai/ws_fastlivo2_rtk/src/FAST-LIVO2/Vel/slam_vel.txt");
    if (slam_file.is_open()) {
        slam_file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < slam_pos_norms.size(); ++i) {
            slam_file << slam_pos_times[i] << " " << slam_pos_norms[i] << std::endl;
        }
        slam_file.close();
    } else {
        ROS_WARN("[initialAlign] could not open gps_vel.txt");
    }

    std::ofstream gps_file("/home/nomai/ws_fastlivo2_rtk/src/FAST-LIVO2/Vel/gps_vel.txt");
    if (gps_file.is_open()) {
        gps_file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < gps_pos_norms.size(); ++i) {
          gps_file << gps_pos_times[i] << " " << gps_pos_norms[i] << std::endl;
        }
        gps_file.close();
    } else {
        ROS_WARN("[initialAlign] could not open gps_vel.txt ");
    }
    return avg_time_diff;
}

void optimization::initialAlign()
{

    std::vector<std::vector<double>> slamdata;
    slamdata.reserve(cloudKeyPoses6D->size());
    for (const auto& p : cloudKeyPoses6D->points) {
        slamdata.push_back({p.time, p.x, p.y, p.z});
    }
    
    std::vector<std::vector<double>> gpsdata;
    gpsdata.reserve(gpsQueue.size());
    for (const auto& g : gpsQueue) {
        gpsdata.push_back({
            g.header.stamp.toSec(),
            g.pose.pose.position.x,
            g.pose.pose.position.y,
            g.pose.pose.position.z
        });
    }

    double avg_time_diff = calculateDtwTimeOffset(gpsdata, slamdata);
    
    ros::Duration time_offset(avg_time_diff);

    for(auto& g : gpsQueue) {
        g.header.stamp += time_offset;
    }
 
    ROS_INFO("[initialAlign] 1. DTW calculated time offset: %.4f seconds.", avg_time_diff);

    //using spline
    std::cout << " [initialAlign] initial align (using Spline)... " << std::endl;

    if (cloudKeyPoses6D->empty() || gpsQueue.empty()) {
        ROS_WARN("[initialAlign] empty buffers.");
        return;
    }
    
    struct Sample { double t; Eigen::Vector3d p; };
    std::vector<Sample> samp; samp.reserve(gpsQueue.size());
    for (const auto& g : gpsQueue) {
        const double t = g.header.stamp.toSec();
        const auto& p  = g.pose.pose.position; 
        samp.push_back({ t, Eigen::Vector3d(p.x, p.y, p.z) });
    }
    std::sort(samp.begin(), samp.end(),
              [](const Sample& a, const Sample& b){ return a.t < b.t; });

    std::vector<double> T; T.reserve(samp.size());
    std::vector<double> X, Y, Z;
    X.reserve(samp.size()); Y.reserve(samp.size()); Z.reserve(samp.size());
    for (size_t i = 0; i < samp.size(); ++i) {
        if (i > 0 && std::fabs(samp[i].t - samp[i-1].t) < 1e-12) continue; 
        T.emplace_back(samp[i].t);
        X.emplace_back(samp[i].p.x());
        Y.emplace_back(samp[i].p.y());
        Z.emplace_back(samp[i].p.z());
    }
    if (T.size() < 2) {
        ROS_WARN("[initialAlign] not enough RTK samples for spline.");
        return;
    }

    auto buildSpline = [](const std::vector<double>& t,
                          const std::vector<double>& y,
                          std::vector<double>& M)->bool {
        const size_t n = t.size();
        M.assign(n, 0.0);
        if (n < 2) return false;
        if (n == 2) return true; // 两点线性
        std::vector<double> h(n-1);
        for (size_t i=0;i<n-1;++i) {
            h[i] = t[i+1]-t[i];
            if (h[i] <= 0) return false; // 要严格递增
        }
        const size_t m = n-2;
        std::vector<double> a(m), b(m), c(m), d(m);
        for (size_t i=0;i<m;++i) {
            a[i] = h[i]/6.0;
            b[i] = (h[i]+h[i+1])/3.0;
            c[i] = h[i+1]/6.0;
            d[i] = (y[i+2]-y[i+1])/h[i+1] - (y[i+1]-y[i])/h[i];
        }
        for (size_t i=1;i<m;++i) {
            double w = a[i]/b[i-1];
            b[i] -= w*c[i-1];
            d[i] -= w*d[i-1];
        }
        std::vector<double> Min(m, 0.0);
        if (m>0) {
            Min[m-1] = d[m-1]/b[m-1];
            for (int i=int(m)-2; i>=0; --i)
                Min[i] = (d[i] - c[i]*Min[i+1]) / b[i];
        }
        for (size_t i=0;i<m;++i) M[i+1] = Min[i]; 
        return true;
    };

    auto evalSpline = [](const std::vector<double>& t,
                         const std::vector<double>& y,
                         const std::vector<double>& M,
                         double tq)->double {
        const size_t n = t.size();
        if (n==0) return 0.0;
        if (n==1) return y[0];
        auto derivLeft = [&](){
            double h = t[1]-t[0];
            return (y[1]-y[0])/h - (h/6.0)*(2.0*M[0] + M[1]);
        };
        auto derivRight = [&](){
            size_t k=n-2; double h = t[k+1]-t[k];
            return (y[k+1]-y[k])/h + (h/6.0)*(2.0*M[k+1] + M[k]);
        };
        if (tq <= t.front()) return y.front() + derivLeft()  * (tq - t.front());
        if (tq >= t.back())  return y.back()  + derivRight() * (tq - t.back());
        size_t k = std::upper_bound(t.begin(), t.end(), tq) - t.begin() - 1;
        double h = t[k+1]-t[k];
        double A = (t[k+1]-tq)/h;
        double B = (tq - t[k])/h;
        return M[k]*(A*A*A)*h/6.0
             + M[k+1]*(B*B*B)*h/6.0
             + (y[k]   - M[k]*h*h/6.0)*A
             + (y[k+1] - M[k+1]*h*h/6.0)*B;
    };

    std::vector<double> Mx, My, Mz;
    if (!buildSpline(T, X, Mx) || !buildSpline(T, Y, My) || !buildSpline(T, Z, Mz)) {
        ROS_WARN("[initialAlign] spline build failed.");
        return;
    }

    auto find_nearest_gps = [&](double t_query) -> const nav_msgs::Odometry* {
        if (gpsQueue.empty()) return nullptr;

        auto it_lower = std::lower_bound(gpsQueue.begin(), gpsQueue.end(), t_query,
            [](const nav_msgs::Odometry& msg, double t) {
                return msg.header.stamp.toSec() < t;
            });

        if (it_lower == gpsQueue.begin()) {
            return &(*it_lower);
        }

        if (it_lower == gpsQueue.end()) {
            return &(*std::prev(it_lower));
        }

        const nav_msgs::Odometry& after = *it_lower;
        const nav_msgs::Odometry& before = *std::prev(it_lower);

        double dt_after = std::abs(after.header.stamp.toSec() - t_query);
        double dt_before = std::abs(t_query - before.header.stamp.toSec());

        if (dt_after < dt_before) {
            return &after;
        } else {
            return &before;
        }
    };
    
    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) 
    {
        const double tk = cloudKeyPoses6D->points[i].time;

        nav_msgs::Odometry gps_odom_B;
        gps_odom_B.header.stamp = ros::Time().fromSec(tk);
        gps_odom_B.header.frame_id = "map";
        gps_odom_B.pose.pose.position.x = evalSpline(T, X, Mx, tk);;
        gps_odom_B.pose.pose.position.y = evalSpline(T, Y, My, tk);
        gps_odom_B.pose.pose.position.z = evalSpline(T, Z, Mz, tk);
        gps_odom_B.pose.pose.orientation.w = 1.0; 

        const nav_msgs::Odometry* nearest_gps = find_nearest_gps(tk);
        gps_odom_B.pose.covariance[0]  = nearest_gps->pose.covariance[0]; 
        gps_odom_B.pose.covariance[7]  = nearest_gps->pose.covariance[7];  
        gps_odom_B.pose.covariance[14] = nearest_gps->pose.covariance[14];
        gpsQueue_B.push_back(gps_odom_B);
    }

    std::vector<Eigen::Vector3d> A_slam_gps; 
    std::vector<Eigen::Vector3d> B_enu_gps;  

    const double t_start = cloudKeyPoses6D->points[0].time;
    const double t_end_svd = t_start + 50.0; 

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) 
    {
        const auto& slam_pose_imu = cloudKeyPoses6D->points[i];
        const double tk = slam_pose_imu.time;
        
        if (tk > t_end_svd) break; 
        if (tk < T.front() || tk > T.back()) continue; 

        Eigen::Vector3d p_enu_gps;
        p_enu_gps.x() = evalSpline(T, X, Mx, tk);
        p_enu_gps.y() = evalSpline(T, Y, My, tk);
        p_enu_gps.z() = evalSpline(T, Z, Mz, tk);
        B_enu_gps.push_back(p_enu_gps);

        gtsam::Pose3 T_slam_imu = initialEstimate.at<gtsam::Pose3>(i); 
        gtsam::Pose3 T_slam_gps = T_slam_imu.compose(T_imu_rtk);
        
        A_slam_gps.push_back(T_slam_gps.translation());
    }

    if (A_slam_gps.size() < 3) {
        ROS_WARN("[initialAlign] too few spline pairs to align.");
        return;
    }

    gtsam::Pose3 T_enu_slam = computeSVD(B_enu_gps, A_slam_gps);

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) {
        gtsam::Pose3 T_slam_imu = initialEstimate.at<gtsam::Pose3>(i);
        gtsam::Pose3 T_slam_gps = T_slam_imu.compose(T_imu_rtk);
        gtsam::Pose3 T_enu_gps = T_enu_slam.compose(T_slam_gps);

        initialEstimate.update(i, T_enu_gps);
        
        const auto& p = T_enu_gps.translation();
        const auto& r = T_enu_gps.rotation().rpy();
        cloudKeyPoses6D->points[i].x = p.x();
        cloudKeyPoses6D->points[i].y = p.y();
        cloudKeyPoses6D->points[i].z = p.z();
        cloudKeyPoses6D->points[i].roll  = r(0);
        cloudKeyPoses6D->points[i].pitch = r(1);
        cloudKeyPoses6D->points[i].yaw   = r(2);
    }
    
    ROS_INFO("[initialAlign] Spline-based alignment complete.");
}


// void optimization::propagateGps()
// {
//     ImuParamInit();
//     propagated_gps_.clear();
//     auto imu_iter = imuQueue.begin();
//     auto gps_iter = gpsQueue.begin();

//     for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i)
//     {
//         double t_keyframe = cloudKeyPoses6D->points[i].time;

//         auto gps_start_iter = gps_iter;
//         while (gps_start_iter != gpsQueue.end() &&
//                (gps_start_iter + 1) != gpsQueue.end() &&
//                (gps_start_iter + 1)->header.stamp.toSec() <= t_keyframe)
//         {
//             gps_start_iter++;
//         }
//         if (gps_start_iter == gpsQueue.end() ||
//             gps_start_iter->header.stamp.toSec() > t_keyframe)
//         {
//             continue;
//         }
//         gps_iter = gps_start_iter; 

//         const auto& gps_msg = *gps_start_iter;
//         double t_gps = gps_msg.header.stamp.toSec();

//         gtsam::Point3 P_WG_A_start(gps_msg.pose.pose.position.x,
//                                    gps_msg.pose.pose.position.y,
//                                    gps_msg.pose.pose.position.z);

//         gtsam::Rot3 R_WG_A_start;
//          auto nearest_aligned_slam_pose_it = std::min_element(
//              cloudKeyPoses6D->points.begin(), cloudKeyPoses6D->points.end(),
//              [&](const PointTypePose& a, const PointTypePose& b){
//                  return std::abs(a.time - t_gps) < std::abs(b.time - t_gps);
//              });
//         if (nearest_aligned_slam_pose_it != cloudKeyPoses6D->points.end() &&
//             std::abs(nearest_aligned_slam_pose_it->time - t_gps) < 0.2) 
//         {
//             R_WG_A_start = gtsam::Rot3::RzRyRx(nearest_aligned_slam_pose_it->roll,
//                                                nearest_aligned_slam_pose_it->pitch,
//                                                nearest_aligned_slam_pose_it->yaw);
//         } else {
//              R_WG_A_start = gtsam::Rot3::RzRyRx(cloudKeyPoses6D->points[i].roll,
//                                                 cloudKeyPoses6D->points[i].pitch,
//                                                 cloudKeyPoses6D->points[i].yaw);
//         }


//         gtsam::Pose3 T_WG_A_start(R_WG_A_start, P_WG_A_start);

//         gtsam::Vector3 V_WG_A_start(gps_msg.twist.twist.linear.x,
//                                     gps_msg.twist.twist.linear.y,
//                                     gps_msg.twist.twist.linear.z);

//         gtsam::Pose3 T_WG_B_start = T_WG_A_start.compose((T_imu_rtk).inverse());
//         gtsam::Vector3 V_WG_B_start = V_WG_A_start; 

//         gtsam::NavState navstate_start_IMU(T_WG_B_start, V_WG_B_start);

//         while (imu_iter != imuQueue.end() && imu_iter->timestamp_ <= t_gps) {
//             imu_iter++;
//         }
//         gtsam::PreintegratedImuMeasurements imu_propagator(imu_params_gtsam_, initial_bias_guess_);

//         double last_imu_time = t_gps;
//         auto imu_window_iter = imu_iter;

//         while (imu_window_iter != imuQueue.end() && imu_window_iter->timestamp_ <= t_keyframe)
//         {
//             double t_imu = imu_window_iter->timestamp_;
//             double dt = t_imu - last_imu_time;
//             if (dt <= 0) { ++imu_window_iter; continue; } 
//             imu_propagator.integrateMeasurement(Eigen::Vector3d(imu_window_iter->acce_),
//                                                 Eigen::Vector3d(imu_window_iter->gyro_), dt);
//             last_imu_time = t_imu;
//             ++imu_window_iter;
//         }

//         gtsam::NavState predicted_state_IMU = imu_propagator.predict(navstate_start_IMU, initial_bias_guess_);
//         gtsam::Pose3 T_WG_A_final = predicted_state_IMU.pose().compose(T_imu_rtk);

//         propagated_gps_[i] = T_WG_A_final.translation();
//     }
// }


VoxelMapManagerPtr optimization::buildLocalMap(double timestamp, int kf_index, int temporal_radius)
{

    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> temp_voxel_map;
    VoxelMapManagerPtr local_map = std::make_shared<VoxelMapManager>(voxel_config_, temp_voxel_map);

    std::vector<int> neighbor_gtsam_keys;
    for (int i = -temporal_radius; i <= temporal_radius; ++i)
    {
        int current_key = kf_index + i;
        
        if (current_key < 0 || current_key >= cloudKeyPoses3D->size()) {
            continue;
        }
        
        neighbor_gtsam_keys.push_back(current_key);
    }
    
    std::vector<pointWithVar> points_to_add;
    
    const gtsam::Values& current_pose_estimates =  initialEstimate;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_map_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (int neighbor_gtsam_key : neighbor_gtsam_keys)
    {
        gtsam::Pose3 neighbor_pose;
        if (current_pose_estimates.exists(neighbor_gtsam_key)) {
             neighbor_pose = current_pose_estimates.at<gtsam::Pose3>(neighbor_gtsam_key);
        } else {
             ROS_WARN("buildLocalMap: Key %d (temporal search) does not exist in the provided Values object.", neighbor_gtsam_key);
             continue; 
        }
        
        if (neighbor_gtsam_key < 0 || neighbor_gtsam_key >= surfCloudKeyFrames.size()) continue; 
        
        PointCloudXYZRGB::Ptr neighbor_cloud = surfCloudKeyFrames[neighbor_gtsam_key];

        for (const auto& pt_rgb : neighbor_cloud->points)
        {
            gtsam::Point3 pt_body(pt_rgb.x, pt_rgb.y, pt_rgb.z);
            gtsam::Point3 pt_world = neighbor_pose.transformFrom(pt_body);
            
            pointWithVar pv;
            pv.point_w = pt_world;
            points_to_add.push_back(pv);

            pcl::PointXYZRGB pt_world_rgb;
            pt_world_rgb.x = pt_world.x();
            pt_world_rgb.y = pt_world.y();
            pt_world_rgb.z = pt_world.z();
            pt_world_rgb.r = pt_rgb.r;
            pt_world_rgb.g = pt_rgb.g;
            pt_world_rgb.b = pt_rgb.b;
            local_map_cloud->push_back(pt_world_rgb);
        }
    }

    if (!points_to_add.empty()) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*local_map_cloud, cloud_msg); 
        cloud_msg.header.stamp = ros::Time().fromSec(timestamp); 
        cloud_msg.header.frame_id = "map"; 
        pubLoopLocalMap.publish(cloud_msg);
        local_map->UpdateVoxelMap(points_to_add);
    }
    
    return local_map;
}

void optimization::buildBatchGraph()
{
    gtSAMgraph.resize(0); 

    ROS_INFO("rtk_cox, livo2_RPY_cov, livo2_XYZ_cov, loop_cov: %f, %f, %f, %f", rtk_cov, livo2_RPY_cov, livo2_XYZ_cov, loop_cov);
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = 
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << livo2_RPY_cov, livo2_RPY_cov, livo2_RPY_cov, livo2_XYZ_cov, livo2_XYZ_cov, livo2_XYZ_cov).finished());
 
    // gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
    //     gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << rtk_cov, rtk_cov, rtk_cov).finished());

    gtsam::SharedNoiseModel pointToPlaneNoise = gtsam::noiseModel::Isotropic::Sigma(1, loop_cov);

    kdtreeHistoryKeyPoses_->setInputCloud(cloudKeyPoses3D);
    size_t rtk_idx = 0; 
    int loopCount = 0;

    for (size_t i = 0; i < initialEstimate.size(); ++i)
    {
        //addpriorfactor and betweenfactor
        if (i == 0)
        {
            gtsam::Pose3 priorPose_T_wa = initialEstimate.at<gtsam::Pose3>(0);
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, priorPose_T_wa, priorNoise));
        }
        else 
        {
            gtsam::Pose3 poseFrom_T_wa = initialEstimate.at<gtsam::Pose3>(i-1);
            gtsam::Pose3 poseTo_T_wa   = initialEstimate.at<gtsam::Pose3>(i);
            gtsam::Pose3 T_rel_ant_from_svd = poseFrom_T_wa.between(poseTo_T_wa);
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(i-1, i, T_rel_ant_from_svd, odometryNoise));
        }

        //addrtkfactor

        if(addrtkfactor_)
        {
            auto it = gpsQueue_B[i];
            double x = it.pose.pose.position.x;
            double y = it.pose.pose.position.y;
            double z = it.pose.pose.position.z;
            gtsam::Point3 gps_point(x, y, z);

            double cov_x = it.pose.covariance[0]; 
            double cov_y = it.pose.covariance[7]; 
            double cov_z = it.pose.covariance[14]; 

            gtsam::Vector3 gps_variance(cov_x, cov_y, 100000);
            auto gps_noise = gtsam::noiseModel::Diagonal::Variances(gps_variance);
            gtSAMgraph.add(gtsam::GPSFactor(i, gps_point, gps_noise));
            aLoopIsClosed = true; 
        
        }

        //addloopfactor
        if(addloopfactor_ == false) {
            continue;
        }

        pcl::PointCloud<PointType>::Ptr currentPose3d(new pcl::PointCloud<PointType>());
        currentPose3d->points.reserve(initialEstimate.size());
        for (size_t i = 0; i < initialEstimate.size(); ++i)
        {
            if (!initialEstimate.exists(i)) continue;
            
            gtsam::Pose3 pose = initialEstimate.at<gtsam::Pose3>(i);
            PointType pt;
            pt.x = pose.translation().x();
            pt.y = pose.translation().y();
            pt.z = pose.translation().z();
            pt.intensity = static_cast<float>(i); 
            currentPose3d->push_back(pt);
        }

        double currentTime = cloudKeyPoses6D->points[i].time;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        PointType currentPose = currentPose3d->points[i];
        kdtreeHistoryKeyPoses_->radiusSearch(currentPose, loopClosureSearchRadius_, pointSearchInd, pointSearchSqDis, 0); 
        
        for (int k = 0; k < pointSearchInd.size(); ++k)
        {
            int historyIndex = pointSearchInd[k];
            if (historyIndex >= static_cast<int>(i) - 50) continue; 
            double historyTime = cloudKeyPoses6D->points[historyIndex].time;
            if (std::abs(currentTime - historyTime) < loopClosureTimeThreshold_) continue; 


            //ROS_INFO("Searching Loop Closure: Current Index %zu -- History Index %d", i,  historyIndex);
            int points_added = 0;
            int buildlocalmapradius = 10;
            VoxelMapManagerPtr map_target = buildLocalMap(currentTime, historyIndex, buildlocalmapradius);
            if (map_target == nullptr || map_target->voxel_map_.empty()) continue; 

            PointCloudXYZRGB::Ptr cloud_source_rgb = surfCloudKeyFrames[i];
            if (cloud_source_rgb->empty()) continue;
            double voxel_size = map_target->config_setting_.max_voxel_size_; 
            gtsam::Pose3 T_wi = initialEstimate.at<gtsam::Pose3>(i);
            Eigen::Affine3f T_wi_affine(T_wi.matrix().cast<float>());
            PointCloudXYZRGB::Ptr cloud_source_world(new PointCloudXYZRGB());
            pcl::transformPointCloud(*cloud_source_rgb, *cloud_source_world, T_wi_affine);

            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud_source_world, cloud_msg);
            cloud_msg.header.stamp = ros::Time().fromSec(currentTime); 
            cloud_msg.header.frame_id = "map"; 
            pubLoopCurrentScan.publish(cloud_msg);

            for (const auto& pt_rgb : cloud_source_rgb->points) 
            {
                gtsam::Point3 p_i(pt_rgb.x, pt_rgb.y, pt_rgb.z);

                gtsam::Point3 p_w_gtsam = T_wi.transformFrom(p_i);
                Eigen::Vector3d p_w = p_w_gtsam;

                float loc_xyz[3];
                for(int j=0; j<3; j++) {
                    loc_xyz[j] = p_w[j] / voxel_size;
                    if(loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
                }
                VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]); 

                auto iter = map_target->voxel_map_.find(position); 
                //ROS_INFO("Searching VoxelOctoTree for point (%.2f, %.2f, %.2f)", p_w[0], p_w[1], p_w[2]);
                if (iter == map_target->voxel_map_.end()) {
                    continue; 
                }
                points_added++;
            }

            if (points_added == 0) 
            {
                break;
            }

            if (points_added < 3000) 
            {
                k+=5; 
                for (auto& pair : map_target->voxel_map_) 
                {
                    if (pair.second != nullptr) {
                        delete pair.second; // pair.second 是 VoxelOctoTree*
                    }
                }
                map_target->voxel_map_.clear();
                continue;
            }

            for (const auto& pt_rgb : cloud_source_rgb->points) 
            {
                gtsam::Point3 p_i(pt_rgb.x, pt_rgb.y, pt_rgb.z);

                gtsam::Point3 p_w_gtsam = T_wi.transformFrom(p_i);
                Eigen::Vector3d p_w = p_w_gtsam;

                float loc_xyz[3];
                for(int j=0; j<3; j++) {
                    loc_xyz[j] = p_w[j] / voxel_size;
                    if(loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
                }
                VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]); 

                auto iter = map_target->voxel_map_.find(position); 
                //ROS_INFO("Searching VoxelOctoTree for point (%.2f, %.2f, %.2f)", p_w[0], p_w[1], p_w[2]);
                if (iter == map_target->voxel_map_.end()) {
                    continue; 
                }

                VoxelOctoTree* root_node = iter->second;
                VoxelOctoTree* leaf = root_node->find_correspond(p_w); 

                if (leaf != nullptr && leaf->octo_state_ == 0 && leaf->plane_ptr_ != nullptr && leaf->plane_ptr_->is_plane_) 
                {
                    const VoxelPlane& plane_w = *(leaf->plane_ptr_); 
                
                    
                    gtSAMgraph.add(boost::make_shared<PointToMapFactor>(
                        i,                 
                        p_i,               
                        plane_w,           
                        pointToPlaneNoise  
                    ));
                }
            }
            
            ROS_INFO("Tight-Coupled Loop Closure Added (%d factors) between %d and %zu",
                    points_added, historyIndex, i);


            for (auto& pair : map_target->voxel_map_) 
            {
                if (pair.second != nullptr) {
                    delete pair.second; // pair.second 是 VoxelOctoTree*
                }
            }
            map_target->voxel_map_.clear();

            break;
        }
    }
    
} 

void optimization::syncedCallback(const nav_msgs::Odometry::ConstPtr& odomMsg, const sensor_msgs::PointCloud2::ConstPtr& cloudMsg)
{
    //ROS_INFO("Optimization got synced data");
    timeLaserInoStamp = cloudMsg->header.stamp;
    timeLaserInfoCur = cloudMsg->header.stamp.toSec();
    //ROS_INFO("timeLaserInfoCur: %f", timeLaserInfoCur);
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    transformTobeMapped[0] = roll;
    transformTobeMapped[1] = pitch;
    transformTobeMapped[2] = yaw;
    transformTobeMapped[3] = odomMsg->pose.pose.position.x;
    transformTobeMapped[4] = odomMsg->pose.pose.position.y;
    transformTobeMapped[5] = odomMsg->pose.pose.position.z;
    //ROS_INFO("transformTobeMapped: %f, %f, %f, %f, %f, %f", transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2], transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
   
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov_map(odomMsg->pose.covariance.data());
    odomCovariance = cov_map;
    
    laserCloudSurfLastDS->clear();
    pcl::fromROSMsg(*cloudMsg, *laserCloudSurfLastDS);

    saveKeyFramesAndFactor();

    if (save_pcd_enable_)
    {
        std::string filename = pcd_save_directory_ + "/" + std::to_string(frame_counter_++) + ".pcd";

        ROS_INFO("Saving PCD file: %s", filename.c_str());
        if (pcl::io::savePCDFileBinary(filename, *laserCloudSurfLastDS) == -1)
        {
            ROS_WARN("Failed to save PCD file: %s", filename.c_str());
        }
    }
    //publishGlobalMap();
}

void optimization::gpsHandler(const gnss_comm::GnssPVTSolnMsg::ConstPtr& gpsMsg)
{
    // if (gpsMsg->h_acc > 0.02)
    // {
    //     return; 
    // }

    Eigen::Vector3d trans_local_;
    static bool first_gps = false;
    if (!first_gps) {
        first_gps = true;
        gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
    }

    const double GPS_EPOCH_UNIX_TIME = 315964800.0;
    const double SECONDS_PER_WEEK = 604800.0; 
    const double LEAP_SECONDS = 18.0;
    int week = gpsMsg->time.week;
    double tow = gpsMsg->time.tow; 
    double total_gps_seconds = (double)week * SECONDS_PER_WEEK + tow;
    double timestamp_sec = total_gps_seconds + GPS_EPOCH_UNIX_TIME - LEAP_SECONDS;
    ros::Time stamp;
    stamp.fromSec(timestamp_sec);
    
    gpstimestamp = timestamp_sec; 


    gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

    nav_msgs::Odometry gps_odom;
    gps_odom.header.stamp = stamp - ros::Duration(gps_offset); 
    gps_odom.header.frame_id = "map";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    
    gps_odom.twist.twist.linear.x = gpsMsg->vel_e;
    gps_odom.twist.twist.linear.y = gpsMsg->vel_n;
    gps_odom.twist.twist.linear.z = -gpsMsg->vel_d;

    gps_odom.pose.covariance[0] = gpsMsg->h_acc; 
    gps_odom.pose.covariance[7] = gpsMsg->h_acc; 
    gps_odom.pose.covariance[14] = gpsMsg->v_acc; 

    pubGpsOdom.publish(gps_odom);
    gpsQueue.push_back(gps_odom);

}

// void optimization::gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
// {
//     Eigen::Vector3d trans_local_;
//     static bool first_gps = false; 
//     if (!first_gps) {
//         first_gps = true;
//         gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
//     }
//     ros::Time stamp = gpsMsg->header.stamp;
//     gpstimestamp = stamp.toSec(); 

//     gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, 
//                        trans_local_[0], trans_local_[1], trans_local_[2]);

//     nav_msgs::Odometry gps_odom;
//     gps_odom.header.stamp = stamp - ros::Duration(gps_offset); 
//     gps_odom.header.frame_id = "map";
//     gps_odom.pose.pose.position.x = trans_local_[0];
//     gps_odom.pose.pose.position.y = trans_local_[1];
//     gps_odom.pose.pose.position.z = trans_local_[2];
    
//     gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

//     pubGpsOdom.publish(gps_odom);
//     gpsQueue.push_back(gps_odom);

// }

void optimization::imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
    imudata data_point;
    data_point.timestamp_ = imuMsg->header.stamp.toSec();
    data_point.acce_[0] = imuMsg->linear_acceleration.x;
    data_point.acce_[1] = imuMsg->linear_acceleration.y;
    data_point.acce_[2] = imuMsg->linear_acceleration.z;

    data_point.gyro_[0] = imuMsg->angular_velocity.x;
    data_point.gyro_[1] = imuMsg->angular_velocity.y;
    data_point.gyro_[2] = imuMsg->angular_velocity.z;

    imuQueue.push_back(data_point);
}

bool optimization::initImudata()
{
    if (init_imuQueue.size() < 10) {
    return false;
    }

    Eigen::Vector3d mean_gyro, mean_acce;
    ComputeMeanAndCovDiag(init_imuQueue, mean_gyro, cov_gyro_, [](const imudata& imu) { return imu.gyro_; });
    ComputeMeanAndCovDiag(init_imuQueue, mean_acce, cov_acce_, [this](const imudata& imu) { return imu.acce_; });

    gravity_ = -mean_acce / mean_acce.norm() * gravity_norm_;

    ComputeMeanAndCovDiag(init_imuQueue, mean_acce, cov_acce_,
                                [this](const imudata& imu) { return imu.acce_ + gravity_; });

    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    return true;
}

bool optimization::ImuParamInit()
{
    const int init_count = 50; 

    if (imuQueue.size() < init_count)
    {
        return false;
    }

    init_imuQueue.clear();
    auto it = imuQueue.begin();

    for (int i = 0; i < init_count; ++i, ++it) 
    {
        init_imuQueue.push_back(*it);
    }

    if (!initImudata()) 
    {
        return false;
    }

    imu_params_gtsam_ = gtsam::PreintegrationParams::MakeSharedU(gravity_norm_);
    imu_params_gtsam_->accelerometerCovariance = cov_acce_.asDiagonal();
    imu_params_gtsam_->gyroscopeCovariance = cov_gyro_.asDiagonal();

    initial_bias_guess_ = gtsam::imuBias::ConstantBias(init_ba_, init_bg_);
    imu_initialized_ = true;
    return true;    

}


void optimization::saveOptimizedGlobalMap()
{
    PointCloudXYZRGB::Ptr globalMapCloud(new PointCloudXYZRGB());

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i)
    {
       Eigen::Affine3f transform = pclPointToAffine3f(cloudKeyPoses6D->points[i]);
        PointCloudXYZRGB::Ptr original_cloud = surfCloudKeyFrames[i];
        PointCloudXYZRGB::Ptr transformed_cloud(new PointCloudXYZRGB());
        pcl::transformPointCloud(*original_cloud, *transformed_cloud, transform);
        *globalMapCloud += *transformed_cloud;
    }
    std::string save_path;
    if (is_optimized) 
    {
        save_path = global_map_pcd_path_ + "after_optimization.pcd";
    } 
    else
    {
        save_path = global_map_pcd_path_ + "before_optimization.pcd";
    }
    pcl::io::savePCDFileBinary(save_path, *globalMapCloud);

}

void optimization::writeOptimizedTumTrajectory() {
    if (cloudKeyPoses6D->empty()) {
        ROS_WARN("No optimized keyframes to write to TUM file.");
        return;
    }
    std::ofstream tum_file(opt_tum_output_path_);
    tum_file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    tum_file << std::fixed << std::setprecision(6);
    for (const auto& pose : cloudKeyPoses6D->points) {
        tf::Quaternion q = tf::createQuaternionFromRPY(pose.roll, pose.pitch, pose.yaw);
        tum_file << pose.time << " "
                 << pose.x << " " << pose.y << " " << pose.z << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    // int id = 0;

    // for (const auto& pose : cloudKeyPoses6D->points) {
    //     tf::Quaternion q = tf::createQuaternionFromRPY(pose.roll, pose.pitch, pose.yaw);
    //     tum_file << id << " "
    //             << pose.x << " " << pose.y << " " << pose.z << " "
    //             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    //     id++;
    // }
    tum_file.close();
}

void optimization::writeRtkTumTrajectory() {
    if (gpsQueue.empty()) {
        ROS_WARN("No RTK data to write to TUM file.");
        return;
    }
    
    std::ofstream tum_file(gps_tum_output_path_);
    tum_file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    tum_file << std::fixed << std::setprecision(6);

    for (const auto& odom_msg : gpsQueue) {
        tum_file << odom_msg.header.stamp.toSec() << " " 
                 << odom_msg.pose.pose.position.x << " " 
                 << odom_msg.pose.pose.position.y << " " 
                 << odom_msg.pose.pose.position.z << " "
                 << odom_msg.pose.pose.orientation.x << " " 
                 << odom_msg.pose.pose.orientation.y << " " 
                 << odom_msg.pose.pose.orientation.z << " " 
                 << odom_msg.pose.pose.orientation.w << std::endl;
    }

    tum_file.close();
}