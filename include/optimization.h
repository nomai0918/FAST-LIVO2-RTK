#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gnss_comm/GnssPVTSolnMsg.h>


#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/gicp.h> 
#include <pcl/filters/voxel_grid.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <GeographicLib/LocalCartesian.hpp> 

#include <sensor_msgs/Imu.h>

#include "LIVMapper.h"
#include "FastDTW/example.hpp"
#include <memory>

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D;                  
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
}EIGEN_ALIGN16;      

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
}

gtsam::Pose3 computeSVD(const std::vector<Eigen::Vector3d>& target, 
                        const std::vector<Eigen::Vector3d>& source)
{
    if (target.empty() || target.size() != source.size()) {
        return gtsam::Pose3::Identity(); 
    }

    Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d source_center = Eigen::Vector3d::Zero();
    for (const auto& p : target) target_center += p;
    for (const auto& p : source) source_center += p;
    target_center /= target.size();
    source_center /= source.size();

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < source.size(); ++i) {
        W += (target[i] - target_center) * (source[i] - source_center).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) { 
        R = svd.matrixU() * Eigen::DiagonalMatrix<double, 3>(1, 1, -1) * svd.matrixV().transpose();
    }

    Eigen::Vector3d t = target_center - R * source_center;
    
    return gtsam::Pose3{gtsam::Rot3(R), gtsam::Point3(t)};
}


Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
}

struct imudata {
    imudata() = default;
    imudata(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acce_ = Eigen::Vector3d::Zero();
};

class PointToMapFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
private:
    gtsam::Point3 point_in_body_; // 源点云在Lidar/Body系下的坐标 (p_i)
    VoxelPlane plane_in_world_; // 目标平面在世界系下的参数 (n_w, d_w)

public:
    // 构造函数：传入姿态的Key、Lidar系下的点、世界系下的平面、噪声模型
    PointToMapFactor(gtsam::Key pose_key, const gtsam::Point3& point_in_body,
                     const VoxelPlane& plane_in_world, gtsam::SharedNoiseModel noise_model) :
        NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
        point_in_body_(point_in_body),
        plane_in_world_(plane_in_world) {}

    /**
     * @brief 计算残差（点到平面的距离）
     * @param T_wi (T_world_from_imu) 当前GTSAM估计的姿态
     * @param H (optional) 用于存储雅可比矩阵
     * @return gtsam::Vector (1维残差)
     */
    gtsam::Vector evaluateError(const gtsam::Pose3& T_wi,
                                boost::optional<gtsam::Matrix&> H = boost::none) const override
    {

        gtsam::Point3 n_w(plane_in_world_.normal_.x(), 
                          plane_in_world_.normal_.y(), 
                          plane_in_world_.normal_.z());
        double d_w = plane_in_world_.d_;


        gtsam::Matrix36 H_transform; // 存储 d(p_w) / d(T_wi)
        gtsam::Point3 p_w = T_wi.transformFrom(point_in_body_, (H ? &H_transform : nullptr));

        //计算残差 e = n_w^T * p_w + d_w
        double residual = n_w.dot(p_w) + d_w;

        //计算雅可比矩阵 d(e) / d(T_wi)
        if (H)
        {
            // d(e)/d(T_wi) = d(e)/d(p_w) * d(p_w)/d(T_wi)
            // d(e)/d(p_w) = n_w^T  (1x3)
            // d(p_w)/d(T_wi) = H_transform (3x6)
            (*H) = n_w.transpose() * H_transform;
        }
        ROS_INFO("PointToMapFactor residual: %.6f", residual);
        return (gtsam::Vector(1) << residual).finished();
    }
};

class optimization
{
public:
    gtsam::NonlinearFactorGraph gtSAMgraph; 
    gtsam::Values initialEstimate;        
    gtsam::Values isamCurrentEstimate;
    gtsam::Values optimizedEstimate;

    optimization(ros::NodeHandle &nh);
    ~optimization();

    void loadData(const std::string& data_dir);
    void offlineOptimizationTask();

    // void odomHandler(const nav_msgs::Odometry::ConstPtr &odomAftMapped);
    // void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud);
    //void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg);
    void gpsHandler(const gnss_comm::GnssPVTSolnMsg::ConstPtr& pvtMsg);
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg);
    bool initImudata();
    bool ImuParamInit();

    bool saveFrame();
    void saveKeyFramesAndFactor();
    void addFastlivo2Factor();
    void addGPSFactor();
    void addLidarFactor();
    void addLoopFactor();
    void correctPoses();
    void syncedCallback(const nav_msgs::Odometry::ConstPtr& odomMsg, const sensor_msgs::PointCloud2::ConstPtr& cloudMsg);
    void publishFrames();
    void updatePath(const PointTypePose& pose_in);
    void toENU();
    void initialAlign();
    void propagateGps();
    void buildBatchGraph();

    VoxelMapManagerPtr buildLocalMap(double timestamp, int kf_index, int temporal_radius);
    void saveOptimizedGlobalMap();
    void writeOptimizedTumTrajectory();
    void writeRtkTumTrajectory();
    
    template<typename T>
    void publishCloud(const ros::Publisher& pub, const T& cloud, const ros::Time& stamp, const std::string& frame_id)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        pub.publish(msg);
    }

    void publishGlobalMap();
    void visualizeGlobalMapThread();

    std::unique_ptr<common_lib> common_lib_;
    ros::Subscriber subGPS;
    ros::Subscriber subImu;
    ros::Subscriber subGPS_pvt;

    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubGpsOdom;
    ros::Publisher pubGlobalMap;
    ros::Publisher pubLoopLocalMap;
    ros::Publisher pubLoopCurrentScan;

    nav_msgs::Path globalPath;

    message_filters::Subscriber<nav_msgs::Odometry> subOdom_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subCloud_;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncPolicy;

    size_t frame_counter_;

    float gpsCovThreshold;
    float poseCovThreshold;

    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float transformTobeMapped[6];
    float last_transformTobeMapped[6];

    float gpstimestamp = 0.0;
    float gps_offset;

    double rtk_cov;
    double livo2_RPY_cov;
    double livo2_XYZ_cov;
    double loop_cov;

    bool load_data = false;
    bool useGpsElevation = true;
    bool aLoopIsClosed = false;
    bool loopClosureEnable_;
    bool addloopfactor_ = false;
    bool addrtkfactor_ = false;
    bool save_pcd_enable_ = false;
    bool imu_initialized_ = false;
    bool is_optimized = false;

    float loopClosureSearchRadius_;   
    float loopClosureTimeThreshold_;    
    float loopClosureFitnessScore_;      
    float loopVoxelLeafSize_;           

    ros::Time timeLaserInoStamp;
    double timeLaserInfoCur;
    
    Eigen::MatrixXd poseCovariance;
    Eigen::Matrix<double, 6, 6> odomCovariance;
    std::deque<Eigen::Matrix<double, 6, 6>, 
    Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> keyFrameCovariances;

    PointCloudXYZRGB::Ptr laserCloudSurfLastDS;
    vector<PointCloudXYZRGB::Ptr> surfCloudKeyFrames;
    vector<double>gps_extrinT;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp_;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses_; 
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icpLoop_;
    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterICP_;

    string gps_topic;
    string imu_topic;
    string optimization_mode_;
    string opt_tum_output_path_;
    string gps_tum_output_path_;
    string global_map_pcd_path_;
    std::string pcd_save_directory_;

    bool gps_en;
    double gps_cov_threshold;
    std::deque<nav_msgs::Odometry> gpsQueue;
    std::deque<nav_msgs::Odometry> gpsQueue_B;
    GeographicLib::LocalCartesian gps_trans_;
    std::vector<nav_msgs::Odometry> rtk_trajectory_data_;
    std::map<int, gtsam::Point3> propagated_gps_;
    

    boost::shared_ptr<gtsam::PreintegrationParams> imu_params_gtsam_;
    gtsam::imuBias::ConstantBias initial_bias_guess_;
    gtsam::Pose3 T_imu_rtk;

    std::thread optimization_thread_;
    
private:

    VoxelMapConfig voxel_config_;
    std::mutex mutex;
    std::deque<imudata> imuQueue;
    std::deque<imudata> init_imuQueue;
    Eigen::Vector3d cov_gyro_;
    Eigen::Vector3d cov_acce_;
    Eigen::Vector3d init_bg_;
    Eigen::Vector3d init_ba_;
    Eigen::Vector3d gravity_;
    double gravity_norm_ = 9.81;
    std::unique_ptr<gtsam::ISAM2> isam;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
};