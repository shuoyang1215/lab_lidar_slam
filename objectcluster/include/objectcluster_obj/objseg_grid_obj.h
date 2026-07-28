#ifndef OBJSEG_GRID_OBJ_H
#define OBJSEG_GRID_OBJ_H
#include <fstream>
#include <sys/stat.h>
#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <nav_msgs/GridCells.h>
#include <chrono>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h> //滤波相关
#include <pcl/filters/random_sample.h> //随机降采样
#include <pcl/console/time.h> //pcl测时间函数
#include<pcl/filters/passthrough.h> //pcl直通滤波
#include <math.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
// #include <pcl/filters/morphological_filter.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include<pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/random_sample.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/angles.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt8MultiArray.h>
#include <ros/package.h>
#include "sort/Kalman.h"
#include "sort/Hungarian.h"
#include <algorithm>
#include<mutex>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cmath>

#include "imu_orientation_estimator/imu_subscriber.hpp"
#include "imu_orientation_estimator/Estimator.h"
#include <deque>
//#include "deque.hpp"
using namespace std;
using namespace Eigen;
using namespace pcl;

#define Debug

struct GridCloudNode {
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr noiseCloud;
    float cluster_flag;
    float ground_z_elevation;
    Eigen::Matrix3f covariance;                    //创建3×3协方差矩阵存储对象
    Eigen::Vector4f centeroid;
//    Eigen::VectorXf eigenvalues;
//    bool enable_obj;

//    float top_planarity;
//    float elevation_score;
//    float appear_score;
    float elev_diff_score;
//    float planarity_score;
//    float total_score;
    Eigen::MatrixXf normal;

    double ground_z_vec;
    int invalid_count;
    float base_z;
    pcl::PointXYZ base_pt;
    pcl::PointXYZ max_pt;
//    pcl::PointXYZ left_pt;
//    pcl::PointXYZ right_pt;

    float slopeflag;
    float slopeangle;
    float ishole;//0未知，1是空洞栅格,2是非空栅格
    bool is_small_hole;

    GridCloudNode() {
        Cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        noiseCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cluster_flag = 0;
        ground_z_elevation = 0;
        covariance.setZero();
        centeroid.setZero();
        slopeangle = 0;
        slopeflag = 0;
        normal.setZero();
        elev_diff_score = 0;
        ishole = 0;
        ground_z_vec = 0;
        base_z = 0;
        is_small_hole = false;
    }

    void ClearCloud() {
        Cloud->clear();
        noiseCloud->clear();
        cluster_flag = 0;
        ground_z_elevation = 0;
        covariance.setZero();
        centeroid.setZero();
        normal.setZero();
        slopeflag = 0;
        slopeangle = 0;

        elev_diff_score = 0;
        ishole = 0;

        ground_z_vec = 0;
        base_z = 0;
        base_pt.x=0;
        base_pt.y=0;
        base_pt.z=0;
        max_pt.x=0;
        max_pt.y=0;
        max_pt.z=0;

        is_small_hole = false;

    }
};

typedef GridCloudNode* GridCloudNodePtr;

struct Object {

    float label;
    float prob;
    float cx;
    float cy;
//    int gridcount;
};


class ObjSegGrid_obj {

public:
    ObjSegGrid_obj(ros::NodeHandle &nh_);

    // registered laser scan callback function
    // 输入点云回调函数
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2);

    void status_callback(const std_msgs::UInt8MultiArray &status_msg);

    // 深度图转换为点云
    void Deep2PointCloud(const sensor_msgs::ImageConstPtr &Depth_row_image);

    // 输入图像回调函数，输入深度图
    void CameraHandler(const sensor_msgs::ImageConstPtr &Depth_row_image);

    void SlopeLineHandler(const visualization_msgs::MarkerArrayPtr &LineArray);

    void SlopeCoffHandler(const geometry_msgs::PosePtr &Posemsg);

    void wheel_callback(const geometry_msgs::TwistStampedConstPtr &twist_msg);

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

//  void CameraHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2);
    // 将点云分布到栅格中
    void ProjectCloudToGrid();

    template<typename PointT>
    inline void extract_initial_seeds_(
            const pcl::PointCloud<PointT> &p_sorted,
            pcl::PointCloud<PointT> &init_seeds);

    template<typename PointT>
    inline void estimate_plane_(const pcl::PointCloud<PointT> &ground);

    // For adaptive
    template<typename PointT>
    // https://blog.csdn.net/qq_38167930/article/details/119165988
    // https://blog.csdn.net/qq_33287871/article/details/106183892
    inline void extract_piecewiseground(
            const pcl::PointCloud<PointT> &src,
            pcl::PointCloud<PointT> &dst);

    void sort_vec(const VectorXf &vec, VectorXf &sorted_vec);

    void ComputeTopCurvature(int indX, int indY);

    void ComputeScore(int indX, int indY);

    void ComputeScorev2(int indX, int indY);

    void ElevationScore(int indX, int indY);

    void AppearScore(int indX, int indY);

    void ElevDiffScore(int indX, int indY);

    void ElevDiffScorev2(int indX, int indY);

    void Update_appear_score();

    void RemoveOutlaier(pcl::PointCloud<pcl::PointXYZ>::Ptr &InputCloud);

    void RemoveOutlaier2(int indX,int indY);

    void RemoveGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr BoundCloud,
                      pcl::PointXYZ CenterPoint);

    void ComputeGridFeature2();
    void SlopeBFS();

    void EmptyHoleBFS();

    void SlopeCheck(int indX,int indY);

    void check_noise_postprocess();

    void check_noise_postprocessv2();

    void ComputeHole(int indX, int indY);
    void ComputeBigHole();

    void FindCoarseObj(int indX, int indY);

    void CheckObj(int indX, int indY);

    void ComputeSmallObj(int indX, int indY);

    void ClusterAndPubObjectGrid();

    void ClusterAndPubObjectGrid2();

    void SlopeCheckv2(int indX,int indY);

    void AddSurroundGrass();

    void EstimatePlane();

    void ComputeBoundingCropHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr BoundCloud, int flag_intensity);

    void PubBoundingBox();

    void PubBoundingBox_WORLD();

    void AddSmallHoleObjectCloud();

    void PubBoundingBox2(std::string BoundingBoxtype);

    void ComputeBoundingPt(const pcl::PointCloud<pcl::PointXYZ>::Ptr BoundCloud,
                           int flag_intensity);

    void PubFeatureVis();

    void SuperVoxel();

    void PublishCloud();

    void ClearAll();

    void cloudsegmetation();

    void EdgeDet();

    void DrivableArea();

    void ComputerFlatness();

    void SORT();

    // 运行函数
    void ObjSegGridRun();

    void SaveObj();

    void computeplaneuseSVD(const pcl::PointCloud<pcl::PointXYZ>::Ptr src,
                            Eigen::RowVector3d &normal,
                            float &d);

    void ComputeBigHolev2();

    void RemoveCloudUnderSlope();

    void RansacPlaneExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputcloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &outputcloud,
                            bool &is_slope);

    void RemoveSlopeWithSlopeLine();

    void RemoveCloudWithSlopeLineAndPlaneCoff();

    void UpdateSensorHigh();

    void ComputeEmptyHoleBound();

    void ComputeHoleTheory();

    void CalculateIMURot();

    void DrivableAreav2();
    // 变量的定义都在私有变量里
private:
    ros::NodeHandle nh;

    GridCloudNodePtr **GridCloudNodeMap;

    string camera_input_topic;
    string input_Lidar;
    string log_path;
    string imu_topic;

    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudCrop;
    pcl::PointCloud<pcl::PointXYZ>::Ptr LidarCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTemp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr total_ground_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ObjPointCloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr CameraDeepCloud;
    pcl::PointCloud<pcl::PointXYZ> ground_pc_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr XYBoundingCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr TempCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ObjCloudNoGround;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_on_slopecloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr slopecloud;


    sensor_msgs::PointCloud2 sensorCloudTemp;

    pcl::CropBox<pcl::PointXYZ> slope_box_filter_;
    pcl::CropBox<pcl::PointXYZ> nearby_box_filter_;
    pcl::CropBox<pcl::PointXYZ> lidar_range_box_filter_;

//    pcl::CropBox<pcl::PointXYZ> DeepCamera_bbox_filter_;

    ros::Publisher pub_MarkerArray_;
    ros::Publisher XYBoundingCloud_pub_;
    ros::Publisher XYBoundingMarkArray_pub_;
    ros::Publisher pub_MarkerArrayArea;
    ros::Publisher full_cloud_pub_;
    ros::Publisher ObjCloudNoGroundPub_;
    ros::Publisher ObjCloudGroundSeg;

    ros::Subscriber camera_sub_;
    ros::Subscriber slope_line_sub_;
    ros::Subscriber slope_coff_sub_;
    ros::Subscriber sub_wheel;
    ros::Subscriber localization_sub; // 订阅定位位姿
    ros::Subscriber subLaserCloud_;
    ros::Subscriber status_sub_;

    visualization_msgs::MarkerArray bounding_marker_array;
    visualization_msgs::MarkerArray bbox_marker_array;

    double dcx = 425.70074462890625;
    double dcy = 234.9811248779297;
    double dfx = 429.1876220703125;
    double dfy = 429.1876220703125;
    float kScaleFactor = 1000;
    float DeepCameraHigh = 0.25;
    float SkyPointHeight = 0.3;
    float noise_underground_thr;
    float minOutlierarea = 0.15 * 0.15;

    float DCameraColSampleRate = 6;
    float DCameraRowSampleRate = 4;
    float min_z_elevation = 0.1;
    float LidarHigh = 0.508;
    float DCameraPitch = 0.3;

    bool newlaserCloud = false;
    bool newCameraCloud = false;

    float lidarXAxis = 20;
    float lidarYAxis = 20;
    float lidarNearbyXAxis = 1;
    float lidarNearbyYAxis = 0.5;
    float lidarNearbyZAxis = 0.5;
    float DCameraXAxis = 5;
    float DCameraYAxis = 5;
    float DCameraZAxis = 1;
    float MinCarDis = 0.6;

    bool is_print_message_ = true;
    bool AutoStatus = false;
    float GridSize;
    int GridSizeInverse = float(1.0 / GridSize);
    int GridCloudWidth = 201;
    int GridCloudHalfWidth = (GridCloudWidth - 1) / 2;
    int minGridCloudNum = 25;

    Eigen::Matrix4d Pose_trans;

    float ground_height = -0.44;
    float NoDownSampleThr = 9;
    float DCameraRandSampleRate = 8;

    //deepcamera ->camera
//  Eigen::Matrix3d dcamera2lidar_R;
//  Eigen::Vector3d dcamera2lidar_T;

    //param for ground plane fit
    int num_lpr_ = 15;
    float th_seeds_ = 0.1;
    int num_iter_ = 3;
    Eigen::Matrix3f cov_;
    Eigen::Vector4f pc_mean_;
    Eigen::VectorXf singular_values_;
    Eigen::MatrixXf normal_;
    float th_dist_d_;
    float d_;
    float th_dist_ = 0.2;
    float uprightness_thr_ = 0.85; //垂直度为30度
    float elevation_thr_ = 0.4;
    float outlier_pc = 0.001;
    float total_score_thr;

    float grasshigh_mid;
    float grasshigh_3sigma;
    float grasshigh_real;
    float grasshigh_original;

    bool AutoDirection = false;
    bool UseDCamera = false;
    bool UseLidar = false;
    bool HighGrass = false;
    bool Grass = false;
    bool plane = false;
    bool isslope = false;
    bool run_alone = false;
    float SensorHigh;
    std_msgs::Header laser_Stamp;
    float nx;
    float ny;
    float nz;
    float d;

    vector<KalmanFilter> filters;
    float wheel_dx;
    std::map<float, pcl::PointCloud<pcl::PointXYZI>> map_objectcloud;
//    float max_label;
    float sort_min_distance_to_track;
    float sort_min_diff_high_to_track;
    float sort_min_disance_to_match;
    float max_gridcount2track;

    bool use_sort;
    float x_min_limit;
    float x_max_limit;
    float y_min_limit;
    float y_max_limit;
    bool hasline = false;
    int lineusecount = false;
    geometry_msgs::Pose planecoff;
    bool hasplanecoff;
    int planecoffusecount;
    float mindistoslope;
    int minframeusecount;
    float relative_diff_thr;
    mutex sloperemove_lock;
    float min_hole_depth;

    vector<pcl::ModelCoefficients::Ptr> planecoeffArray;
    int startindX;
    int indXInterval;
    int endindX;
    int startindY;
    int indYInterval;
    int endindY;
    int YAxisPlaneNum=1;
    int XAxisPlaneNum=1;
    float BigHoleMaxY = 3;
    int MaxPtPerGrid;
    int connect_distance;
    int additional_grid = 2;
    float clusterlabel;
    float emptyholestartlabel;
    float NearbyLidarfilter;
    map<int,set<pair<int, int>>> AllEmptyneighbour;

//    visualization_msgs::MarkerArray bbox_marker_array;
    float SensorXAxis,SensorYAxis;

    pcl::RandomSample<pcl::PointXYZ> rs;	//创建滤波器对象
    set<float> slopeclusterflag;
    set<float> smallholeintensity;

    bool CheckUpSlope = false;
    bool CheckDownSlope = false;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> CloudVector;
    vector<pcl::PointIndices> ece_inlier;
    search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;

#ifdef Debug
    int ground_filter_rate = 3;
    int point_filter_rate = 1;

    ros::Publisher ObjPointCloudPub_;
    ros::Publisher pub_MarkerArrayGrid_;
    ros::Publisher pub_MarkerArrayFeature_;
    ros::Publisher UnderGroundCloudPub_;
    ros::Publisher pub_grassplane_;
    ros::Publisher Tempviscloud_;
    ros::Publisher pub_noisecloud_;
    ros::Publisher pub_mismatch_objectcloud_;
    ros::Publisher ransacslopecloud_pub_;
    ros::Publisher revertnoisecloud_pub_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr mismatchObjectCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr UnderGroundCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr revertnoisecloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud;
#endif

    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::deque<IMUData> imu_data_buff;
    double gyro_noise = 1e-6;
    double gyro_bias_noise = 1e-8;
    double acc_noise = 1e-6;
    std::shared_ptr<OriEst::Estimator> orientation_estimator;

    Eigen::Matrix4d point_transform;


};

#endif
