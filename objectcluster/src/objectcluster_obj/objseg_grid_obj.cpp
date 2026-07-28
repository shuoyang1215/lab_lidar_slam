#include "objectcluster_obj/objseg_grid_obj.h"

#define Random(x) (rand() % x)


using namespace std;
using namespace Eigen;
using namespace pcl;


ObjSegGrid_obj::ObjSegGrid_obj(ros::NodeHandle &nh_) : nh(nh_) {

    // 计算深度相机坐标系到雷达坐标系的变换
/*
    dcamera2lidar_R << 1, 0, 0, 0, -1, 0, 0, 0, 1;
    dcamera2lidar_T << 0, 0, 0;
    vector<double> l2dc_R, l2dc_T;
    // 从节点中读取变换矩阵参数
    nh.param<vector<double>>("/objseg_gridclass/dcamera2lidar_T", l2dc_T, l2dc_T);
    nh.param<vector<double>>("/objseg_gridclass/dcamera2lidar_R", l2dc_R, l2dc_R);
    // 变换矩阵赋值
    if (!l2dc_R.empty())
    {
      dcamera2lidar_R(0, 0) = l2dc_R[0];
      dcamera2lidar_R(0, 1) = l2dc_R[1];
      dcamera2lidar_R(0, 2) = l2dc_R[2];
      dcamera2lidar_R(1, 0) = l2dc_R[3];
      dcamera2lidar_R(1, 1) = l2dc_R[4];
      dcamera2lidar_R(1, 2) = l2dc_R[5];
      dcamera2lidar_R(2, 0) = l2dc_R[6];
      dcamera2lidar_R(2, 1) = l2dc_R[7];
      dcamera2lidar_R(2, 2) = l2dc_R[8];
    }
    if (!l2dc_T.empty())
    {
      dcamera2lidar_T(0) = l2dc_T[0];
      dcamera2lidar_T(1) = l2dc_T[1];
      dcamera2lidar_T(2) = l2dc_T[2];
    }
*/  
     nh.param<bool>("/objseg_gridclass/is_print_message_", is_print_message_, false);
    nh.param<bool>("/objseg_gridclass/AutoStatus", AutoStatus, false);
    // 读取输入点云topic
    nh.param<string>("/objseg_gridclass/input_Lidar", input_Lidar, "/zvision_lidar_points");
    // nh.param<string>("/objseg_gridclass/input_Lidar", input_Lidar, "/undistort_cloud");

    // 读取输入图像topic
//    nh.param<string>("/objseg_gridclass/camera_input_topic", camera_input_topic, "/camera/depth/image_rect_raw");
    nh.param<string>("/objseg_gridclass/camera_input_topic", camera_input_topic, "/camera/depth/color/points");

    // 读取栅格大小
    nh.param<float>("/objseg_gridclass/GridSize", GridSize, 0.25);
    // 读取总的点云格子一边数量
    nh.param<int>("/objseg_gridclass/GridCloudWidth", GridCloudWidth, 101);
    // 激光雷达安装高度
    nh.param<float>("/objseg_gridclass/LidarHigh", LidarHigh, 0.508);
    // 栅格内最少的点云数量
    nh.param<int>("/objseg_gridclass/minGridCloudNum", minGridCloudNum, 10);
    // 高度阈值
    nh.param<float>("/objseg_gridclass/elevation_thr", elevation_thr_, 0.4);
    // 角度阈值
    nh.param<float>("/objseg_gridclass/uprightness_thr", uprightness_thr_, 0.85);
    // low point representative 低的点数？？？
    nh.param<int>("/objseg_gridclass/num_lpr", num_lpr_, 15);
    // 距离？？？
    nh.param<float>("/objseg_gridclass/th_dist", th_dist_, 0.1);
    // 迭代次数
    nh.param<int>("/objseg_gridclass/num_iter", num_iter_, 5);
    // 种子数
    nh.param<float>("/objseg_gridclass/th_seeds", th_seeds_, 0.1);
    //小于min_z_elevation高度直接判定为地面
    nh.param<float>("/objseg_gridclass/min_z_elevation", min_z_elevation, 0.1);

    // 相机内参标定值
    nh.param<double>("/objseg_gridclass/dcx", dcx, 425.7007446289062);
    nh.param<double>("/objseg_gridclass/dcy", dcy, 234.9811248779297);
    nh.param<double>("/objseg_gridclass/dfx", dfx, 429.1876220703125);
    nh.param<double>("/objseg_gridclass/dfy", dfy, 429.1876220703125);
    nh.param<float>("/objseg_gridclass/kScaleFactor", kScaleFactor, 1000.0);
    nh.param<float>("/objseg_gridclass/DeepCameraHigh", DeepCameraHigh, 0.25);
    nh.param<float>("/objseg_gridclass/SkyPointHeight", SkyPointHeight, 0.3);
    nh.param<float>("/objseg_gridclass/minOutlierarea", minOutlierarea, 0.15*0.15);


    nh.param<float>("/objseg_gridclass/DCameraPitch", DCameraPitch, 0);
    nh.param<float>("/objseg_gridclass/outlier_pc", outlier_pc, 0.01);

    nh.param<float>("/objseg_gridclass/DCameraColSampleRate", DCameraColSampleRate, 6);
    nh.param<float>("/objseg_gridclass/DCameraRowSampleRate", DCameraRowSampleRate, 4);

    // 地面位置，z=-0.35
    nh.param<float>("/objseg_gridclass/ground_height", ground_height, -0.4);

    // 激光雷达使用范围
    nh.param<float>("/objseg_gridclass/lidarXAxis", lidarXAxis, 10);
    nh.param<float>("/objseg_gridclass/lidarYAxis", lidarYAxis, 10);
    // 激光雷达最近处盲区
    nh.param<float>("/objseg_gridclass/lidarNearbyXAxis", lidarNearbyXAxis, 1);
    nh.param<float>("/objseg_gridclass/lidarNearbyYAxis", lidarNearbyYAxis, 0.5);
    nh.param<float>("/objseg_gridclass/lidarNearbyZAxis", lidarNearbyZAxis, 0.5);
    // 相机视野范围
    nh.param<float>("/objseg_gridclass/DCameraXAxis", DCameraXAxis, 4);
    nh.param<float>("/objseg_gridclass/DCameraYAxis", DCameraYAxis, 4);
    nh.param<float>("/objseg_gridclass/DCameraZAxis", DCameraZAxis, 2);
    nh.param<float>("/objseg_gridclass/NoDownSampleThr", NoDownSampleThr, 3 * 3);
    nh.param<float>("/objseg_gridclass/MinCarDis", MinCarDis, 0.6);
    if (is_print_message_) cout<<"MinCarDis: "<<MinCarDis<<endl;
    nh.param<float>("/objseg_gridclass/DCameraRandSampleRate", DCameraRandSampleRate, 8);
    nh.param<float>("/objseg_gridclass/total_score_thr", total_score_thr, 1.5);

    nh.param<bool>("/objseg_gridclass/AutoDirection", AutoDirection, true);
    nh.param<bool>("/objseg_gridclass/UseLidar", UseLidar, true);//使用雷达或者深度相机检测
    nh.param<bool>("/objseg_gridclass/UseDCamera", UseDCamera, false);
    nh.param<bool>("/objseg_gridclass/HighGrass", HighGrass, false);
    nh.param<bool>("/objseg_gridclass/Grass", Grass, false);
    nh.param<bool>("/objseg_gridclass/plane", plane, false);
    nh.param<bool>("/objseg_gridclass/isslope", isslope, false);
    nh.param<bool>("/objseg_gridclass/run_alone", run_alone, false);

    nh.param<float>("/objseg_gridclass/sort_min_distance_to_track", sort_min_distance_to_track, 3);
    nh.param<float>("/objseg_gridclass/sort_min_diff_high_to_track", sort_min_diff_high_to_track, 0.4);
    nh.param<float>("/objseg_gridclass/sort_min_disance_to_match", sort_min_disance_to_match, 0.5);
    nh.param<float>("/objseg_gridclass/max_gridcount2track", max_gridcount2track, 5*5);



    nh.param<bool>("/objseg_gridclass/use_sort", use_sort, true);
    nh.param<float>("/objseg_gridclass/mindistoslope", mindistoslope, 0.08);
    nh.param<int>("/objseg_gridclass/minframeusecount", minframeusecount, 5);

    nh.param<float>("/objseg_gridclass/relative_diff_thr", relative_diff_thr, 0.16);
    nh.param<float>("/objseg_gridclass/min_hole_depth", min_hole_depth, 0.07);

    nh.param<int>("/objseg_gridclass/YAxisPlaneNum", YAxisPlaneNum, 2);
    nh.param<int>("/objseg_gridclass/XAxisPlaneNum", XAxisPlaneNum, 3);
    nh.param<float>("/objseg_gridclass/BigHoleMaxY", BigHoleMaxY, 3.0);

    nh.param<int>("/objseg_gridclass/MaxPtPerGrid", MaxPtPerGrid, 50);

    nh.param<int>("/objseg_gridclass/connect_distance", connect_distance, 121);

    nh.param<float>("/objseg_gridclass/noise_underground_thr", noise_underground_thr, 0.05);

    nh.param<int>("/objseg_gridclass/additional_grid", additional_grid, 2);
    nh.param<bool>("/objseg_gridclass/CheckUpSlope", CheckUpSlope, false);
    nh.param<bool>("/objseg_gridclass/CheckDownSlope", CheckDownSlope, false);

    nh.param<float>("/objseg_gridclass/NearbyLidarfilter", NearbyLidarfilter, 0.3);

    nh.param<string>("/objseg_gridclass/imu_topic", imu_topic, "/livox/imu");
    cout<<"imu_topic: "<<imu_topic<<endl;


    nh.param<string>("/objseg_gridclass/log_path", log_path,"/log/mlog.log");
    // log_path = ros::package::getPath("objectcluster")+log_path;
    size_t last_slash_idx = log_path.rfind('/');
    std::string directory_path = log_path.substr(0, last_slash_idx);
    // 判断目录是否存在，不存在则创建
    struct stat sb;
    if (stat(directory_path.c_str(), &sb) != 0 || !S_ISDIR(sb.st_mode))
    {
        if (mkdir(directory_path.c_str(), 0777) == -1)
        {
            std::cerr << "Error: Failed to create directory." << std::endl;
        }
    }

#ifdef Debug
    // 不均匀采样率
    nh.param<int>("/objseg_gridclass/ground_filter_rate", ground_filter_rate, 3);
    nh.param<int>("/objseg_gridclass/point_filter_rate", point_filter_rate, 1);

#endif

    if (is_print_message_) {
        std::cout << "AutoDirection" << AutoDirection << endl;
        std::cout << "UseLidar" << UseLidar << endl;
        std::cout << "UseDCamera" << UseDCamera << endl;
        std::cout << "HighGrass" << HighGrass << endl;
        std::cout << "Grass" << Grass << endl;
        std::cout << "plane" << plane << endl;
        std::cout << "run_alone" << run_alone << endl;
    }

    // 创立新点云
    laserCloudCrop.reset(new pcl::PointCloud<pcl::PointXYZ>());
    LidarCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    laserCloudTemp_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    ground_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    total_ground_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    ObjPointCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    CameraDeepCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    XYBoundingCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    TempCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
//    EdgeCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    ObjCloudNoGround.reset(new pcl::PointCloud<pcl::PointXYZI>());
    obj_on_slopecloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    slopecloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    CloudVector.resize(XAxisPlaneNum*YAxisPlaneNum);
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr &element : CloudVector){
        element.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    tree.reset(new search::KdTree<pcl::PointXYZ>);

#ifdef Debug
    UnderGroundCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    mismatchObjectCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    revertnoisecloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    noise_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());


#endif

    if (UseLidar) {
        SensorHigh = LidarHigh;
    } else if (UseDCamera) {
        SensorHigh = DeepCameraHigh;
//        minGridCloudNum = minGridCloudNum - 5;
    }

    // 只保留xy方向正负20m，z方向2m以下的点云 只保留地面及以下点
    slope_box_filter_.setMin(Eigen::Vector4f(-1.5, 0, -SensorHigh - 2, 1.0));
    slope_box_filter_.setMax(Eigen::Vector4f(1.5, 2, 0.5, 1.0));

//    boundary_box_filter_.setMin(Eigen::Vector4f(-10, -10, -10, 1.0));
//    boundary_box_filter_.setMax(Eigen::Vector4f(10, 10, 10, 1.0));

//    boundary_box_filter_.setNegative(false); //保留框内的

    // 保留点云投影到图像视野范围内的点
//    DeepCamera_bbox_filter_.setMin(Eigen::Vector4f(-DCameraXAxis, -DCameraYAxis, -DeepCameraHigh - 0.5, 1.0));
//    DeepCamera_bbox_filter_.setMax(Eigen::Vector4f(DCameraXAxis, DCameraYAxis, DCameraZAxis, 1.0));
//    DeepCamera_bbox_filter_.setMin(Eigen::Vector4f(-10, -10, -10 - 0.5, 1.0));
//    DeepCamera_bbox_filter_.setMax(Eigen::Vector4f(10, 10, 10, 1.0));
//    DeepCamera_bbox_filter_.setNegative(false); //保留框内的

    //zvision滤去没有接收到的点，没有接收到的点都是0，0，0，
    // 去除左右细长条
    //修改这里可以去除雷达后方点云
    lidar_range_box_filter_.setMin(Eigen::Vector4f(-lidarXAxis, -lidarYAxis, -SensorHigh -10, 1.0));
    lidar_range_box_filter_.setMax(Eigen::Vector4f(lidarXAxis, lidarYAxis, 1.5, 1.0)); // 10
//    lidar_range_box_filter_.setNegative(true); //保留框外的
    lidar_range_box_filter_.setNegative(false);

    //去掉轮腿机器人的中心
    nearby_box_filter_.setMin(Eigen::Vector4f(-NearbyLidarfilter, -NearbyLidarfilter, -NearbyLidarfilter, 1.0));
    nearby_box_filter_.setMax(Eigen::Vector4f(NearbyLidarfilter, NearbyLidarfilter, NearbyLidarfilter, 1.0));
    nearby_box_filter_.setNegative(true); //保留框外的



    // 获取点云一半的.一边151个格子,即左边75个格子,右边75个格子
    GridCloudHalfWidth = (GridCloudWidth - 1) / 2;
    // 格子宽度的导数
    GridSizeInverse = 1 / GridSize;

    status_sub_ = nh.subscribe("/Usb_Ai", 1, &ObjSegGrid_obj::status_callback, this);

    // 订阅输入点云,运行laserCloudHandler函数，对点云进行初筛，存入全局变量中
    subLaserCloud_ = nh.subscribe<sensor_msgs::PointCloud2>(input_Lidar, 1, &ObjSegGrid_obj::laserCloudHandler, this);

// 订阅图像，运行CameraHandler函数，输入深度图,转换为点云
    camera_sub_ = nh.subscribe(camera_input_topic, 1, &ObjSegGrid_obj::CameraHandler, this);
//        camera_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(camera_input_topic, 1, &ObjSegGrid_obj::CameraHandler,this);
    slope_line_sub_ = nh.subscribe("/slopeSegDCamera/line_markarray", 1, &ObjSegGrid_obj::SlopeLineHandler, this);
    slope_coff_sub_ = nh.subscribe("/slopeSegDCamera/plane_coff", 1, &ObjSegGrid_obj::SlopeCoffHandler, this);
    sub_wheel = nh.subscribe("/Ser_odom", 1, &ObjSegGrid_obj::wheel_callback, this);

    // 订阅定位话题，获得当前位姿
    localization_sub = nh.subscribe("/Odometry", 10, &ObjSegGrid_obj::odometryCallback, this);

    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, imu_topic, 100);
    imu_sub_ptr_->StartSubscribe();

    // 发布变量
    ObjCloudNoGroundPub_ = nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/ObjCloudNoGround", 1);
    pub_MarkerArray_ = nh.advertise<visualization_msgs::MarkerArray>("/objseg_gridclass/LidarObjBox", 1);
    XYBoundingCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/XYBoundingCloud", 1);
    XYBoundingMarkArray_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/objseg_gridclass/BoundingMarkArray", 1);
    full_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/fullPointCloud", 1);
    pub_MarkerArrayArea = nh.advertise<visualization_msgs::MarkerArray>("/objseg_gridclass/DrivableArea", 1);
    ObjCloudGroundSeg = nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/GroundCloud", 1);

#ifdef Debug
    ObjPointCloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/ObjPointCloud", 1);
    UnderGroundCloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/UnderGroundCloud", 1);
    Tempviscloud_ = nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/Tempviscloud", 1);
    pub_MarkerArrayGrid_ = nh.advertise<visualization_msgs::MarkerArray>("/objseg_gridclass/MarkerArrayGrid", 1);
    pub_MarkerArrayFeature_ = nh.advertise<visualization_msgs::MarkerArray>("/objseg_gridclass/MarkerArrayFeature", 1);
    pub_grassplane_ = nh.advertise<visualization_msgs::MarkerArray>("/objseg_gridclass/grassplane", 1);
    pub_noisecloud_ = nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/noisecloud", 1);
    pub_mismatch_objectcloud_ = nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/mismatch_objectcloud", 1);
    revertnoisecloud_pub_ =  nh.advertise<sensor_msgs::PointCloud2>("/objseg_gridclass/revernoisecloud", 1);
#endif

    // 创建一个数组,作为栅格地图.GridCloudWidth个位置
    GridCloudNodeMap = new GridCloudNodePtr *[GridCloudWidth];
    // 每一个数组，在创建一个GridCloudWidth个位置的数组
    for (int i = 0; i < GridCloudWidth; i++) {
        GridCloudNodeMap[i] = new GridCloudNodePtr[GridCloudWidth];
        for (int j = 0; j < GridCloudWidth; j++) {
            // 二维数组内每一个点，初始一个栅格对象，里面包含点云、是否聚类等信息，是结构体
            GridCloudNodeMap[i][j] = new GridCloudNode();
        }
    }

    hasline = false;
    lineusecount = false;



    rs.setSample(MaxPtPerGrid);					//设置下采样点云的点数

    if (UseLidar) {
        SensorHigh = LidarHigh;
        SensorXAxis = lidarXAxis;
        SensorYAxis = lidarYAxis;

    } else if (UseDCamera) {
        SensorHigh = DeepCameraHigh;
        SensorXAxis = DCameraXAxis;
        SensorYAxis = DCameraYAxis;
    }


    orientation_estimator = std::make_shared<OriEst::Estimator>(gyro_noise, gyro_bias_noise, acc_noise);
    imu_data_buff.clear();

    // 初始化完成
    std::cerr << "Init objSeg" << endl;
}

void ObjSegGrid_obj::RemoveOutlaier2(int indX,int indY) {

    // remove outlier in the sky
    auto &InputCloud = GridCloudNodeMap[indX][indY]->Cloud;
    bool top_outlier = false,bottom_outlier=false;
    int OutlierCloudNum = 0;
//    int outliar_in_sky_num = 0;
    int idx;

    if(isslope){
//        noise_underground_thr = 0.1;
        SkyPointHeight = 0.05;
        noise_underground_thr = 0.05;
    }
    else{
//        noise_underground_thr = 0.1;
        SkyPointHeight = 0.1;
    }


    //remove outlier underground
    auto it1 = InputCloud->points.begin();
    int cloudsize = InputCloud->points.size();
    int temp_idx=0;

    //首先要求最低点低于地平面，否则不计算地面下异常点
    if(InputCloud->points[0].z < -SensorHigh){

        for (idx = 0; idx < cloudsize - 1; idx++)
        {


            if (InputCloud->points[idx+1].z - InputCloud->points[idx].z > noise_underground_thr){
//            if (InputCloud->points[idx+1].z - InputCloud->points[idx].z > 0.03){


                bottom_outlier = true;
                while(temp_idx <= idx){
                    it1++;
                    temp_idx++;
                }


                //地下异常点直接接上天空异常点，没有点在地面上
                if( InputCloud->points[idx+1].z > -SensorHigh+0.1){
                    break;
                }


//                it1++;
//                break;
            }

            if(InputCloud->points[idx+1].z > - SensorHigh)
                break;


//            it1++;
        }
    }


//    if(InputCloud->points.size() == 1){
//        if(InputCloud->points[0].z < -SensorHigh - 0.1){
//            bottom_outlier = true;
//            it1++;
//        }
//    }
//    if(indX == 86 && indY ==115){
//        cout<<"----------------------------------"<<endl;
//
//        for(int i =0 ;i <InputCloud->points.size();i++ ){
//            cout<<"xyz :"<<InputCloud->points[i].x<<"  "<<InputCloud->points[i].y<<"  "<<InputCloud->points[i].z<<endl;
//        }
//        cout<<"bottom_outlier:"<<bottom_outlier<<"idx: "<<idx<<endl;
//        cout<<"InputCloud->pointssize:"<<InputCloud->points.size()<<endl;
//    }

    if(bottom_outlier && (temp_idx < 30 || isslope)){
        for (int i = 0; i < temp_idx; i++){
            //可视化
//            noise_cloud->points.emplace_back(InputCloud->points[i]);
            OutlierCloudNum++;

        }
        InputCloud->erase(InputCloud->points.begin(),it1);

    }
    else if(InputCloud->points.size() <=5 && isslope && ~bottom_outlier){
            //栅格间的连续性判断

//        if(InputCloud->points.size() <=2){
//
//            for (int i = 0; i < InputCloud->points.size(); i++){
//                //可视化
//                noise_cloud->points.emplace_back(InputCloud->points[i]);
//                OutlierCloudNum++;
//
//            }
//            InputCloud->clear();
//
//        }

//        else{
//            bool z_bottom_continue=false;
//            bool z_top_continue=false;
//            int invalid_count=0;
//            for (int dx = -1; dx <= 1; dx++) {
//                for (int dy = -1; dy <= 1; dy++) {
//                    // 如果这个栅格不等于1，即等于0或者2，即没有障碍物或研究遍历过。
//                    // 或者就是当前这个栅格
//                    // 或者待选的栅格过界
//                    // 则跳过这个栅格
//                    if ((dx == 0 && dy == 0) ||
//                        indX + dx < 0 || indX + dx > GridCloudWidth - 1 || indY + dy < 0 ||
//                        indY + dy > GridCloudWidth - 1)
//                        continue;
//                    if(GridCloudNodeMap[indX+dx][indY+dy]->Cloud->points.size() == 0){
//                        invalid_count++;
//                        continue;
//                    }
//
//                    pcl::PointXYZ &indxy_bottom_pt = GridCloudNodeMap[indX][indY]->base_pt;
//                    pcl::PointXYZ &indxy_max_pt = GridCloudNodeMap[indX][indY]->max_pt;
//                    pcl::PointXYZ &inddxdy_bottom_pt = GridCloudNodeMap[indX+dx][indY+dy]->base_pt;
//                    pcl::PointXYZ &inddxdy_max_pt = GridCloudNodeMap[indX+dx][indY+dy]->max_pt;
//
//                    if(UseDCamera){
//                        if(dy < 0 ){
//                            if(abs(GridCloudNodeMap[indX][indY]->max_pt.z - GridCloudNodeMap[indX+dx][indY+dy]->base_pt.z) < 0.1){
//                                z_top_continue = true;
//                            }else{
//                                invalid_count++;
//                            }
//                        }else if(dy > 0){
//                            if(abs(GridCloudNodeMap[indX][indY]->base_pt.z - GridCloudNodeMap[indX+dx][indY+dy]->max_pt.z) < 0.1){
//                                z_bottom_continue = true;
//                            }else{
//                                invalid_count++;
//                            }
//                        }else{
//
//                        }
//                    }else if(UseLidar){
//                        if(dy > 0 ){
//                            if(abs(GridCloudNodeMap[indX][indY]->max_pt.z - GridCloudNodeMap[indX+dx][indY+dy]->base_pt.z) < 0.1){
//                                z_top_continue = true;
//                            }else{
//                                invalid_count++;
//                            }
//                        }else if(dy < 0){
//                            if(abs(GridCloudNodeMap[indX][indY]->base_pt.z - GridCloudNodeMap[indX+dx][indY+dy]->max_pt.z) < 0.1){
//                                z_bottom_continue = true;
//                            }else{
//                                invalid_count++;
//                            }
//                        }else{
//                            float dis =
//                        }
//                    }
//
//
////                    if(abs(GridCloudNodeMap[indX][indY]->base_pt.z - GridCloudNodeMap[indX+dx][indY+dy]->max_pt.z) < noise_underground_thr){
////                        z_bottom_continue = true;
////                    }else if(abs(GridCloudNodeMap[indX][indY]->max_pt.z - GridCloudNodeMap[indX+dx][indY+dy]->base_pt.z) < noise_underground_thr){
////                        z_top_continue = true;
////                    }else{
////                        invalid_count++;
////                    }
//
//                }
//            }
//            if((z_bottom_continue== false && z_bottom_continue== false) || invalid_count >= 6){
//                for (int i = 0; i < InputCloud->points.size(); i++){
//                    //可视化
//                    noise_cloud->points.emplace_back(InputCloud->points[i]);
//                    OutlierCloudNum++;
//
//                }
//                InputCloud->clear();
//
//            }
    }



    //remove outlier in sky
    auto it2 = InputCloud->points.end();
    cloudsize = InputCloud->points.size();
    temp_idx=cloudsize-1;
//    cout<<"grasshigh_mid: "<<grasshigh_mid<<endl;
    //fixme 如果栅格中只有1个点或2个点，并且这个点是异常点 1.bag -s 101.4 2023.02.24
    //这个for循环不处理点云中最后一个点
    for (idx = cloudsize - 1; idx > 0;idx--){
            //地一个判断用于避免上斜坡，远处斜坡只有1个点且高于草面的问题


//        if(InputCloud->points[idx].z < 0)
//            break;

            if (InputCloud->points[idx].z - InputCloud->points[idx - 1].z > SkyPointHeight){
                top_outlier = true;
                while(temp_idx >= idx){
                    it2--;
                    temp_idx--;
                }
//            cout<<"indx: "<<indX<<"indy: "<<indY<<"temp_idx: "<<temp_idx<<" idx :"<<idx<<endl;

//            outliar_in_sky_num++;
//            break;
            }

            if(InputCloud->points[idx].z < -SensorHigh - 0.1)
                break;

//        it2--;
//        outliar_in_sky_num++;
        }



//    if(it2->z > -SensorHigh + min_z_elevation){

    //上斜坡的时候远处的点在雷达之上，且只有一个，也会被剔除，但是问题不大。
    //TODO 这里的-0.1改成与周围一圈地面点计算
    //indY < GridCloudHalfWidth + 4*GridSizeInverse 表示上坡的时候远处坡上的点会被当成天上的点
    //这个判断的情况，1:搜索到最后一个了，并且还不是top_outlier，最后一个点没有检查，2:栅格中只有一个点
//    if(idx == 0  && indY < GridCloudHalfWidth + 4*GridSizeInverse && top_outlier == false){
//        float min_diff_neighbor_high = 99;
//
//    }


//    if(indX == 86 && indY ==115){
//        for(int i =0 ;i <InputCloud->points.size();i++ ){
//            cout<<"xyz :"<<InputCloud->points[i].x<<"  "<<InputCloud->points[i].y<<"  "<<InputCloud->points[i].z<<endl;
//        }
//        cout<<"top_outlier:"<<bottom_outlier<<"idx: "<<idx<<endl;
//        cout<<"InputCloud->pointssize:"<<InputCloud->points.size()<<endl;
//    }

    if(idx == 0 && InputCloud->points.size()>2 && InputCloud->points[idx].z > -0.1 && indY < GridCloudHalfWidth + 4*GridSizeInverse){
        //如果没有点打在地面上，都直接打在天上
        top_outlier = true;
        while(temp_idx >= idx){
            it2--;
            temp_idx--;
        }
//        if(top_outlier ==true){
//            it2--;
//        }
    }
    else if (idx == 0 && InputCloud->points[idx].z > -SensorHigh + 0.2 && InputCloud->points.size() <= 2 &&
               indY < GridCloudHalfWidth + 4 * GridSizeInverse) {
        //栅格内只有1,2个点并且还在天上
        top_outlier = true;
        while(temp_idx >= idx){
            it2--;
            temp_idx--;
        }
//        if(top_outlier ==true){
//            it2--;
//        }
    }

//    if(top_outlier ==true){
//        it2--;
//    }

    TempCloud->clear();
    pcl::PointXYZ min; //xyz的最小值
    pcl::PointXYZ max; //xyz的最大值

    if(top_outlier && ((cloudsize-1) - temp_idx) <= 30){
//        cout<<"in_15"<<endl;

        for (int i = cloudsize - 1; i > temp_idx; i--){

#ifdef Debug
            noise_cloud->points.emplace_back(InputCloud->points[i]);
#endif
            GridCloudNodeMap[indX][indY]->noiseCloud->points.emplace_back(InputCloud->points[i]);
            OutlierCloudNum++;
        }
//        cout<<"in_15 indX:"<<indX<<" indY: "<<indY<<"OutlierCloudNum"<<OutlierCloudNum<<endl;
//        if(cloudsize == 1){
//            cout<<"before in_15 indX:"<<indX<<" indY: "<<indY<<"CloudNum"<<InputCloud->points.size()<<endl;
//
//            InputCloud->erase(InputCloud->points.begin());
//
//            cout<<"before in_15 indX:"<<indX<<" indY: "<<indY<<"CloudNum"<<InputCloud->points.size()<<endl;
//            cout<<"before in_15 indX:"<<indX<<" indY: "<<indY<<"GridCloudNum"<<GridCloudNodeMap[indX][indY]->Cloud->points.size()<<endl;
//
//
//        }else{
//            InputCloud->erase(--it2,InputCloud->points.end());
        InputCloud->erase(it2,InputCloud->points.end());

//        cout<<"before in_15 indX:"<<indX<<" indY: "<<indY<<"GridCloudNum"<<GridCloudNodeMap[indX][indY]->Cloud->points.size()<<endl;

//        }

    }


}



void ObjSegGrid_obj::status_callback(const std_msgs::UInt8MultiArray &status_msg) {
    if (AutoDirection) return; // 如果使用轮速计，则就不订阅消息
    if (!AutoStatus) return;
    // 第一位预留，用于复位
    // 第二位表示上下车状态，0-非上下车，1-上车，2-下车
    // 第三位表示前视后视，0-前视，1-后视
    // 第四位表示是否在工作区（即是否在草地里)，0表示不在草地里，1表示在草地里
    int i1 = stoi(to_string(status_msg.data[0]));
    int i2 = stoi(to_string(status_msg.data[1]));
    int i3 = stoi(to_string(status_msg.data[2]));
    int i4 = stoi(to_string(status_msg.data[3]));
    if (is_print_message_) cout << "obj i1:" << i1 << " i2:" << i2 << " i3:" << i3 << " i4:" << i4 << endl;

    if (i4 == 0) {
        plane = true;
        Grass = false;
    } else if (i4 == 1) {
        plane = false;
        Grass = true;
    } else {
        plane = false;
        Grass = true;
    }

    if (i2 == 0) {
        UseLidar = true;
        UseDCamera = false;
        // HighGrass = false;
        // Grass = true;
        // plane = false;
        isslope = false;
    } else if (i2 == 1) {
        UseLidar = true;
        UseDCamera = false;
        // HighGrass = false;
        Grass = true;
        plane = false;
        isslope = true;
    } else if (i2 == 2) {
        UseLidar = false;
        UseDCamera = true;
        // HighGrass = false;
        Grass = true;
        plane = false;
        isslope = true;
    } else {
        UseLidar = true;
        UseDCamera = false;
        // HighGrass = false;
        // Grass = true;
        // plane = false;
        isslope = false;
    }

    if (i3 == 0) {
        UseLidar = true;
        UseDCamera = false;
    } else if (i3 == 1) {
        UseLidar = false;
        UseDCamera = true;
    } else {
        UseLidar = true;
        UseDCamera = false;
    }

    if (UseLidar) {
        SensorHigh = LidarHigh;
        SensorXAxis = lidarXAxis;
        SensorYAxis = lidarYAxis;

    } else if (UseDCamera) {
        SensorHigh = DeepCameraHigh;
        SensorXAxis = DCameraXAxis;
        SensorYAxis = DCameraYAxis;
    }

}

void ObjSegGrid_obj::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2) {


    LidarCloud->clear();
    if (!UseLidar) return;
    // 从ros中读入点云到变量里
    pcl::fromROSMsg(*laserCloud2, *LidarCloud);
    laser_Stamp.stamp = laserCloud2->header.stamp;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*LidarCloud, *LidarCloud, indices);


//    for (int i = 0; i < LidarCloud->points.size(); i++) {
//        LidarCloud->points[i].y = -LidarCloud->points[i].y;
//        LidarCloud->points[i].x = -LidarCloud->points[i].x;
//
//    }


    lidar_range_box_filter_.setInputCloud(LidarCloud);
    lidar_range_box_filter_.filter(*LidarCloud);

    nearby_box_filter_.setInputCloud(LidarCloud);
    nearby_box_filter_.filter(*LidarCloud);

//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_near(new pcl::PointCloud<pcl::PointXYZ>);
//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_far(new pcl::PointCloud<pcl::PointXYZ>);
//
//
//      boundary_box_filter_.setMin(Eigen::Vector4f(-lidarXAxis, MinCarDis, -SensorHigh+0.1, 1.0));
//      boundary_box_filter_.setMax(Eigen::Vector4f(lidarXAxis, lidarYAxis, -SensorHigh+elevation_thr_, 1.0));
//      boundary_box_filter_.setNegative(false); //保留框内的
//      boundary_box_filter_.setInputCloud(laserCloudCrop);
//      boundary_box_filter_.filter(*cloud_filter_near);
//      cout<<"cloud_filter_near size:"<<cloud_filter_near->size()<<endl;
//
//      boundary_box_filter_.setNegative(true); //保留框内的
//      boundary_box_filter_.filter(*cloud_filter_far);
//      std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
//      pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;

//      pcl::PassThrough<pcl::PointXYZ> passthrough;
//      passthrough.setInputCloud(laserCloudCrop);//输入点云
//      passthrough.setFilterFieldName("y");//对z轴进行操作
//      passthrough.setFilterLimits(0, 3);//设置直通滤波器操作范围
//      passthrough.filter(*cloud_filter_near);//

//      if(cloud_filter_near->size() > 0){
//          //要求点云连续
//          sor.setInputCloud(cloud_filter_near);
//          sor.setRadiusSearch(0.05);
//          sor.setMinNeighborsInRadius(3);
//          sor.setNegative(false);
//          sor.filter(*cloud_filter_near);
//      }



//      std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
//      laserCloudCrop->clear();
//      *laserCloudCrop += *cloud_filter_far;
//      *laserCloudCrop += *cloud_filter_near;


    newlaserCloud = true;

}

// 轮速计消息回调函数
void ObjSegGrid_obj::wheel_callback(const geometry_msgs::TwistStampedConstPtr &twist_msg) {
    // double dx = twist_msg->twist.linear.x;
    // std::cout << "dx:"  << dx << endl;

    wheel_dx = twist_msg->twist.linear.x;

    if (AutoStatus) return; // 如果使用订阅消息，那么就不使用轮速计判断正反
    if (AutoDirection) {
        wheel_dx = twist_msg->twist.linear.x;
        // std::cout << "  dx "  << dx << endl;
        if (wheel_dx < 0.0) {
            UseLidar = false;
            UseDCamera = true;
        } else {
            UseLidar = true;
            UseDCamera = false;
        }
        // cout << "obj UseLidar:" << UseLidar << "UseDCamera" << UseDCamera <<endl;
    }
}

// SLAM位姿订阅回调函数
void ObjSegGrid_obj::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // // 打印接收到的位姿信息
    // ROS_INFO("Position: (%f, %f, %f), Orientation: (%f, %f, %f, %f)", 
    //        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
    //        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // 将odometry类型转换为eigen矩阵
    Eigen::Matrix4d Pose = Eigen::Matrix4d::Identity();
    Pose.block<3, 1>(0, 3) =
            Eigen::Vector3d(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
    Pose.block<3, 3>(0, 0) =
            Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z).normalized().toRotationMatrix();
    // cerr << "Position(Eigen):" << Pose << endl;
    // 对读入的位姿进行坐标系转换 得到栅格地图坐标系中的IMU坐标系位姿
    // G_T_I = G_T_I0 * I0_T_I
    // Pose_trans = G_T_I * Pose;
    // 建图时直接以IMU初始帧为地图坐标系
    Pose_trans = Pose;
    // cerr << "Position transed(Eigen):" << Pose_trans << endl;
}

// 深度图转换为点云
void ObjSegGrid_obj::Deep2PointCloud(const sensor_msgs::ImageConstPtr &Depth_row_image) {

    auto cv_ptr = cv_bridge::toCvCopy(*Depth_row_image, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat Depth_img = cv_ptr->image;

    int rows = Depth_img.rows, cols = Depth_img.cols;
    pcl::PointXYZ thisPoint;

    int bias_row = 0;
    int bias_col = 0;

    for (int i = 50; i < rows; i += (DCameraRowSampleRate + bias_row)) {
        if (i > rows * 0.9) {
            bias_row = 12;
            bias_col = 12;
        }
        else if (i > rows * 0.8) {
            bias_row = 9;
            bias_col = 9;
        }
        else if (i > rows * 0.7) {//深度图越下方离车越近，降采样倍率越高 indY:108开始
            bias_row = 6;
            bias_col = 6;
        }
        else if (i > rows * 0.65) {//深度图越下方离车越近，降采样倍率越高 indY:110开始
            bias_row = 4;
            bias_col = 5;
        }
        else if (i > rows * 0.6) {//深度图越下方离车越近，降采样倍率越高 indY:112开始
            bias_row = 3;
            bias_col = 4;
        }
        else if (i > rows * 0.55) {//深度图越下方离车越近，降采样倍率越高 indY:114开始
            bias_row = 2;
            bias_col = 3;
        }
        else if (i > rows * 0.5) {//深度图越下方离车越近，降采样倍率越高 indY:116开始
            bias_row = 0;
            bias_col = 1;
        }
//        else if (i > rows * 0.4) {//深度图越下方离车越近，降采样倍率越高
//            bias_row = -1;
//            bias_col = -1;
//        }
        for (int j = 0; j < cols; j += (DCameraColSampleRate + bias_col)) {

            thisPoint.y = (double) (Depth_img.at<unsigned short>(i, j)) / kScaleFactor;
//              if (isnan(thisPoint.y) || thisPoint.y <= 0 || thisPoint.y >= DCameraYAxis)
//                  continue;

            thisPoint.x = ((double) j - dcx) * thisPoint.y / dfx;
//              if(thisPoint.x < -DCameraXAxis || thisPoint.x > DCameraXAxis)
//                  continue;

            thisPoint.z = -((double) i - dcy) * thisPoint.y / dfy;
//              if(thisPoint.z > DCameraZAxis)
//                  continue;

            laserCloudTemp_->points.emplace_back(thisPoint);
        }
    }

}


void ObjSegGrid_obj::SlopeLineHandler(const visualization_msgs::MarkerArrayPtr &LineArray){

//    cout<<"\033[31m"<<"inSlopeLineHandler"<<"\033[0m"<<endl;
//    cout<<"isslope: "<<isslope<<endl;

    if(isslope==false)
        return;
    pcl::PointXYZ lp_max;
    pcl::PointXYZ lp_min;
    pcl::PointXYZ rp_max;
    pcl::PointXYZ rp_min;


    if (is_print_message_) cout<<"LineArray->markers size: "<<LineArray->markers.size()<<endl;

    //单边上下坡情况
    if(LineArray->markers.size() == 1){
        //todo id = 1 左边线，id=2右边线
        if(LineArray->markers[0].id == 1){
//            cout<<"\033[31m"<<"only left line in objectcluster slopefun"<<"\033[0m"<<endl;

            lp_min.x = LineArray->markers[0].points[0].x;
            lp_min.y = LineArray->markers[0].points[0].y;
            lp_min.z = LineArray->markers[0].points[0].z;

            rp_min.x = lp_min.x + 2.45;
            rp_min.y = lp_min.y;
            rp_min.z = lp_min.z;

//            lp_max = LineArray->markers[0].points[1];
            lp_max.x = LineArray->markers[0].points[1].x;
            lp_max.y = LineArray->markers[0].points[1].y;
            lp_max.z = LineArray->markers[0].points[1].z;

            rp_max.x = lp_max.x + 2.45;
            rp_max.y = lp_max.y;
            rp_max.z = lp_max.z;
        }else if(LineArray->markers[0].id == 2){
//            cout<<"\033[31m"<<"only right line in objectcluster slopefun"<<"\033[0m"<<endl;

//            rp_min = LineArray->markers[0].points[0];
            rp_min.x = LineArray->markers[0].points[0].x;
            rp_min.y = LineArray->markers[0].points[0].y;
            rp_min.z = LineArray->markers[0].points[0].z;

            lp_min.x = rp_min.x - 2.45;
            lp_min.y = rp_min.y;
            lp_min.z = rp_min.z;

//            rp_max = LineArray->markers[0].points[1];
            rp_max.x = LineArray->markers[0].points[1].x;
            rp_max.y = LineArray->markers[0].points[1].y;
            rp_max.z = LineArray->markers[0].points[1].z;

            lp_max.x = rp_max.x - 2.45;
            lp_max.y = rp_max.y;
            lp_max.z = rp_max.z;
        }



//        hasline = true;
//        lineusecount = minframeusecount;
//
//        return;

    }else{

        lp_min.x = LineArray->markers[0].points[0].x;
        lp_min.y = LineArray->markers[0].points[0].y;
        lp_min.z = LineArray->markers[0].points[0].z;

        lp_max.x = LineArray->markers[0].points[1].x;
        lp_max.y = LineArray->markers[0].points[1].y;
        lp_max.z = LineArray->markers[0].points[1].z;

        rp_min.x = LineArray->markers[1].points[0].x;
        rp_min.y = LineArray->markers[1].points[0].y;
        rp_min.z = LineArray->markers[1].points[0].z;

        rp_max.x = LineArray->markers[1].points[1].x;
        rp_max.y = LineArray->markers[1].points[1].y;
        rp_max.z = LineArray->markers[1].points[1].z;

    }


//    cout<<"lp_min.x: "<<lp_min.x<<"lp_min.y: "<<lp_min.y<<"lp_min.z: "<<lp_min.z<<endl;
//    cout<<"lp_max.x: "<<lp_max.x<<"lp_max.y: "<<lp_max.y<<"lp_max.z: "<<lp_max.z<<endl;
//    cout<<"rp_min.x: "<<rp_min.x<<"rp_min.y: "<<rp_min.y<<"rp_min.z: "<<rp_min.z<<endl;
//    cout<<"rp_max.x: "<<rp_max.x<<"rp_max.y: "<<rp_max.y<<"rp_max.z: "<<rp_max.z<<endl;

    if (lp_min.x == 0 && lp_min.y == 0 && lp_min.z == 0) {
//        hasline = false;
//        x_min_limit = 0;
//        x_max_limit = 0;
//        y_min_limit = 0;
//        y_max_limit = 0;
        return;

    }
    if(UseLidar){
        if(LineArray->markers[0].color.r == 1.0){
            x_min_limit =  lp_min.x > lp_max .x ? lp_max .x:lp_min.x;
            x_max_limit =  rp_min.x < rp_max .x ? rp_max .x:rp_min.x;
            y_min_limit =  lp_min.y > rp_min.y ? rp_min.y : lp_min.y;
            y_max_limit =  lp_max.y < rp_max.y ? rp_max.y : lp_max.y;
        }else if(LineArray->markers[0].color.g == 1.0){
            x_min_limit =  lp_min.x > lp_max .x ? lp_max .x:lp_min.x;
            x_max_limit =  rp_min.x < rp_max .x ? rp_max .x:rp_min.x;
            y_max_limit =  lp_min.y < rp_min.y ? rp_min.y : lp_min.y;
            y_min_limit =  lp_max.y > rp_max.y ? rp_max.y : lp_max.y;
        }
    }
    else if(UseDCamera){
        if(LineArray->markers[0].color.r == 1.0){
            x_min_limit =  lp_min.x > lp_max .x ? lp_max .x:lp_min.x;
            x_max_limit =  rp_min.x < rp_max .x ? rp_max .x:rp_min.x;
            y_max_limit =  lp_min.y > rp_min.y ? rp_min.y : lp_min.y;
            y_min_limit =  lp_max.y < rp_max.y ? rp_max.y : lp_max.y;
        }else if(LineArray->markers[0].color.g == 1.0){
            x_min_limit =  lp_min.x > lp_max .x ? lp_max .x:lp_min.x;
            x_max_limit =  rp_min.x < rp_max .x ? rp_max .x:rp_min.x;
            y_min_limit =  lp_min.y > rp_min.y ? rp_min.y : lp_min.y;
            y_max_limit =  lp_max.y < rp_max.y ? rp_max.y : lp_max.y;
        }
    }


    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
    pointcloud->points.emplace_back(lp_min);
    pointcloud->points.emplace_back(lp_max);
    pointcloud->points.emplace_back(rp_min);
    pointcloud->points.emplace_back(rp_max);


    pcl::SACSegmentation<pcl::PointXYZ> sac2d;
    sac2d.setInputCloud(pointcloud);
    sac2d.setMethodType(pcl::SAC_RANSAC);
    sac2d.setModelType(pcl::SACMODEL_PLANE);
    sac2d.setDistanceThreshold(0.05);
    sac2d.setMaxIterations(20);
    sac2d.setProbability(0.5);
    sac2d.setOptimizeCoefficients(true);
    sac2d.segment(*inliers, *coefficients); // 提取出斜坡点的索引和斜坡系数


    planecoff.orientation.x = coefficients->values[0];
    planecoff.orientation.y = coefficients->values[1];
    planecoff.orientation.z = coefficients->values[2];
    planecoff.orientation.w = coefficients->values[3];


//    cout<<"x_min_limit: "<<x_min_limit<<"x_max_limit: "<<x_max_limit<<"y_min_limit: "<<y_min_limit<<"y_max_limit: "<<y_max_limit<<endl;

//    if(UseDCamera){
//        x_min_limit -= 0.4;
//        x_max_limit += 0.4;
//        y_min_limit -= 0.4;
//        y_max_limit += 0.4;
//    }else if(UseLidar){
//        x_min_limit -= 0.4;
//        x_max_limit += 0.4;
//        y_min_limit -= 0.4;
//        y_max_limit += 0.4;
//    }


//    cout<<"x_min_limit: "<<x_min_limit<<"x_max_limit: "<<x_max_limit<<"y_min_limit: "<<y_min_limit<<"y_max_limit: "<<y_max_limit<<endl;
    hasline = true;
    lineusecount = minframeusecount;
    hasplanecoff = true;
    planecoffusecount = minframeusecount;


}

void ObjSegGrid_obj::SlopeCoffHandler(const geometry_msgs::PosePtr &Posemsg){

    // cout<<"\033[31m"<<"SlopeCoffHandler"<<"\033[0m"<<endl;

//    planecoff.orientation.x = Posemsg->orientation.x;
//    planecoff.orientation.y = Posemsg->orientation.y;
//    planecoff.orientation.z = Posemsg->orientation.z;
//    planecoff.orientation.w = Posemsg->orientation.w;



    // cout <<"\033[31m"<< "planecoff.orientation.x" << planecoff.orientation.x << "planecoff.orientation.y" << planecoff.orientation.y
    //      << "planecoff.orientation.z" << planecoff.orientation.z << "planecoff.orientation.w" << planecoff.orientation.w<<"\033[0m"
    //      << endl;
//    hasplanecoff = true;
//    planecoffusecount = minframeusecount;
    // cout<<"\033[31m"<<"init_palnecoffusecount: "<<minframeusecount<<"\033[0m"<<endl;

}


//   输入图像回调函数，输入深度图
void ObjSegGrid_obj::CameraHandler(const sensor_msgs::ImageConstPtr &Depth_row_image) {
    CameraDeepCloud->clear();
    laserCloudTemp_->clear();
    if (UseLidar) return;
    // 深度图转换为点云
    Deep2PointCloud(Depth_row_image);

    float cosPitch = cos(DCameraPitch);
    float sinPitch = sin(DCameraPitch);
    pcl::PointXYZ thisPoint;
    for (int i = 0; i < laserCloudTemp_->points.size(); i++) {
        thisPoint.x = laserCloudTemp_->points[i].x;
        thisPoint.y = laserCloudTemp_->points[i].y * cosPitch - laserCloudTemp_->points[i].z * sinPitch;
        thisPoint.z = laserCloudTemp_->points[i].z * cosPitch + laserCloudTemp_->points[i].y * sinPitch;

        if (thisPoint.y > DCameraYAxis || thisPoint.y < 0.01 || thisPoint.x < -DCameraXAxis || thisPoint.x > DCameraXAxis)
            continue;
        if (thisPoint.z < -4 * DeepCameraHigh || thisPoint.z > DeepCameraHigh)
            continue;
//      CameraDeepCloud->points[i] = thisPoint;
        CameraDeepCloud->points.emplace_back(thisPoint);
    }


//    DeepCamera_bbox_filter_.setInputCloud(CameraDeepCloud);
//    DeepCamera_bbox_filter_.filter(*CameraDeepCloud);

    // 将点云按z轴数值大小排序z值小的排的位置小，z值大的索引大
//    sort(CameraDeepCloud->begin(), CameraDeepCloud->end(), [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){return  p1.z < p2.z;});
//    std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
//
//    auto it1 = CameraDeepCloud->points.begin() + outlier_pc * CameraDeepCloud->size();
//    CameraDeepCloud->points.erase(CameraDeepCloud->points.begin(), it1);
//
//    std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
//
//    auto it2 = CameraDeepCloud->points.end() - outlier_pc * CameraDeepCloud->size();
//    CameraDeepCloud->points.erase(it2, CameraDeepCloud->points.end());
//
//    std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
//
//    CameraDeepCloud->width = CameraDeepCloud->points.size();

    newCameraCloud = true;
}



//直接输入点云
//   void ObjSegGrid_obj::CameraHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2)
//{
//
//    // 深度图转换为点云
////     Deep2PointCloud(Depth_row_image);
//
////     pcl::PointCloud<PointT>::Ptr cloud_sub(new pcl::PointCloud<PointT>);	//随机下采样点云
//
//     pcl::fromROSMsg(*laserCloud2, *laserCloudTemp);
//
//    float cosPitch = cos(DCameraPitch);
//    float sinPitch = sin(DCameraPitch);
//    pcl::PointXYZ thisPoint;
//    for (int i = 0; i < laserCloudTemp->points.size(); i++)
//    {
//        thisPoint.x = laserCloudTemp->points[i].x;
//      if(thisPoint.x< -DCameraXAxis || thisPoint.x > DCameraXAxis)//直通滤波
//        continue;
//
//        thisPoint.z = -(laserCloudTemp->points[i].y * cosPitch - laserCloudTemp->points[i].z * sinPitch);
//      if (thisPoint.z > 2 * LidarHigh || thisPoint.z < -2 * DeepCameraHigh)
//        continue;
//
//        thisPoint.y = laserCloudTemp->points[i].z * cosPitch + laserCloudTemp->points[i].y * sinPitch;
//      if(thisPoint.y< -DCameraYAxis || thisPoint.y > DCameraYAxis)
//        continue;
//
//      CameraDeepCloud->push_back(thisPoint);
//    }
//
//
///*
//
//    int ptsize = laserCloudTemp->points.size();
//    pcl::PointXYZ pt;
//    float dis;
//    // 遍历每一个点
//    for (int i = 0; i < ptsize; i++)
//    {
//      pt = laserCloudTemp->points[i];
//
//      if(pt.x< -DCameraXAxis || pt.x > DCameraXAxis)//直通滤波
//        continue;
//      if(pt.y< -DCameraYAxis || pt.y > DCameraYAxis)
//        continue;
//      // 如果点的高度大于激光雷达的安装高度的三倍，则不管了
//      if (pt.z > 2 * LidarHigh)
//        continue;
//
//      //点打在除草车上就不管了
//  //    if(pt.y < MinCarDis)
//  //      continue;
//      // 计算点到原点的距离
//      dis = pt.x * pt.x + pt.y * pt.y;
//
//      // 过远的点就比较稀疏了，不对其降采样
//      if (dis > NoDownSampleThr){
//        CameraDeepCloud->push_back(pt);
//        continue;
//      }
//
//      // 如果点的高度小于地面位置
//      if (pt.z < -DeepCameraHigh)
//      {
//        // 24个点采一个
//        if ((i / 8) % ground_filter_rate == 0)
//        {
//          CameraDeepCloud->push_back(pt);
//        }
//      }
//      else
//      {   //非地面点不降采样
//        if ((i / 8) % point_filter_rate == 0) {
//          CameraDeepCloud->push_back(pt);
//        }
//      }
//    }
//*/
//
//     pcl::RandomSample<pcl::PointXYZ> rs;	//创建滤波器对象
//     rs.setInputCloud(CameraDeepCloud);				//设置待滤波点云
//     rs.setSample(CameraDeepCloud->size()/DCameraRandSampleRate);					//设置下采样点云的点数
//     //rs.setSeed(1);						//设置随机函数种子点
//     rs.filter(*CameraDeepCloud);
//
//
//    sort(CameraDeepCloud->begin(), CameraDeepCloud->end(), comparez);
//
//    auto it1 = CameraDeepCloud->points.begin() + outlier_pc * CameraDeepCloud->size();
//    CameraDeepCloud->points.erase(CameraDeepCloud->points.begin(), it1);
//
//    auto it2 = CameraDeepCloud->points.end() - outlier_pc * CameraDeepCloud->size();
//    CameraDeepCloud->points.erase(it2, CameraDeepCloud->points.end());
//
//    newCameraCloud = true;
//}

// 将点云分布到栅格中
void ObjSegGrid_obj::ProjectCloudToGrid() {
    // laserCloudCrop为随机采样后的点云，这里获取采样后的点云的大小
    int CloudSize = laserCloudCrop->size();

    int indX, indY;
    // 遍历每一个点云
    for (int i = 0; i < CloudSize; i++) {
        // fixme 栅格的中心点好像有偏置 将点云除以单位栅格的长度，得到栅格的id
        indX = floor(GridSizeInverse * (laserCloudCrop->points[i].x + GridSize / 2.0)) + GridCloudHalfWidth;
        indY = floor(GridSizeInverse * (laserCloudCrop->points[i].y + GridSize / 2.0)) + GridCloudHalfWidth;
        // 如果点云不在栅格内，则跳过这个点
        if (indY < 0 || indX < 0 || indX > GridCloudWidth - 1 ||  indY > GridCloudWidth - 1)
            continue;
        // 向栅格内填充点
        GridCloudNodeMap[indX][indY]->Cloud->points.emplace_back(laserCloudCrop->points[i]);

    }

    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth - lidarYAxis * GridSizeInverse;
            indY < GridCloudHalfWidth+ lidarYAxis * GridSizeInverse; indY++) {
            rs.setInputCloud(GridCloudNodeMap[indX][indY]->Cloud);				//设置待滤波点云
            rs.filter(*GridCloudNodeMap[indX][indY]->Cloud);					//执行下采样滤波，保存滤波结果于cloud_sub
        }
    }


    vector<float> grasshigh;
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    vector<float> lowest_point;

    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth - lidarYAxis * GridSizeInverse;
            indY < GridCloudHalfWidth + lidarYAxis * GridSizeInverse; indY++) {

            // 将点云按z轴数值大小排序z值小的排的位置小，z值大的索引大
            sort(GridCloudNodeMap[indX][indY]->Cloud->points.begin(),
                 GridCloudNodeMap[indX][indY]->Cloud->points.end(),
                 [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) { return p1.z < p2.z; });

            if (GridCloudNodeMap[indX][indY]->Cloud->points.size() > 0) {
                GridCloudNodeMap[indX][indY]->base_z = GridCloudNodeMap[indX][indY]->Cloud->points[0].z;
                GridCloudNodeMap[indX][indY]->base_pt = GridCloudNodeMap[indX][indY]->Cloud->points[0];
                GridCloudNodeMap[indX][indY]->max_pt = GridCloudNodeMap[indX][indY]->Cloud->points[GridCloudNodeMap[indX][indY]->Cloud->points.size()-1];
                //                cout<<"checkpt x:"<<GridCloudNodeMap[indX][indY]->base_pt.x<<" y: "<<GridCloudNodeMap[indX][indY]->base_pt.y<<" z: "<<GridCloudNodeMap[indX][indY]->base_pt.z<<endl;
            }

//            if (GridCloudNodeMap[indX][indY]->Cloud->points.size() >= 1) {
//                RemoveOutlaier2(indX, indY);
//            }
        }
    }


    // 遍历每一个栅格
    int bias = 2 * GridSizeInverse;//用前4m的点来计算草高
    int cloudsize;
    float ground_z_elevation;
    float top_point_limit,above_elevation_thr_;

    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth - lidarYAxis * GridSizeInverse;
             indY < GridCloudHalfWidth + lidarYAxis * GridSizeInverse; indY++) {

//            // 将点云按z轴数值大小排序z值小的排的位置小，z值大的索引大
//            sort(GridCloudNodeMap[indX][indY]->Cloud->points.begin(),
//                 GridCloudNodeMap[indX][indY]->Cloud->points.end(),
//                 [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){return  p1.z < p2.z;});
//
//            if(GridCloudNodeMap[indX][indY]->Cloud->points.size() >= 1){
//                RemoveOutlaier2(indX,indY);
//            }




            if (GridCloudNodeMap[indX][indY]->Cloud->points.size() >= minGridCloudNum) {
                //假设2m外的点障碍物草高没有厚度

//                ///计算栅格中点云高度v2,在斜坡上容易误检
//                ///计算栅格最低点用该栅格周围一圈的最低点来，防止栅格中的点没有接地点
//                lowest_point.clear();
//                int cloudsize = GridCloudNodeMap[indX][indY]->Cloud->points.size();
//                for (int dx = -2; dx <= 2; dx++) {
//                    for (int dy = -2; dy <= 2; dy++) {
//
//                        if (indX + dx < 0 || indX + dx > GridCloudWidth - 1 || indY + dy < 0 ||
//                                indY + dy > GridCloudWidth - 1)
//                            continue;
//
//                        if(GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points.size() == 0){
//                            continue;
//                        }
//
//                        lowest_point.emplace_back(GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[0].z);
//                    }
//                }
//
//                sort(lowest_point.begin(),lowest_point.end(),[](const float z1,const float z2){return z1 < z2;});
////                for(auto element : lowest_point){cout<<"indX: "<<indX<<"indY: "<<indY<<element <<endl;}
//                double ground_z_elevation = GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize - 1].z - lowest_point[0];
//                GridCloudNodeMap[indX][indY]->ground_z_elevation = ground_z_elevation;


                ////////////////////////////////////////v2

                ///计算栅格中点云高度v1
                cloudsize = GridCloudNodeMap[indX][indY]->Cloud->points.size();
                ground_z_elevation = GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize - 1].z -
                                            GridCloudNodeMap[indX][indY]->Cloud->points[0].z;
                GridCloudNodeMap[indX][indY]->ground_z_elevation = ground_z_elevation;

//                cout << "indX: " << indX << " indY: " << indY << " ground_z_elevationv2"
//                     << GridCloudNodeMap[indX][indY]->ground_z_elevation << " ground_z_elevationv1: "
//                     << ground_z_elevationv1 << endl;


                top_point_limit = -SensorHigh + min_z_elevation;
//                cout<<"top_point_limit: "<<top_point_limit<<endl;
                above_elevation_thr_ = GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize - 1].z - top_point_limit;
//                cout<<"above_elevation_thr_: "<<above_elevation_thr_<<endl;

//                bias = 2 * GridSizeInverse;//用前2m的点来计算障碍物
                //above_elevation_thr_ > 0 表示栅格的最高点比车轮位置+elevation_thr_ 还要高。
                if (indY < GridCloudHalfWidth + bias && above_elevation_thr_  < 0){
//                    cout<<"indX: "<<indX<<"indY: "<<indY<<"ground_z_elevation: "<<ground_z_elevation<<endl;
                    grasshigh.emplace_back(ground_z_elevation);

                }

            } else if (GridCloudNodeMap[indX][indY]->Cloud->size() >= 2) {

                sort(GridCloudNodeMap[indX][indY]->Cloud->points.begin(),
                     GridCloudNodeMap[indX][indY]->Cloud->points.end(),
                     [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){return  p1.z < p2.z;});
                cloudsize = GridCloudNodeMap[indX][indY]->Cloud->points.size();

                    GridCloudNodeMap[indX][indY]->ground_z_elevation = GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize - 1].z -
                                                                       GridCloudNodeMap[indX][indY]->Cloud->points[0].z;

            }
        }
    }





//    cout<<"grass_high_size: "<<grasshigh.size()<<endl;
    if(grasshigh.size()<5){
//        if(is_print_message_)cout<<"isslope and grasshigh is not enough"<<endl;
        if(plane){
            grasshigh_mid = 0.02;
            grasshigh_real = 0.02;
        }
        else{
            grasshigh_mid = 0.05;
            grasshigh_real = 0.05;
        }

        return;

    }

    ///////////////////////////////////////////
    ///Compute GrassHighv2 计算草高的加权平均
    ///////////////////////////////////////////
    //TODO 计算草高 + 3sigma 为后面ComputeElevScorev2
    map<int,int> elev_value_count;
    int key;
//    vector<pair<int,int>> elev_value_count_vector;
    for(auto &elev_value: grasshigh){
//        cout<<"elev_value: "<<elev_value<<endl;
//        cout<<"approximate elev_value: "<<ceil(elev_value/0.01)<<endl;
        key = 2* ceil(elev_value/0.02);
        elev_value_count[key] += 1;
    }
    //删除草高中位数一下的草地高度，
    vector<pair<int,int>> elev_value_count_vector(elev_value_count.begin(),elev_value_count.end());
    sort(elev_value_count_vector.begin(),elev_value_count_vector.end(),
         [](const pair<int,int> &p1,const pair<int,int> &p2){return p1.second>p2.second;});


    //假设我要去除草多一点，所以不考虑去草地高度矮小一点的
    int mode_grass_high = elev_value_count_vector[0].first;
//    for(int i=1;i< mode_grass_high;i++){
//        if(elev_value_count.count(i) != 0){
//            elev_value_count.erase(i);
//        }
//    }

    grasshigh_mid = grasshigh_real = 0;
    int valid_grasshigh_count = 0;

    for(auto &map_value:elev_value_count){valid_grasshigh_count +=map_value.second;}
    //计算草高的加权平均
    for(auto &map_value: elev_value_count){
//        cout<<"key: "<<map_value.first<<" value: "<<map_value.second<<endl;
        grasshigh_real += (map_value.second/(float)valid_grasshigh_count)*(map_value.first*0.01);
    }
    double mean,stddev;
    pcl::getMeanStd(grasshigh, mean, stddev);

    if(UseDCamera && grasshigh_real < 0.01){
        //防止深度相机的时候草地太矮
        grasshigh_real = 0.01;
        grasshigh_original = grasshigh_real;
    }else{
        grasshigh_original = mode_grass_high*0.01;
    }
    grasshigh_3sigma = grasshigh_real + 3*stddev + 0.2;
    grasshigh_mid =grasshigh_real + 2*stddev + 0.2;

//    if(grasshigh_3sigma < 0.1 && plane ==false){
//        grasshigh_real = grasshigh_mid;
//    }
    if(grasshigh_3sigma > min_z_elevation){
        grasshigh_3sigma = min_z_elevation;
    }
    if(grasshigh_mid > min_z_elevation){
        grasshigh_mid = min_z_elevation;
    }

    if (is_print_message_) cout<<"grasshigh_original: "<<grasshigh_original<<"grasshigh_v2 grasshigh_real: "<<grasshigh_real<<" grasshigh_mid: "<<grasshigh_mid<<" grasshigh_3sigma"<<grasshigh_3sigma<<endl;
//    cout<<"grasshigh_original: "<<grasshigh_original<<endl;


    ///////////////////////////////////////////
    ///Compute GrassHighv1
    ///////////////////////////////////////////
//    sort(grasshigh.begin(), grasshigh.end());
//    int size = grasshigh.size();
//
//    int mid_size = (int) (size * 0.8);
//    grasshigh_mid = grasshigh[mid_size];
//    grasshigh_real = grasshigh_mid;
//    if (grasshigh_real < 0.05 && Grass) {
//        grasshigh_real = 0.05;
//    }
////    else if (grasshigh_real < 0.05 && plane){
////        grasshigh_real = 0.02;
////    }
//    if (grasshigh_mid >= min_z_elevation)
//        grasshigh_mid = min_z_elevation;//草高中位数大于19cm
//    if (grasshigh_mid + 0.05 < 0.1 && Grass) {
//        grasshigh_mid = 0.1;//草高中位数小于0.05
//    } else {
//        grasshigh_mid = grasshigh_mid + 0.05;//草高分位数大于0.05
//    }
//     cout<<"grasshigh_real"<<grasshigh_real<<"grasshigh"<<grasshigh_mid<<"min_grasshigh"<<grasshigh[0]<<"max_grasshigh"<<grasshigh[size-1]<<endl;

    //可视化去除外点的点云
    // laserCloudCrop->clear();
    // for (int indX = 0; indX < GridCloudWidth; indX++)
    // {
    //     for (int indY = (GridCloudWidth-1)/2; indY < GridCloudWidth; indY++) {
    //         if(GridCloudNodeMap[indX][indY]->Cloud->size() > 0)
    //             *laserCloudCrop += *GridCloudNodeMap[indX][indY]->Cloud;
    //     }
    // }

}

template<typename PointT>
inline void ObjSegGrid_obj::extract_initial_seeds_(
        const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds) {

    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;

    // Calculate the mean height value.
    for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            init_seeds.points.emplace_back(p_sorted.points[i]);
        }
    }
}

template<typename PointT>
inline void ObjSegGrid_obj::estimate_plane_(const pcl::PointCloud<PointT> &ground) {
    pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;
}

// For adaptive
template<typename PointT>
// https://blog.csdn.net/qq_38167930/article/details/119165988
// https://blog.csdn.net/qq_33287871/article/details/106183892
inline void ObjSegGrid_obj::extract_piecewiseground(
        const pcl::PointCloud<PointT> &src,
        pcl::PointCloud<PointT> &dst) {
    // 0. Initialization
    if (!ground_pc_.empty())
        ground_pc_.clear();
    if (!dst.empty())
        dst.clear();

    // 1. set seeds!

    // 选取种子点,即选取一个栅格里,z值较小的一群点
    extract_initial_seeds_(src, ground_pc_);
    // 2. Extract ground
    for (int i = 0; i < num_iter_; i++) {
        // 估计平面
        estimate_plane_(ground_pc_);
        ground_pc_.clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(src.points.size(), 3);
        int j = 0;
        for (auto &p : src.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++) {
            if (i < num_iter_ - 1) {
                if (result[r] < th_dist_d_) {
                    ground_pc_.points.emplace_back(src[r]);
                }
            } else { // Final stage
                if (result[r] < th_dist_d_) {
                    dst.points.emplace_back(src[r]);
                }
            }
        }


//        double ground_z_vec = abs(normal_(2, 0));
//        float lenresult = result.size() * 0.7;
//        float lendst = dst.points.size();
//        if (ground_z_vec < uprightness_thr_)
//            non_ground_dst += src;

    }
}

void ObjSegGrid_obj::sort_vec(const VectorXf &vec, VectorXf &sorted_vec) {
    VectorXi ind = VectorXi::LinSpaced(vec.size(), 0, vec.size() - 1);//[0 1 2 3 ... N-1]
    auto rule = [vec](int i, int j) -> bool {
        return vec(i) > vec(j);
    };//正则表达式，作为sort的谓词
    std::sort(ind.data(), ind.data() + ind.size(), rule);
    //data成员函数返回VectorXd的第一个元素的指针，类似于begin()
    sorted_vec.resize(vec.size());
    for (int i = 0; i < vec.size(); i++) {
        sorted_vec(i) = vec(ind(i));
    }
}


void ObjSegGrid_obj::ElevDiffScorev2(int indX, int indY) {

    if(indX == GridCloudWidth - 1 || indX == 0 || indY == GridCloudWidth - 1){
        return;
    }

    int valid_diff_count = 0;
    vector<float> diff_vector;
    int min_point_per_grid;
    if(UseLidar){
        //较远的栅格用更少的点
        if(indY > GridCloudHalfWidth + 2*GridSizeInverse){
            min_point_per_grid = 3;
        }
        else{
            min_point_per_grid = 4;

        }
    }else if(UseDCamera){
        min_point_per_grid = 5;
    }
    int min_valid_diff_count = int((2.0/6.0)*min_point_per_grid * 6.0);
    int real_valid_diff_count = 0;
//    cout<<"min_valid_diff_count: "<<min_valid_diff_count<<endl;

    int valid_grid = 0;
    int cloud_size = GridCloudNodeMap[indX][indY]->Cloud->points.size();
//    if(cloud_size < min_point_per_grid){
//        return;
//    }
    float valid_precent = 0;

    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if ((dx == 0 && dy == 0) ||
                indX + dx <= 0 || indX + dx >= GridCloudWidth - 1 || indY + dy <= 0 ||
                indY + dy >= GridCloudWidth - 1
                || (indY <= GridCloudHalfWidth + 2*GridSizeInverse && GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points.size() <= min_point_per_grid)
               || (indY > GridCloudHalfWidth + 2*GridSizeInverse && GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points.size() < 2)
                ) {
//                cout<<"indX+dx: "<<indX+dx<<"indY+dy: "<<indY+dy<<"cloudsize: "<<GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points.size()<<" not good"<<endl;
                continue;
            }
//            cout<<"indX+dx: "<<indX+dx<<"indY+dy: "<<indY+dy<<"cloudsize: "<<GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points.size()<<"good"<<endl;

            int cloudsize_relative = GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points.size();
            float base_z_diff,point_diff;

            for(int i = cloud_size - 1;i >=(cloud_size - min_point_per_grid) && i >= 0  ;i--){

                valid_diff_count++;

                // fixme [indX + dx][indY + dy]中无点云
                ///v2 计算相邻高度时候考虑斜坡
//                if(dy == 0){
//
//                    point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
//                                       GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;
//                }
//                else if(dy < 0){
//                    //相邻栅格在源栅格前方
//
//                    float r = sqrt((GridCloudNodeMap[indX][indY]->base_pt.x -
//                                    GridCloudNodeMap[indX + dx][indY + dy]->base_pt.x) *
//                                   (GridCloudNodeMap[indX][indY]->base_pt.x -
//                                    GridCloudNodeMap[indX + dx][indY + dy]->base_pt.x)
//                                   + (GridCloudNodeMap[indX][indY]->base_pt.y -
//                                      GridCloudNodeMap[indX + dx][indY + dy]->base_pt.y) *
//                                     (GridCloudNodeMap[indX][indY]->base_pt.y -
//                                      GridCloudNodeMap[indX + dx][indY + dy]->base_pt.y));
//
//
////                    cout<<"indX: "<<indX<<"indY: "<<indY<<"indX + dx "<<indX + dx<<" indY + dy"<<indY + dy<<endl;
//
////                    if(GridCloudNodeMap[indX][indY]->base_z - GridCloudNodeMap[indX + dx][indY + dy]->base_z > GridSize){
////                        //上斜坡上障碍物计算，坡度太大了
////                        base_z_diff = GridCloudNodeMap[indX][indY]->base_z - GridCloudNodeMap[indX + dx][indY + dy]->base_z - GridSize;
////                        point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
////                                (GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z + base_z_diff);
////
////                    }
//                    if (GridCloudNodeMap[indX][indY]->base_z - GridCloudNodeMap[indX + dx][indY + dy]->base_z > 0.03){
//                        //上斜坡
//                        base_z_diff = GridCloudNodeMap[indX][indY]->base_z - GridCloudNodeMap[indX + dx][indY + dy]->base_z;
//
//                        float angle = atan2(base_z_diff,r);
////                        if(angle < 1){
//                            //上斜坡上障碍物计算，小于45度进行补偿
////                            point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
////                                         (GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z + base_z_diff);
//
////                        }
////                        else{
////                            //上斜坡大于45度，不进行补偿
//                            point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
//                                         GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;
////                        }
////                        if(indY ==141 && indX == 100){
////                            cout << "indX: " << indX << "indY: " << indY << "indX + dx " << indX + dx << " indY + dy"
////                                 << indY + dy << base_z_diff << " angle " << angle << " point_diff:" << point_diff
////                                 << endl;
////                        }
//
//                    }
//                    else if(GridCloudNodeMap[indX][indY]->base_z - GridCloudNodeMap[indX + dx][indY + dy]->base_z < -0.1){
//                        //下斜坡障碍物计算，可算凹坑
//                        base_z_diff = GridCloudNodeMap[indX + dx][indY + dy]->base_z - GridCloudNodeMap[indX][indY]->base_z;
//                        float angle = atan2(base_z_diff,r);
//                        //坡度小于30度进行补偿,这是斜坡
//                        if(angle < 0.577){
////                            point_diff = GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z - (GridCloudNodeMap[indX][indY]->Cloud->points[i].z + base_z_diff);
//
////                            point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
////                                         GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;
//                        }else{
//                            //坡度大于30度,认为是凹坑，不进行补偿
////                            point_diff = GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z - GridCloudNodeMap[indX][indY]->Cloud->points[i].z;
////                            GridCloudNodeMap[indX][indY]->cluster_flag = 1;
////                            break;
//
//                        }
//                        point_diff = base_z_diff;
////                        point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
////                                     GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;
//
//                    }else{
//                        //在-0.03～0.03之间，属于点云波动误差
//                        point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
//                                     GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;
////                        if(point_diff < 0)
////                            point_diff = -point_diff;
//
//                    }
//                }
//                else if(dy > 0){
//                    //相邻栅格在源栅格后方
//
//                    float r = sqrt((GridCloudNodeMap[indX][indY]->base_pt.x -
//                                    GridCloudNodeMap[indX + dx][indY + dy]->base_pt.x) *
//                                   (GridCloudNodeMap[indX][indY]->base_pt.x -
//                                    GridCloudNodeMap[indX + dx][indY + dy]->base_pt.x)
//                                   + (GridCloudNodeMap[indX][indY]->base_pt.y -
//                                      GridCloudNodeMap[indX + dx][indY + dy]->base_pt.y) *
//                                     (GridCloudNodeMap[indX][indY]->base_pt.y -
//                                      GridCloudNodeMap[indX + dx][indY + dy]->base_pt.y));
//
//
////                    cout<<"indX: "<<indX<<"indY: "<<indY<<"indX + dx "<<indX + dx<<" indY + dy"<<indY + dy<<endl;
//
////                    if(GridCloudNodeMap[indX][indY]->base_z - GridCloudNodeMap[indX + dx][indY + dy]->base_z > GridSize){
////                        //上斜坡上障碍物计算，坡度太大了
////                        base_z_diff = GridCloudNodeMap[indX][indY]->base_z - GridCloudNodeMap[indX + dx][indY + dy]->base_z - GridSize;
////                        point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
////                                (GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z + base_z_diff);
////
////                    }
//                    if (GridCloudNodeMap[indX + dx][indY + dy]->base_z - GridCloudNodeMap[indX][indY]->base_z > 0.03){
//                        //上斜坡
//                        base_z_diff = GridCloudNodeMap[indX + dx][indY + dy]->base_z - GridCloudNodeMap[indX][indY]->base_z;
//                        float angle = atan2(base_z_diff,r);
////                        if(angle < 1){
//                            //上斜坡上障碍物计算，小于45度进行补偿
////                            point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
////                                         (GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z - base_z_diff);
//
////                        }
////                        else{
////                            //上斜坡大于45度，不进行补偿
//                            point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
//                                         GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;
////                        }
//
//
//
////                        if(indY ==141 && indX == 100){
////                            point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
////                                         (GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z);
////                            cout<<"point_diff: "<<point_diff<<endl;
////                            point_diff  = point_diff + base_z_diff;
////                            cout<<"base_z_diff point_diff: "<<point_diff<<endl;
////                            cout << "indX: " << indX << "indY: " << indY << "indX + dx " << indX + dx << " indY + dy"
////                                 << indY + dy <<"base_z_diff "<< base_z_diff << " angle " << angle << " point_diff:" << point_diff
////                                 << endl;
////                        }
//
//                    }else if(GridCloudNodeMap[indX + dx][indY + dy]->base_z - GridCloudNodeMap[indX][indY]->base_z < -0.1){
//                        //下斜坡障碍物计算，可算凹坑
//                        base_z_diff = GridCloudNodeMap[indX][indY]->base_z - GridCloudNodeMap[indX + dx][indY + dy]->base_z;
//                        float angle = atan2(base_z_diff,r);
//                        //坡度小于30度进行补偿,这是斜坡
//                        if(angle < 0.577){
////                            point_diff = GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z - (GridCloudNodeMap[indX][indY]->Cloud->points[i].z - base_z_diff);
//
////                            point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
////                                         GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;
//
//                        }else{
//                            //坡度大于30度,认为是凹坑，不进行补偿
////                            point_diff = GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z - GridCloudNodeMap[indX][indY]->Cloud->points[i].z;
////                            GridCloudNodeMap[indX][indY]->cluster_flag = 1;
////                            break;
//
//                        }
//                        point_diff = base_z_diff;
//
////                        point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
////                                     GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;
//
//                    }else{
//                        //在-0.03～0.03之间，属于点云波动误差
//                        point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
//                                     GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;
////                        if(point_diff < 0)
////                            point_diff = -point_diff;
//
//                    }
//                }
//
//                if(point_diff < 0){
//                    point_diff = -point_diff;
//                }




//                float base_z_diff = GridCloudNodeMap[indX][indY]->base_z - GridCloudNodeMap[indX + dx][indY + dy]->base_z;
//                if(base_z_diff < 0){
//                    cout<<"indX + dx"<<indX + dx<<" indY + dy "<<indY + dy<<"base_z_diff "<<base_z_diff<<endl;
//
//                }
//                float point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
//                                   GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative - 1].z
//                                   - base_z_diff;

                ///v1
                float point_diff = GridCloudNodeMap[indX][indY]->Cloud->points[i].z -
                                   GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points[cloudsize_relative-1].z;



                if(point_diff > 0){
                    diff_vector.emplace_back(point_diff);
                }else{
                    break;
                }
            }



        }
    }
//    cout<<"indX: "<<indX<<" indY: "<<indY<<" min_valid_diff_count: "<<diff_vector.size()<<endl;

    if(diff_vector.size() < min_valid_diff_count){
        return;
    }else{
        int middle_element = floor(diff_vector.size()/2);
//        cout<<"middle_element_size: "<<middle_element<<endl;
        sort(diff_vector.begin(),diff_vector.end(),[](float p1,float p2){return p1<p2;});
//        middle_element = floor(diff_vector.size()/2);
//        cout<<"middle_element: "<<middle_element<<endl;
//        for( auto &element: diff_vector)
//            cout<<"element: "<<element<<endl;
//        cout<<"middle_value: "<<middle_value<<endl;

//        nth_element(diff_vector.begin(),diff_vector.begin()+middle_element,diff_vector.end());

        float middle_value = diff_vector[middle_element];

        double mean,stddev;
        pcl::getMeanStd(diff_vector, mean, stddev);

        //todo fixit
        float final_relative_diff;
        if(valid_diff_count > 2* min_valid_diff_count){
            if(middle_value < relative_diff_thr){
                final_relative_diff = middle_value + 2*stddev;

            }else{
                final_relative_diff = middle_value + 3*stddev;

            }
        }else{
            final_relative_diff = middle_value + 2*stddev;
        }
        if(plane){
            final_relative_diff = diff_vector[diff_vector.size() - 1];
        }

//        cout<<"diff_vector[diff_vector.size() - 1]: "<<diff_vector[diff_vector.size() - 1]<<"diff_vector[diff_vector.size() - 2]: "<<diff_vector[diff_vector.size() - 2]<<endl;

//        if(final_relative_diff > min_z_elevation){
//            for(auto &diff_value : diff_vector){
//                cout<<"diff_value: "<<diff_value<<endl;
//            }
//        }
//        cout<< "indX: " << indX << " indY: " << indY<<"middle_value: "<<middle_value<<"stddev: "<<stddev<<"final_relative_diff: "<<final_relative_diff<<endl;

//        if(final_relative_diff > relative_diff_thr)
//            GridCloudNodeMap[indX][indY]->cluster_flag = 1;
//        GridCloudNodeMap[indX][indY]->elev_diff_score = final_relative_diff -grasshigh_original;
        GridCloudNodeMap[indX][indY]->elev_diff_score = final_relative_diff;


    }

}

void ObjSegGrid_obj::ComputeScorev2(int indX, int indY) {
    ElevDiffScorev2(indX, indY);

}



void ObjSegGrid_obj::ComputeBigHolev2() {

    planecoeffArray.clear();

#ifdef Debug
    visualization_msgs::MarkerArray planemark_array;
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = "camera_init";
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 1.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;
#endif


    //多块局部区域拟合地平面
    float maxY,minY,maxX,minX;
    int minpointspregrid = 5;
    int mincountpergrid = 3;
//    YAxisPlaneNum=2,XAxisPlaneNum=2;


    if(UseLidar){
        minY = MinCarDis;maxY = BigHoleMaxY;
        minX = -lidarXAxis;maxX = lidarXAxis;
    }else if(UseDCamera){
        minY = 0.45;maxY = BigHoleMaxY;
        minX = -DCameraXAxis;maxX = DCameraXAxis;
    }

    startindX = GridCloudHalfWidth + int(GridSizeInverse*minX);
    endindX = GridCloudHalfWidth + int(GridSizeInverse*maxX);
    indXInterval = int((endindX - startindX + 1)/XAxisPlaneNum);

    startindY = GridCloudHalfWidth + int(GridSizeInverse*minY);
    endindY = GridCloudHalfWidth + GridSizeInverse*maxY;
    indYInterval = int((endindY - startindY + 1)/YAxisPlaneNum);

//    cout<<"startindX:"<<startindX<<" indXInterval:"<<indXInterval<<" endindX:"<<endindX<<endl;
//    cout<<"startindY:"<<startindY<<" indYInterval:"<<indYInterval<<" endindY:"<<endindY<<endl;

    vector<pair<int, int>> candidate;
    for(int i = 0;i<XAxisPlaneNum;i++){
        for(int j=0;j<YAxisPlaneNum;j++){
            int index = i*YAxisPlaneNum + j;

//            cout << "index: " << index << "startindX: " << startindX + i * indXInterval << "endindX: "
//                 << startindX + (i + 1) * indXInterval -1<< "startindY: " << startindY + j * indYInterval
//                 << "endindY: " << startindY + (j + 1) * indYInterval -1<< endl;


//            int index = j*XAxisPlaneNum + i;
            pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud = CloudVector[index];

//            std::cout<<"is null"<<(pointcloud == nullptr)<<std::endl;
//            pcl::PointXYZ temppoint;
//            temppoint.x = 1;
//            temppoint.y = 2;
//            temppoint.z =3;
//
//            pointcloud->points.emplace_back(temppoint);
////            pointcloud->points.emplace_back(temppoint);
            for(int indX = startindX + i*indXInterval- additional_grid;indX <startindX + (i+1)*indXInterval + additional_grid;indX++) {
                for (int indY = startindY + j * indYInterval - additional_grid; indY < startindY + (j + 1) * indYInterval + additional_grid; indY++) {
//                    cout<<"indX: "<<indX<<"indY: "<<indY<<"Cloudsize: "<<GridCloudNodeMap[indX][indY]->Cloud->points.size()<<endl;

                    float grid_min_z = GridCloudNodeMap[indX][indY]->Cloud->points[0].z;


                    if (GridCloudNodeMap[indX][indY]->Cloud->points.size() > minpointspregrid) {

                        for(int k = 0;k<minpointspregrid;k++){
                            pointcloud->points.emplace_back(GridCloudNodeMap[indX][indY]->Cloud->points[k]);
//                            UnderGroundCloud->points.emplace_back(GridCloudNodeMap[indX][indY]->Cloud->points[k]);
                        }

                    }else{
                        for(int k = 0;k<GridCloudNodeMap[indX][indY]->Cloud->points.size();k++){
                            pointcloud->points.emplace_back(GridCloudNodeMap[indX][indY]->Cloud->points[k]);
//                            UnderGroundCloud->points.emplace_back(GridCloudNodeMap[indX][indY]->Cloud->points[k]);

                        }
                    }

                }
            }

            if(pointcloud->points.size() < 0.2*indXInterval*indYInterval*minpointspregrid){
//                coefficients = nullptr;
//                cout << "bighole index size to small : " << index << " pointcloud_size: " << pointcloud->size() << "Intervalgrid: "
//                     << 0.2 * indXInterval * indYInterval * minpointspregrid << endl;
                planecoeffArray.emplace_back(nullptr);
                continue;
            }

//            cout<<"before i: "<<i<<" j: "<<j<<" cloudsize: "<<pointcloud->points.size()<<endl;

            double mean;    //点云均值
            double stddev;    //点云标准差
            vector<float> vec_x;
            for (size_t i = 0; i < pointcloud->points.size(); i++) { vec_x.emplace_back(pointcloud->points[i].z); }
            pcl::getMeanStd(vec_x, mean, stddev);


//    cout << "mean:" << mean << " stdz:" << stddev;
            sort(pointcloud->points.begin(), pointcloud->points.end(), [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){return  p1.z < p2.z;});
            auto it = pointcloud->points.begin();

//            cout<<"index: "<<index<<"before erase pointcloud size: "<<pointcloud->points.size()<<endl;
            for (int i = 0; i < pointcloud->points.size(); i++) {
                if (pointcloud->points[i].z < mean - 1*stddev) {
                    it++;

                }
            }
            pointcloud->erase(pointcloud->points.begin(), it);
            pointcloud->width = pointcloud->points.size();

#ifdef Debug
            *UnderGroundCloud += *pointcloud;

#endif

//            cout<<"index: "<<index<<"after erase pointcloud size: "<<pointcloud->points.size()<<endl;

            //如果点云波动依旧很大，则不计算凹坑
            vec_x.clear();
            for (size_t i = 0; i < pointcloud->points.size(); i++) { vec_x.emplace_back(pointcloud->points[i].z); }
            pcl::getMeanStd(vec_x, mean, stddev);
            if (2*stddev > 0.07){

//                cout<<"index: "<<index<<" stddev: "<<stddev<<endl;

                planecoeffArray.emplace_back(nullptr);
                continue;
            }

//            cout<<"after i: "<<i<<" j: "<<j<<" cloudsize: "<<pointcloud->points.size()<<endl;

            ///v1 Ransac
//            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//
//            pcl::SACSegmentation<pcl::PointXYZ> sac2d;
//            sac2d.setInputCloud(pointcloud);
//            sac2d.setMethodType(pcl::SAC_RANSAC);
//            sac2d.setModelType(pcl::SACMODEL_PLANE);
//            sac2d.setDistanceThreshold(0.06);
//            sac2d.setMaxIterations(500);
//            sac2d.setProbability(0.5);
//            sac2d.setOptimizeCoefficients(true);
//            sac2d.segment(*inliers, *coefficients); // 提取出斜坡点的索引和斜坡系数
//
//
//            float nx = coefficients->values[0];
//            float ny = coefficients->values[1];
//            float nz = coefficients->values[2];
//            float d = coefficients->values[3];//nx*X+ny*Y+nz*Z+d=0
//            float r1 = sqrt(nx * nx + ny * ny + nz * nz);

            ///v2 最小二乘
//            Eigen::RowVector3d normal;
//            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//            float d;
//            computeplaneuseSVD(pointcloud, normal, d);
//
//            coefficients->values.emplace_back(float(normal.x())) ;
//            coefficients->values.emplace_back(float(normal.y())) ;
//            coefficients->values.emplace_back(float(normal.z())) ;
//            coefficients->values.emplace_back(d) ;
//
//            planecoeffArray.emplace_back(coefficients);
//
//            float nx = normal.x();
//            float ny = normal.y();
//            float nz = normal.z();
//
//            if (nz < 0) {
//                nx = -nx;
//                ny = -ny;
//                nz = -nz;
//                d = -d;
//            }

            ///v3 use ransac and Least Square
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

            pcl::SACSegmentation<pcl::PointXYZ> sac2d;
            sac2d.setInputCloud(pointcloud);
            sac2d.setMethodType(pcl::SAC_RANSAC);
            sac2d.setModelType(pcl::SACMODEL_PLANE);
            sac2d.setDistanceThreshold(0.06);
            sac2d.setMaxIterations(500);
            sac2d.setProbability(0.5);
            sac2d.setOptimizeCoefficients(true);
            sac2d.segment(*inliers, *coefficients); // 提取出斜坡点的索引和斜坡系数


            float ransacnx = coefficients->values[0];
            float ransacny = coefficients->values[1];
            float ransacnz = coefficients->values[2];
            float ransacd = coefficients->values[3];//nx*X+ny*Y+nz*Z+d=0
            float ransacr1 = sqrt(ransacnx * ransacnx + ransacny * ransacny + ransacnz * ransacnz);


            Eigen::RowVector3d LSnormal;
            float LSd;
            computeplaneuseSVD(pointcloud, LSnormal, LSd);

            float LSnx = LSnormal.x();
            float LSny = LSnormal.y();
            float LSnz = LSnormal.z();
            float LSr1 = sqrt(LSnx * LSnx + LSny * LSny + LSnz * LSnz);

            if (ransacnz < 0) {
                ransacnx = -ransacnx;ransacny = -ransacny;ransacnz = -ransacnz;ransacd = -ransacd;
            }
            if (LSnz < 0) {
                LSnx = -LSnx;LSny = -LSny;LSnz = -LSnz;LSd = -LSd;
            }
            float angle = (ransacnx*LSnx +ransacny*LSny+ransacnz*LSnz)/(ransacr1*LSr1);
            if(angle < 0.965){
//                cout<<"\033[31;1m"<<"index: "<<index<<" plane angle between ransac and LS is to big :"<<angle<<"\033[0m"<<endl;
                planecoeffArray.emplace_back(nullptr);
                continue;
            }
//            cout<<"index: "<<index<<" angle with each other :"<<angle<<endl;

//            else{
//                cout<<"\033[33;1m"<<"index: "<<index<<" plane angle between ransac and LS :"<<angle<<"\033[0m"<<endl;
//
//            }

            //如果平面太斜了cos30 0.866 cos20 0.939 cos15 0.965 cos10 0.985
//            cout<<"index: "<<index<<" angle with z ransac :"<<ransacnz/ransacr1<<"LS :"<< (LSnz/LSr1)<<endl;

            if((ransacnz/ransacr1) < 0.965 || (LSnz/LSr1) < 0.965){
                if(is_print_message_) cout<<"index: "<<index<<" (ransacnz/ransacr1)"<<(ransacnz/ransacr1)<<" (LSnz/LSr1)"<<(LSnz/LSr1)<<endl;
                planecoeffArray.emplace_back(nullptr);
                continue;
            }

            pcl::computeMeanAndCovarianceMatrix(*pointcloud, cov_, pc_mean_);

            //平面中心太高
//            if(pc_mean_.z() > -SensorHigh + grasshigh_mid +){

//            cout << "index: " << index << " pc_mean_.z()" << pc_mean_.z() << " -SensorHigh + grasshigh_mid"
//                 << -SensorHigh + grasshigh_mid << " pointcloud->points[0].z + grasshigh_mid"
//                 << pointcloud->points[0].z + grasshigh_mid << endl;
            //todo 地面不平，可能是由于点云畸变导致
            if(pc_mean_.z() > -SensorHigh+0.1){
//                cout<<"\033[31;1m"<<"index: "<<index<<" planemean is higher than -SensorHigh+0.1:"<<-SensorHigh+0.1<<" planehigh: "<<pc_mean_.z()<<"\033[0m"<<endl;

                planecoeffArray.emplace_back(nullptr);
                continue;
            }else{
//                cout<<"\033[33;1m"<<"index: "<<index<<" planemean is lower than -SensorHigh+0.1:"<<-SensorHigh+0.1<<" planehigh: "<<pc_mean_.z()<<"\033[0m"<<endl;

            }

//            float nx = LSnx;
//            float ny = LSny;
//            float nz = LSnz;
//            float r1 = LSr1;
//            float d = LSd;

            float nx = ransacnx;
            float ny = ransacny;
            float nz = ransacnz;
            float r1 = ransacr1;
            float d = ransacd;

            Eigen::Vector3f vectorbefore(0, 0, 1);
            Eigen::Vector3f vectorafter(nx, ny, nz);
            Eigen::Quaternionf rotQuaternionf = Eigen::Quaternionf::FromTwoVectors(vectorbefore, vectorafter);



            pcl::PointXYZ min; //xyz的最小值
            pcl::PointXYZ max; //xyz的最大值
            pcl::getMinMax3D(*pointcloud, min, max); //在聚类的坐标系中计算最大最小值

            planecoeffArray.emplace_back(coefficients);

#ifdef Debug
            bbox_marker.id = index+1;
            bbox_marker.color.a = 0.5;
            bbox_marker.pose.position.x = pc_mean_.x();
            bbox_marker.pose.position.y = pc_mean_.y();
            bbox_marker.pose.position.z = pc_mean_.z();
            bbox_marker.pose.orientation.x = rotQuaternionf.x();
            bbox_marker.pose.orientation.y = rotQuaternionf.y();
            bbox_marker.pose.orientation.z = rotQuaternionf.z();
            bbox_marker.pose.orientation.w = rotQuaternionf.w();

            bbox_marker.scale.x = max.x - min.x;
            bbox_marker.scale.y = max.y - min.y;
            bbox_marker.scale.z = 0.02;

            bbox_marker.header.stamp = laser_Stamp.stamp;
            planemark_array.markers.emplace_back(bbox_marker);
#endif

            for (int indX = startindX + i * indXInterval; indX < startindX + (i + 1) * indXInterval; indX++) {
                for (int indY = startindY + j * indYInterval; indY < startindY + (j + 1) * indYInterval; indY++) {



                    float value;
                    int count;
                    //todo 是否需要对凹坑在z轴方向进行限制
                    if ((GridCloudNodeMap[indX][indY]->Cloud->points.size() >= minpointspregrid && UseLidar) ||
                        (GridCloudNodeMap[indX][indY]->Cloud->points.size() >= minpointspregrid && UseDCamera)) {

//                    cout<<"1  indX: "<<indX<<" indY: "<<indY<<" count :"<<count<<endl;

                        count = 0;
                        for (int i = 0; i < mincountpergrid; i++) {


                            value = (nx * GridCloudNodeMap[indX][indY]->Cloud->points[i].x +
                                     ny * GridCloudNodeMap[indX][indY]->Cloud->points[i].y +
                                     nz * GridCloudNodeMap[indX][indY]->Cloud->points[i].z + d) / r1;

//                        if(index == 2){
//                            cout<<"index : "<<index<<" indX: "<<indX<<" indY: "<<indY<<" value: "<<value<<endl;
//                            //在车很近处且在车轮子底下，因为在车很近的地方，（车就在一个有点凹坑的地上会误检）
//                            if(GridCloudNodeMap[indX][indY]->Cloud->points[i].z < -SensorHigh - 0.08){
//                                count++;
//
//                            }
//                        }
//                        else
                            //加了第二个判断则不考虑车在平地、斜坡上有凹坑的情况
//                        if (value < -min_hole_depth && GridCloudNodeMap[indX][indY]->Cloud->points[i].z < -SensorHigh - min_hole_depth) {
                            if (value < -min_hole_depth) {

                                //在平面下方
                                count++;
//                                if (is_print_message_) cout<<"index : "<<index<<" indX: "<<indX<<" indX: "<<indX<<" value: "<<value<<endl;

//                            cout <<"bighole indX:"<<indX<<"bighole indY:"<<indY<<"count: "<<count<<endl;

                            }

                            if ((count >= mincountpergrid && UseLidar) || (count >= mincountpergrid && UseDCamera) ||
                                (count >= 1 && UseLidar && startindY >= 120) || (count >= 1 && UseDCamera && startindY >= 120) ) {


//                                if ((GridCloudNodeMap[indX][indY]->ground_z_vec < 0.98 && UseLidar) ||
//                                    (GridCloudNodeMap[indX][indY]->ground_z_vec < 0.98 && UseDCamera)) {
                                    GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                    GridCloudNodeMap[indX][indY]->is_small_hole = true;
                                    if (is_print_message_) cout<<"aokeng object indX: "<<indX<<" indY: "<<indY<<" gridhigh: "<<GridCloudNodeMap[indX][indY]->ground_z_elevation<<endl;

                                    candidate.emplace_back(pair<int, int>(indX, indY));
//                                    cout << "bighole indX:" << indX << "bighole indY:" << indY << "planarity: "
//                                         << GridCloudNodeMap[indX][indY]->ground_z_vec << endl;

                                    break;
//                                }

                            }

                        }
                    }

                }
            }


        }
    }

#ifdef Debug
    pub_grassplane_.publish(planemark_array);
#endif

    //凹坑连通
    for(int i =0;i < candidate.size();i++){
//        cout<<"i:"<<i<<endl;
        int indx1 = candidate[i].first;
        int indy1 = candidate[i].second;
        GridCloudNodeMap[indx1][indy1]->cluster_flag = 1;
        GridCloudNodeMap[indx1][indy1]->is_small_hole = true;

        for(int j =i+1;j<candidate.size();j++){
//            cout<<"j:"<<j<<endl;
            int indx2 = candidate[j].first;
            int indy2 = candidate[j].second;
            int distance = (indx1 - indx2) * (indx1 - indx2) + (indy1 - indy2) * (indy1 - indy2);
            //设置为9 的话是凹坑中心周围5×5的其他凹坑连接起来(0.2m)，18的话是7×7(0.3m)，32的话是9×9(0.4m),50的话是11×11(0.5m),72的话是13×13(0.6m)
            if(distance <= connect_distance){
                int max_indx = indx1>indx2 ? indx1:indx2;
                int min_indx = indx1>indx2 ? indx2:indx1;
                int max_indy = indy1>indy2 ? indy1:indy2;
                int min_indy = indy1>indy2 ? indy2:indy1;
//                cout<<"max_indx"<<max_indx<<"min_indx"<<min_indx<<"max_indy"<<max_indy<<"min_indy"<<min_indy<<endl;
                for(int indx = min_indx;indx<=max_indx;indx++){
                    for(int indy = min_indy;indy<=max_indy;indy++){
                        GridCloudNodeMap[indx][indy]->cluster_flag = 1;
                        GridCloudNodeMap[indx][indy]->is_small_hole = true;

//                        cout<<"aokeng surround:"<<indx<<"  "<<indy<<endl;

                    }
                }

            }
        }
    }



}

void ObjSegGrid_obj::SlopeCheckv2(int indX, int indY) {


    //9个方向
//    vector<pair<int,int>> direction{pair<int,int>(-1,0),
//                                    pair<int,int>(-2,1),
//                                    pair<int,int>(-1,1),
//                                    pair<int,int>(-1,2),
//                                    pair<int,int>(0,1),
//                                    pair<int,int>(1,2),
//                                    pair<int,int>(1,1),
//                                    pair<int,int>(2,1),
//                                    pair<int,int>(1,0)};

    if (indX < GridCloudHalfWidth - SensorXAxis * GridSizeInverse ||
            indX > GridCloudHalfWidth + SensorXAxis * GridSizeInverse ||
            indY > GridCloudHalfWidth + SensorYAxis * GridSizeInverse ||
            indY < GridCloudHalfWidth + MinCarDis * GridSizeInverse ||
        GridCloudNodeMap[indX][indY]->cluster_flag >= 1 ||
        GridCloudNodeMap[indX][indY]->Cloud->points.size() == 0){
        return;
    }

    //斜坡大于cos15度0.965，cos10度0.984,20度0.939 ,15度0.267,10度0.176
    if (GridCloudNodeMap[indX][indY]->ground_z_vec > 0.939 &&
        GridCloudNodeMap[indX][indY]->Cloud->points.size() > MaxPtPerGrid / 2 && isslope == false) {
        //平面垂直，点数很对，说明是地面
        return;
    }

    vector<pair<int,int>> direction;

    int total_step = 1;//todo 后续改成按照栅格远近设置自适应step
    int valid_count;
    int valid_line_count = 0;
    int direction_x,direction_y;
    int up_down_flag;//1上坡，2下坡,0待定
    float max_up_angle,max_down_angle;
    float max_max_up_angle,max_max_down_angle;
    float min_xy_dis = 0.05;
    int max_recompute_angle_count = 3;

    max_max_up_angle = -99;
    max_max_down_angle = 99;

    //斜坡大于30度0.577，25度0.466,20度0.363 ,15度0.267,10度0.176
    float slope_angle_thr;
    if(isslope){
        slope_angle_thr = 0.176;
    }else{
        slope_angle_thr = 0.363;

    }
//    int line_index_start,line_index_end;
//    line_index_start = 0;
//    line_index_end = direction.size();

    //不检测内侧斜坡
    if (isslope) {
        direction = vector<pair<int, int>>{
//                pair<int, int>(-3, 0),
//                pair<int, int>(-3, 1),
//                pair<int, int>(-3, 2),
//                pair<int, int>(-3, 3),
//                pair<int, int>(-2, 3),
//                pair<int, int>(-1, 3),
//                pair<int, int>(0, 3),
//                pair<int, int>(1, 3),
//                pair<int, int>(2, 3),
//                pair<int, int>(3, 3),
//                pair<int, int>(3, 2),
//                pair<int, int>(3, 1),
//                pair<int, int>(3, 0),
//                pair<int, int>(2, 2),
//
//                pair<int, int>(-1, 2),
//                pair<int, int>(-2, 0),
//                pair<int, int>(-1, 0),
//                pair<int, int>(-2, 1),
//                pair<int, int>(-1, 1),
//                pair<int, int>(-1, 2),
//                pair<int, int>(0, 1),
//                pair<int, int>(0, 2),
//                pair<int, int>(1, 2),
//                pair<int, int>(1, 1),
//                pair<int, int>(2, 1),
//                pair<int, int>(1, 0),
//                pair<int, int>(2, 0)


                pair<int,int>(-1,0),
//                pair<int,int>(-2,1),
                pair<int, int>(-1, 1),
//                pair<int, int>(-1, 2),
                pair<int, int>(0, 1),
//                pair<int, int>(1, 2),
                pair<int, int>(1, 1),
//                pair<int, int>(2, 1),
                pair<int, int>(1, 0)

        };
    }else{
        direction = vector<pair<int, int>>{
                pair<int,int>(-1,0),
                pair<int,int>(-2,1),
                pair<int, int>(-1, 1),
                pair<int, int>(-1, 2),
                pair<int, int>(0, 1),
                pair<int, int>(1, 2),
                pair<int, int>(1, 1),
                pair<int, int>(2, 1),
                pair<int, int>(1, 0)
        };
    }


    for(int i = 0;i<direction.size();i++){
        valid_count = 0;
        max_up_angle = -99;
        max_down_angle = 99;
        up_down_flag = 0;
        vector<float> slopeanglelist;

        for(int step=1;step<=total_step;step++){
            direction_x = indX + step * direction[i].first;
            direction_y = indY + step * direction[i].second;

            if (direction_x < GridCloudHalfWidth - SensorXAxis * GridSizeInverse ||
                direction_x > GridCloudHalfWidth + SensorXAxis * GridSizeInverse ||
                direction_y > GridCloudHalfWidth + SensorYAxis * GridSizeInverse ||
                direction_y < GridCloudHalfWidth + MinCarDis * GridSizeInverse ||
                GridCloudNodeMap[direction_x][direction_y]->cluster_flag >= 1 ||
                    GridCloudNodeMap[direction_x][direction_y]->Cloud->points.size() == 0){
                continue;
            }


            float xy_dis = sqrt((GridCloudNodeMap[direction_x][direction_y]->max_pt.y -
                              GridCloudNodeMap[indX][indY]->max_pt.y) *
                             (GridCloudNodeMap[direction_x][direction_y]->max_pt.y -
                              GridCloudNodeMap[indX][indY]->max_pt.y)
                             + (GridCloudNodeMap[direction_x][direction_y]->max_pt.x -
                                GridCloudNodeMap[indX][indY]->max_pt.x) *
                               (GridCloudNodeMap[direction_x][direction_y]->max_pt.x -
                                GridCloudNodeMap[indX][indY]->max_pt.x));
            //下坡z_high < 0 上坡 z_high>0
            float z_high = GridCloudNodeMap[direction_x][direction_y]->max_pt.z - GridCloudNodeMap[indX][indY]->max_pt.z;
            float slopeangle = atan2(z_high, xy_dis);
            int loopcount = 1;
            int cloudsize = GridCloudNodeMap[indX][indY]->Cloud->points.size();
            while (xy_dis < min_xy_dis && loopcount < max_recompute_angle_count &&
                   GridCloudNodeMap[indX][indY]->Cloud->points.size() > loopcount){
                xy_dis = sqrt((GridCloudNodeMap[direction_x][direction_y]->max_pt.y -
                                     GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize -1 - loopcount].y) *
                                    (GridCloudNodeMap[direction_x][direction_y]->max_pt.y -
                                     GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize -1 - loopcount].y)
                                    + (GridCloudNodeMap[direction_x][direction_y]->max_pt.x -
                                       GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize -1 - loopcount].x) *
                                      (GridCloudNodeMap[direction_x][direction_y]->max_pt.x -
                                       GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize -1 - loopcount].x));
                //下坡z_high < 0 上坡 z_high>0
                z_high = GridCloudNodeMap[direction_x][direction_y]->max_pt.z - GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize -1 - loopcount].z;
                slopeangle = atan2(z_high, xy_dis);

                loopcount ++;
            }

            //距离过短，或者倾斜角度过大，都不是斜坡。1为45度
            if(slopeangle > 1 || slopeangle < -1 || xy_dis < min_xy_dis){
                continue;
            }
//            cout<<"indX:"<<indX<<" indY:"<<indY<<" z_high:"<<z_high<<" slopeangle: "<<slopeangle<<endl;

            //表示当前栅格计算出的角度是可用的
            valid_count++;

//            cout << "indX: " << indX << " indY: " << indY << " direction_x:" << direction_x << " direction_y: "
//                 << direction_y << " xy_dis: " << xy_dis << " z_high: " << z_high << " slopeangle: " << slopeangle
//                 << endl;

            if(up_down_flag ==0){
                //首次计算是上坡还是下坡
                slopeanglelist.emplace_back(slopeangle);
                if(slopeangle > 0 && CheckUpSlope){
                    up_down_flag = 1;
                    max_up_angle = slopeangle;
                }else if(slopeangle < 0 && CheckDownSlope){
                    up_down_flag = 2;
                    max_down_angle = slopeangle;

                }
            }else if(up_down_flag !=0){
                //要求一条线上的每次角度计算都要是上坡角度或者下坡角度
                if((slopeangle > 0 && up_down_flag != 1) || (slopeangle < 0 && up_down_flag != 2)){

                    //todo 角度一次上坡一次下坡
                    //当前线不进行判断
                    valid_count = 0;
                    break;
                }else{
                    //todo 改一下这个角度值 M_PI/18
                    if(abs(*slopeanglelist.end() - slopeangle) > M_PI/18){
                        //同一条线上的角度差大于10度，有可能是地上的线到障碍物上或者是障碍物上的点到地上
                        valid_count=0;
                        break;
                    }

                    //当前直线角度都是上坡或者下坡
                    if(slopeangle > max_up_angle){
                        max_up_angle = slopeangle;
                    }else if(slopeangle < max_down_angle){
                        max_down_angle = slopeangle;
                    }
                }

            }

//            if(max_up_angle > 0.363 || max_down_angle < -0.363){
//                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
//                return;
//            }
        }

        //斜坡大于 45度 1,30度0.577，25度0.466,20度0.363 ,15度0.267,10度0.176
        if(valid_count == 0){
            //当前线条不符合要求
            continue;
        }
        else if((valid_count < total_step && isslope == false )|| (valid_count <= total_step && isslope == true)){
            //todo
            if((up_down_flag ==1 && max_up_angle > 0.267) || (up_down_flag ==2 && max_down_angle < -0.267)){
                valid_line_count++;
//                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;

                if(max_up_angle > max_max_up_angle){
                    max_max_up_angle = max_up_angle;
                }
                if(max_down_angle < max_max_down_angle){
                    max_max_down_angle = max_down_angle;
                }
            }
            if(valid_line_count > 0.3 * direction.size() &&((max_max_down_angle < -slope_angle_thr && CheckDownSlope) || (max_max_up_angle > slope_angle_thr && CheckUpSlope))){
//                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                GridCloudNodeMap[indX][indY]->slopeflag =1;
                GridCloudNodeMap[direction_x][direction_y]->slopeflag = 1;

                if(up_down_flag == 1){
                    GridCloudNodeMap[indX][indY]->slopeangle = max_max_up_angle;
                    GridCloudNodeMap[direction_x][direction_y]->slopeangle = max_max_up_angle;

                    if(is_print_message_) cout<<" indX: "<<indX<<" indY: "<<indY<<" max_max_up_angle: "<<max_max_up_angle<<endl;

                }else if(up_down_flag == 2){
                    GridCloudNodeMap[indX][indY]->slopeangle = max_max_down_angle;
                    GridCloudNodeMap[direction_x][direction_y]->slopeangle = max_max_up_angle;

                    if(is_print_message_) cout<<" indX: "<<indX<<" indY: "<<indY<<" max_max_down_angle: "<<max_max_down_angle<<endl;

                }

            }

        }
        else if(valid_count == total_step && isslope == false){
            if((up_down_flag ==1 && max_up_angle > slope_angle_thr && CheckUpSlope) || (up_down_flag ==2 && max_down_angle < -slope_angle_thr && CheckDownSlope)){
//                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                GridCloudNodeMap[indX][indY]->slopeflag = 1;
                GridCloudNodeMap[direction_x][direction_y]->slopeflag = 1;
                if(up_down_flag == 1){
                    GridCloudNodeMap[indX][indY]->slopeangle = max_up_angle;
                    GridCloudNodeMap[direction_x][direction_y]->slopeangle = max_up_angle;

                    if(is_print_message_) cout<<" indX: "<<indX<<" indY: "<<indY<<" max_up_angle: "<<max_up_angle<<endl;

                }else if(up_down_flag == 2){
                    GridCloudNodeMap[indX][indY]->slopeangle = max_down_angle;
                    GridCloudNodeMap[direction_x][direction_y]->slopeangle = max_down_angle;

                    if(is_print_message_) cout<<" indX: "<<indX<<" indY: "<<indY<<" max_down_angle: "<<max_down_angle<<endl;

                }
                return;
            }
        }
    }
}






void ObjSegGrid_obj::SlopeCheck(int indX,int indY) {






    //坡度检查,检测水渠边缘处
    //todo SlopeCheck增加二次检查栅格点为0的时候，如果直接在一定范围内，并且不是雷达打不到的左下或者右下点的时候直接认为是边界栅格
    if( indY < GridCloudHalfWidth + 2*GridSizeInverse){
        int interval = 2;


            if(UseLidar){


            if(GridCloudNodeMap[indX][indY+interval]->Cloud->points.size()>= 1 &&
               GridCloudNodeMap[indX-interval][indY+interval]->Cloud->points.size()>= 1 &&
               GridCloudNodeMap[indX+interval][indY+interval]->Cloud->points.size()>= 1 &&
               GridCloudNodeMap[indX-interval][indY]->Cloud->points.size()>= 1 &&
               GridCloudNodeMap[indX+interval][indY]->Cloud->points.size()>= 1
               && GridCloudNodeMap[indX][indY]->Cloud->points.size()>= 1
                    ){

                //前方
                float y = sqrt((GridCloudNodeMap[indX][indY + interval]->base_pt.y -
                                GridCloudNodeMap[indX][indY]->base_pt.y) *
                               (GridCloudNodeMap[indX][indY + interval]->base_pt.y -
                                GridCloudNodeMap[indX][indY]->base_pt.y)
                               + (GridCloudNodeMap[indX][indY + interval]->base_pt.x -
                                  GridCloudNodeMap[indX][indY]->base_pt.x) *
                                 (GridCloudNodeMap[indX][indY + interval]->base_pt.x -
                                  GridCloudNodeMap[indX][indY]->base_pt.x));
                float z = GridCloudNodeMap[indX][indY+interval]->base_pt.z - GridCloudNodeMap[indX][indY]->base_pt.z;
                //左前
                float yfl = sqrt((GridCloudNodeMap[indX-interval][indY+interval]->base_pt.y -
                                  GridCloudNodeMap[indX][indY]->base_pt.y) *
                                 (GridCloudNodeMap[indX-interval][indY+interval]->base_pt.y -
                                  GridCloudNodeMap[indX][indY]->base_pt.y)
                                 + (GridCloudNodeMap[indX-interval][indY+interval]->base_pt.x -
                                    GridCloudNodeMap[indX][indY]->base_pt.x) *
                                   (GridCloudNodeMap[indX-interval][indY+interval]->base_pt.x -
                                    GridCloudNodeMap[indX][indY]->base_pt.x));
                float zfl = GridCloudNodeMap[indX-interval][indY+interval]->base_pt.z - GridCloudNodeMap[indX][indY]->base_pt.z;
                //右前
                float yfr = sqrt((GridCloudNodeMap[indX+interval][indY+interval]->base_pt.y -
                                  GridCloudNodeMap[indX][indY]->base_pt.y) *
                                 (GridCloudNodeMap[indX+interval][indY+interval]->base_pt.y -
                                  GridCloudNodeMap[indX][indY]->base_pt.y)
                                 + (GridCloudNodeMap[indX+interval][indY+interval]->base_pt.x -
                                    GridCloudNodeMap[indX][indY]->base_pt.x) *
                                   (GridCloudNodeMap[indX+interval][indY+interval]->base_pt.x -
                                    GridCloudNodeMap[indX][indY]->base_pt.x));
                float zfr = GridCloudNodeMap[indX+interval][indY+interval]->base_pt.z - GridCloudNodeMap[indX][indY]->base_pt.z;
                //左
                float yl = sqrt((GridCloudNodeMap[indX-interval][indY]->base_pt.y -
                                 GridCloudNodeMap[indX][indY]->base_pt.y) *
                                (GridCloudNodeMap[indX-interval][indY]->base_pt.y -
                                 GridCloudNodeMap[indX][indY]->base_pt.y)
                                + (GridCloudNodeMap[indX-interval][indY]->base_pt.x -
                                   GridCloudNodeMap[indX][indY]->base_pt.x) *
                                  (GridCloudNodeMap[indX-interval][indY]->base_pt.x -
                                   GridCloudNodeMap[indX][indY]->base_pt.x));

                float zl = GridCloudNodeMap[indX-interval][indY]->base_pt.z - GridCloudNodeMap[indX][indY]->base_pt.z;
                //右
                float yr = sqrt((GridCloudNodeMap[indX+interval][indY]->base_pt.y -
                                 GridCloudNodeMap[indX][indY]->base_pt.y) *
                                (GridCloudNodeMap[indX+interval][indY]->base_pt.y -
                                 GridCloudNodeMap[indX][indY]->base_pt.y)
                                + (GridCloudNodeMap[indX+interval][indY]->base_pt.x -
                                   GridCloudNodeMap[indX][indY]->base_pt.x) *
                                  (GridCloudNodeMap[indX+interval][indY]->base_pt.x -
                                   GridCloudNodeMap[indX][indY]->base_pt.x));
                float zr = GridCloudNodeMap[indX+interval][indY]->base_pt.z - GridCloudNodeMap[indX][indY]->base_pt.z;

                float min_z = 0.267*GridSize*(interval-1);


                float thr = atan2(-z,y);
                float thrfl = atan2(-zfl,yfl);
                float thrfr = atan2(-zfr,yfr);
                float thrl = atan2(-zl,yl);
                float thrr = atan2(-zr,yr);

                //不检测内侧斜坡
                if(indX < 110){
                    thrr = 0;
                    zr = 0;
                }
                if(indX > 90){
                    thrl = 0;
                    zl = 0;
                }

                float maxthr = thr > thrl ? thr:thrl;
                maxthr = maxthr > thrr ? maxthr : thrr;
                maxthr = maxthr > thrfr ? maxthr : thrfr;
                maxthr = maxthr > thrr ? maxthr : thrr;



//                if((z< -min_z && zfl < -min_z && zfr< -min_z) ||
//                   (zl< -min_z && zfl < -min_z && z< -min_z) ||
//                   (z< -min_z && zfr < -min_z && zr< -min_z) ){

//                if ((zl < -min_z && zfl < -min_z) ||
//                    (zfl < -min_z && z < -min_z) ||
//                    (z < -min_z && zfr < -min_z) ||
//                    (zfr < -min_z && zr < -min_z)) {

                if ((zl < -min_z) ||
                    (zfl < -min_z) ||
                    (z < -min_z )||
                     (zfr < -min_z) ||
                     (zr < -min_z)) {


                    //z<0 下坡，可能是凹坑


//                    cout<<"indX: "<<indX<<" indY: "<<indY<<" thr: "<<thr<<endl;
                    if(maxthr > 0.3){
                        //斜坡大于30度0.577，25度0.466,20度0.363 ,15度0.267,10度0.176

                        ///2次检查判断是否是小凸起
                        //todo 2次检查是否是水渠或者下坡
//                        int _indX,_indY;
//                        if(maxthr == thr){
//                            _indX = indX;
//                            _indY = indY + 2*interval;
//                        }else if(maxthr == thrfl){
//                            _indX = indX-2*interval;
//                            _indY = indY+2*interval;
//                        }else if(maxthr == thrfr){
//                            _indX = indX + 2 * interval;
//                            _indY = indY+2*interval;
//                        }else if(maxthr == thrr){
//                            _indX = indX + 2 * interval;
//                            _indY = indY;
//                        }else if(maxthr == thrl){
//                            _indX = indX-2*interval;
//                            _indY = indY;
//                        }


                        if(maxthr == thr){
                            if(GridCloudNodeMap[indX][indY+2*interval]->Cloud->points.size()>= 1){
                                float z_ = GridCloudNodeMap[indX][indY+2*interval]->base_pt.z - GridCloudNodeMap[indX][indY]->base_pt.z;
                                if(z_ < 2*z){
                                    GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                    if (is_print_message_) cout<<"SlopeCheck thr object indX: "<<indX<<" indY: "<<indY<<endl;

                                }
                            }else if(GridCloudNodeMap[indX][indY+2*interval]->Cloud->points.size() == 0){
                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"SlopeCheck thr object indX: "<<indX<<" indY: "<<indY<<endl;
                            }
                        }else if(maxthr == thrfl){
                            if(GridCloudNodeMap[indX-2*interval][indY+2*interval]->Cloud->points.size()>= 1){
                                float z_ = GridCloudNodeMap[indX-2*interval][indY+2*interval]->base_pt.z - GridCloudNodeMap[indX][indY]->base_pt.z;
                                if(z_ < 2*z){
                                    GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                    if (is_print_message_) cout<<"SlopeCheck thrfl object indX: "<<indX<<" indY: "<<indY<<endl;

                                }
                            }else if(GridCloudNodeMap[indX-2*interval][indY+2*interval]->Cloud->points.size() == 0){
                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"SlopeCheck thr object indX: "<<indX<<" indY: "<<indY<<endl;
                            }

                        }else if(maxthr == thrfr) {
                            if (GridCloudNodeMap[indX + 2 * interval][indY +2 * interval]->Cloud->points.size() >= 1) {
                                float z_ =GridCloudNodeMap[indX + 2 * interval][indY + 2 * interval]->base_pt.z -GridCloudNodeMap[indX][indY]->base_pt.z;
                                if (z_ < 2 * z) {
                                    GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                    if (is_print_message_) cout<<"SlopeCheck thrfr object indX: "<<indX<<" indY: "<<indY<<endl;

                                }
                            }else if(GridCloudNodeMap[indX + 2 * interval][indY +2 * interval]->Cloud->points.size() == 0){
                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"SlopeCheck thr object indX: "<<indX<<" indY: "<<indY<<endl;
                            }
                        }else if(maxthr == thrr) {
                            if (GridCloudNodeMap[indX + 2 * interval][indY]->Cloud->points.size() >= 1) {
                                float z_ =GridCloudNodeMap[indX + 2 * interval][indY]->base_pt.z -GridCloudNodeMap[indX][indY]->base_pt.z;
                                if (z_ < 2 * z) {
                                    GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                    if (is_print_message_) cout<<"SlopeCheck thrr object indX: "<<indX<<" indY: "<<indY<<endl;

                                }
                            }else if(GridCloudNodeMap[indX + 2 * interval][indY]->Cloud->points.size() == 0){
                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"SlopeCheck thr object indX: "<<indX<<" indY: "<<indY<<endl;
                            }

                        }else if(maxthr == thrl) {
                            if (GridCloudNodeMap[indX - 2 * interval][indY]->Cloud->points.size() >= 1) {
                                float z_ =GridCloudNodeMap[indX - 2 * interval][indY]->base_pt.z -GridCloudNodeMap[indX][indY]->base_pt.z;
                                if (z_ < 2 * z) {
                                    GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                    if (is_print_message_) cout<<"SlopeCheck thrl object indX: "<<indX<<" indY: "<<indY<<endl;

                                }
                            }else if(GridCloudNodeMap[indX - 2 * interval][indY]->Cloud->points.size() == 0){
                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"SlopeCheck thr object indX: "<<indX<<" indY: "<<indY<<endl;
                            }

                        }

//                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;


//                                for(int i = 0;i<=interval;i++) {
//                                    GridCloudNodeMap[indX][indY + i]->cluster_flag = 1;
//                                }
//                                    GridCloudNodeMap[indX][indY]->cluster_flag = 1;


//                        if(GridCloudNodeMap[indX][indY]->cluster_flag = 1){
//                            cout<<"indX: "<<indX<<" indY: "<<indY<<" y:"<<y<<" z:"<<z<<" yl:"<<yl<<" zl:"<<zl<<" yr:"<<yr<<" zr:"<<zr<<" yfl:"<<yfl<<" zfl:"<<zfl<<" yfr:"<<yfr<<" zfr:"<<zfr<<" min_z: "<<min_z<<endl;
//                            cout<<"thr: "<<thr<<" thrl:"<<thrl<<" thrr: "<<thrr<<" thrfl:"<<thrfl<<" thrfr:"<<thrfr<<" maxthr :"<<maxthr<<endl;
//                            cout<<"GridCloudNodeMap[indX-interval][indY]->base_pt.y "<<GridCloudNodeMap[indX-interval][indY]->base_pt.y<<endl;
//                            cout<<"GridCloudNodeMap[indX][indY]->base_pt.y "<<GridCloudNodeMap[indX][indY]->base_pt.y<<endl;
//                        }
//

                    }
                }
            }
        }
    }

}

void ObjSegGrid_obj::EmptyHoleBFS(){

    if(isslope){
        return;
    }

    float minYAxis,maxYAxis,minXAxis,maxXAxis,max_farAxis;
//    float  = 2.5;

    if(UseLidar){
        minYAxis = MinCarDis;
        maxYAxis = 2.1;
        max_farAxis = 3.5;
        minXAxis = -lidarXAxis;
        maxXAxis = lidarXAxis;
    }else if(UseDCamera){
        minYAxis =MinCarDis;
        maxYAxis = 2.1;
        max_farAxis = 3.5;
        minXAxis = -DCameraXAxis;
        maxXAxis = DCameraXAxis;
    }
    //栅格BFS聚类

    pcl::PointXYZI thispoint;
    bool last_cluster = false;
    int minEmptyGridNum = 4*4;
    int farhole_minEmptyGridNum = 4*4;
    int minpointpergrid = 1; //0 or 1
    set<pair<int, int>> nodelneighbour;
    set<pair<int, int>> neighbour;
    bool hole_is_far = false;
    float dis,max_dis;
    float autoEmptyGridNum;
    float temp_theta = 0.4375/180*M_PI;

//    map<int,vector<pair<int, int>>> AllEmptyneighbour;
//    vector<pair<int, int>> neighbour;
    AllEmptyneighbour.clear();
    for (int indX = GridCloudHalfWidth + minXAxis * GridSizeInverse;
         indX <= GridCloudHalfWidth + maxXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth + minYAxis * GridSizeInverse ;
             indY <= GridCloudHalfWidth + max_farAxis * GridSizeInverse; indY++) {


            if(UseLidar){
                //激光雷达左下角和右下角的两个空区域不要
                //左区域直线方程 y=-x+193 右区域 y=x - 6

                int thr_y1 = -indX +193;
                int thr_y2 = indX -6;

                if( ( indY < thr_y1 ) || (indY < thr_y2) ){
                    continue;
                }
            }else if(UseDCamera){
                //深度相机左下角和右下角的两个空区域不要
                //左区域直线方程 y=-(3/2)x+248 右区域 y=(3/2)x -50.5

                int thr_y1 = -1.5*indX +248;
                int thr_y2 = 1.5*indX -50.5;

                if( ( indY < thr_y1 ) || (indY < thr_y2) ){
                    continue;
                }

            }

            nodelneighbour.clear();
            neighbour.clear();
            hole_is_far = false;
            max_dis = 0;
            dis = sqrt(0.01*((indX - GridCloudHalfWidth)*(indX - GridCloudHalfWidth) + (indY - GridCloudHalfWidth)*(indY - GridCloudHalfWidth)));

            float theta_a = atan2(SensorHigh,dis);
//            cout<<"SensorHigh: "<<SensorHigh<<" dis: "<<dis<<" theta_a:"<<theta_a<<" temp_theta:"<<temp_theta<<endl;
            float theta_b = theta_a - temp_theta;
            float dis_resolution_y = SensorHigh/tan(theta_b) - dis;
            float dis_resolution_x = dis*temp_theta;
            if(dis > 1.5){
                minpointpergrid =  floor(2*GridSize/(dis_resolution_y));
            }else{
                minpointpergrid =  floor(GridSize/(dis_resolution_y));
            }

            //minpointpergrid 大于一定值则不认为是空栅格了
            if(minpointpergrid > 1 && UseDCamera){
                minpointpergrid = 1;
            }else if(minpointpergrid > 1 && UseLidar) {
                minpointpergrid = 1;
            }
//            cout<<"dis : "<<dis<<" theta_a: "<<theta_a<<" theta_b: "<<theta_b<<" dis_resolution_x: "<<dis_resolution_x<<" dis_resolution_y: "<<dis_resolution_y<<" minpointpergrid:"<<minpointpergrid<<endl;

//            if ((GridCloudNodeMap[indX][indY]->Cloud->points.size() <= minpointpergrid &&
//                    dis <= 2.2)
//                || (GridCloudNodeMap[indX][indY]->Cloud->points.size() <= 0 &&
//                    dis > 2.2 &&
//                    dis <= maxYAxis)) {

            if ((GridCloudNodeMap[indX][indY]->Cloud->points.size() <= minpointpergrid &&
                    dis <= maxYAxis)) {


                if(GridCloudNodeMap[indX][indY]->ishole ==0) {
                    dis = sqrt(0.01*((indX - GridCloudHalfWidth)*(indX - GridCloudHalfWidth) + (indY - GridCloudHalfWidth)*(indY - GridCloudHalfWidth)));
//                    cout<<"indX: "<<indX<<" indY: "<<indY<<" dis: "<<dis<<endl;
                    if(dis > max_dis){max_dis = dis;}

                    // 存入当前ID到neighbour
                    neighbour.insert(pair<int, int>(indX, indY));
                    nodelneighbour.insert(pair<int, int>(indX, indY));
                    GridCloudNodeMap[indX][indY]->ishole = 1;
                    GridCloudNodeMap[indX][indY]->cluster_flag = clusterlabel;
                }
            } else if (GridCloudNodeMap[indX][indY]->Cloud->points.size() <= 0 &&
                    dis <=  max_farAxis  &&
                    dis > maxYAxis ) {

//                bool ishole = true;
                //大于2.5m的栅格判断,当一个3×3的栅格都为空栅格的时候，才把这3×3的栅格都当作凹坑空栅格
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        // 如果这个栅格不等于1，即等于0或者2，即没有障碍物或研究遍历过。
                        // 或者就是当前这个栅格
                        // 或者待选的栅格过界
                        // 则跳过这个栅格
                        if (
                                indX + dx < GridCloudHalfWidth + minXAxis * GridSizeInverse ||
                                indX + dx > GridCloudHalfWidth + maxXAxis * GridSizeInverse ||
                                indY + dy > GridCloudHalfWidth + max_farAxis * GridSizeInverse ||
                                indY + dy < GridCloudHalfWidth + minYAxis * GridSizeInverse ||
                                GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points.size() > 0) {
//                            ishole = false;
                            GridCloudNodeMap[indX][indY]->ishole = 2;
                            break;
                        }
                    }
                }

                if (GridCloudNodeMap[indX][indY]->ishole == 0 || GridCloudNodeMap[indX][indY]->ishole == 1) {
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {

                            if(UseLidar){
                                //激光雷达左下角和右下角的两个空区域不要
                                //左区域直线方程 y=-x+193 右区域 y=x - 6

                                int thr_y1 = -(indX + dx) +193;
                                int thr_y2 = indX + dx -6;

                                if( ( indY + dy < thr_y1 ) || (indY + dy < thr_y2) ){
                                    continue;
                                }
                            }else if(UseDCamera){
                                //深度相机左下角和右下角的两个空区域不要
                                //左区域直线方程 y=-(3/2)x+248 右区域 y=(3/2)x -50.5

                                int thr_y1 = -1.5*(indX + dx) +248;
                                int thr_y2 = 1.5*(indX + dx) -50.5;

                                if( ( indY + dy < thr_y1 ) || (indY + dy < thr_y2) ){
                                    continue;
                                }

                            }

                            if(GridCloudNodeMap[indX + dx][indY + dy]->ishole ==0){

                                dis = sqrt(0.01*((indX + dx - GridCloudHalfWidth)*(indX + dx - GridCloudHalfWidth) + (indY + dy - GridCloudHalfWidth)*(indY + dy - GridCloudHalfWidth)));
//                                cout<<"indX + dx: "<<indX + dx<<" indY + dy: "<<indY + dy<<" dis: "<<dis<<endl;
                                if(dis > max_dis){max_dis = dis;}

                                hole_is_far =true;
                                neighbour.insert(pair<int, int>(indX + dx, indY + dy));
    //                                cout<<"indX+dx: "<<indX+dx<<"indY +dy: "<<indY + dy<<" neighbour size:"<<neighbour.size()<<" clusterlabel:"<<clusterlabel<<endl;
                                nodelneighbour.insert(pair<int, int>(indX + dx, indY + dy));
                                GridCloudNodeMap[indX + dx][indY + dy]->ishole = 1;
                                GridCloudNodeMap[indX + dx][indY + dy]->cluster_flag = clusterlabel;
                                }
                        }
                    }
                }
            }

            while (!neighbour.empty()) {
                // 提取出这个栅格
//                    pair<int, int> thisgrid = neighbour.begin()->first;
//                    neighbour.pop_back();
                int indx = neighbour.begin()->first;
                int indy = neighbour.begin()->second;
                neighbour.erase(pair<int, int>(indx, indy));


                if (UseLidar) {
                    //激光雷达左下角和右下角的两个空区域不要
                    //左区域直线方程 y=-x+193 右区域 y=x - 6

                    int thr_y1 = -indx + 193;
                    int thr_y2 = indx - 6;

                    if ((indy < thr_y1) || (indy < thr_y2)) {
                        continue;
                    }
                } else if (UseDCamera) {
                    //深度相机左下角和右下角的两个空区域不要
                    //左区域直线方程 y=-(3/2)x+250 右区域 y=(3/2)x -50.5

                    int thr_y1 = -1.5 * indx + 250;
                    int thr_y2 = 1.5 * indx - 50.5;

                    if ((indy < thr_y1) || (indy < thr_y2)) {
                        continue;
                    }

                }

                dis = sqrt(0.01*((indx - GridCloudHalfWidth)*(indx - GridCloudHalfWidth) + (indy - GridCloudHalfWidth)*(indy - GridCloudHalfWidth)));

                float theta_a = atan2(SensorHigh,dis);
                float theta_b = theta_a - temp_theta;
                float dis_resolution_y = SensorHigh/tan(theta_b) - dis;
                float dis_resolution_x = dis*temp_theta;
                minpointpergrid =  floor(3* GridSize/(dis_resolution_y));
                if(dis > 1.5){
                    minpointpergrid =  floor(2*GridSize/(dis_resolution_y));
                }else{
                    minpointpergrid =  floor(GridSize/(dis_resolution_y));
                }

                //minpointpergrid 大于一定值则不认为是空栅格了
                if(minpointpergrid > 1 && UseDCamera){
                    minpointpergrid = 1;
                }else if(minpointpergrid > 1 && UseLidar) {
                    minpointpergrid = 1;
                }

//                cout<<"dis : "<<dis<<" theta_a: "<<theta_a<<" theta_b: "<<theta_b<<" dis_resolution_x: "<<dis_resolution_x<<" dis_resolution_y: "<<dis_resolution_y<<" minpointpergrid:"<<minpointpergrid<<endl;


//                if ((GridCloudNodeMap[indx][indy]->Cloud->points.size() <= minpointpergrid &&
//                        dis <= 2.2)
//                    || (GridCloudNodeMap[indx][indy]->Cloud->points.size() <= 0 &&
//                        dis > 2.2 &&
//                        dis <= maxXAxis)) {
                if ((GridCloudNodeMap[indX][indY]->Cloud->points.size() <= minpointpergrid &&
                     dis <= maxYAxis)) {
                    // 查找前后左右的8个栅格
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            // 如果这个栅格不等于1，即等于0或者2，即没有障碍物或研究遍历过。
                            // 或者就是当前这个栅格
                            // 或者待选的栅格过界
                            // 则跳过这个栅格
                            if ((dx == 0 && dy == 0) ||
                                indx + dx < GridCloudHalfWidth + minXAxis * GridSizeInverse ||
                                indx + dx > GridCloudHalfWidth + maxXAxis * GridSizeInverse ||
                                indy + dy > GridCloudHalfWidth + max_farAxis * GridSizeInverse ||
                                indy + dy < GridCloudHalfWidth + minYAxis * GridSizeInverse + 1 ||
                                GridCloudNodeMap[indx + dx][indy + dy]->Cloud->points.size() > minpointpergrid)
                                continue;
                            // 否则将这个栅格加入待查找列表
//                cout<<"indx+dx:"<<indx+dx<<"indy+dy:"<<indy+dy<<"clusterflag:"<<GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag<<endl;

                            if (GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag > 0) {
                                continue;
                            }

                            if(UseLidar){
                                //激光雷达左下角和右下角的两个空区域不要
                                //左区域直线方程 y=-x+193 右区域 y=x - 6

                                int thr_y1 = -(indx + dx) +193;
                                int thr_y2 = indx + dx -6;

                                if( ( indy + dy < thr_y1 ) || (indy + dy < thr_y2) ){
                                    continue;
                                }
                            }else if(UseDCamera){
                                //深度相机左下角和右下角的两个空区域不要
                                //左区域直线方程 y=-(3/2)x+248 右区域 y=(3/2)x -50.5

                                int thr_y1 = -1.5*(indx + dx) +249;
                                int thr_y2 = 1.5*(indx + dx) -50.5;

                                if( ( indy + dy < thr_y1 ) || (indy + dy < thr_y2) ){
                                    continue;
                                }

                            }


                            //分段点数限制
//                            if ((GridCloudNodeMap[indx + dx][indy + dy]->Cloud->points.size() <= minpointpergrid &&
//                                    dis <= 2.2)
//                                || (GridCloudNodeMap[indx + dx][indy + dy]->Cloud->points.size() <= 0 &&
//                                    dis > 2.2 &&
//                                    dis <= max_farAxis)) {

                                if(GridCloudNodeMap[indx + dx][indy + dy]->ishole ==0) {

                                    dis = sqrt(0.01*((indx + dx - GridCloudHalfWidth)*(indx + dx - GridCloudHalfWidth) + (indy + dy - GridCloudHalfWidth)*(indy + dy - GridCloudHalfWidth)));
//                                    cout<<"indx + dx: "<<indx + dx<<" indy + dy: "<<indy + dy<<" dis: "<<dis<<endl;

                                    if(dis > max_dis){max_dis = dis;}

                                    neighbour.insert(pair<int, int>(indx + dx, indy + dy));
//                            cout<<"near indX+dx: "<<indX+dx<<"indY +dy: "<<indY + dy<<" neighbour size:"<<neighbour.size()<<endl;

                                    nodelneighbour.insert(pair<int, int>(indx + dx, indy + dy));

//                            cout<<"neighbour size: "<<neighbour.size()<<endl;
                                    GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = clusterlabel;
                                    GridCloudNodeMap[indx + dx][indy + dy]->ishole = 1;

                                }
//                            }
                        }

                    }

                } else if (GridCloudNodeMap[indx][indy]->Cloud->points.size() <= 0 &&
                        dis > maxYAxis &&
                        dis <= max_farAxis) {

                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            // 如果这个栅格不等于1，即等于0或者2，即没有障碍物或研究遍历过。
                            // 或者就是当前这个栅格
                            // 或者待选的栅格过界
                            // 则跳过这个栅格
                            if (
                                    indx + dx < GridCloudHalfWidth + minXAxis * GridSizeInverse ||
                                    indx + dx > GridCloudHalfWidth + maxXAxis * GridSizeInverse ||
                                    indy + dy > GridCloudHalfWidth + max_farAxis * GridSizeInverse ||
                                    indy + dy < GridCloudHalfWidth + minYAxis * GridSizeInverse ||
                                    GridCloudNodeMap[indx + dx][indy + dy]->Cloud->points.size() > 0) {
                                GridCloudNodeMap[indx][indy]->ishole = 2;
                                break;
                            }
                        }
                    }

                    if (GridCloudNodeMap[indx][indy]->ishole == 0 || GridCloudNodeMap[indx][indy]->ishole == 1) {
                        for (int dx = -1; dx <= 1; dx++) {
                            for (int dy = -1; dy <= 1; dy++) {

                                if(UseLidar){
                                    //激光雷达左下角和右下角的两个空区域不要
                                    //左区域直线方程 y=-x+193 右区域 y=x - 6

                                    int thr_y1 = -(indx + dx) +193;
                                    int thr_y2 = indx + dx -6;

                                    if( ( indy + dy < thr_y1 ) || (indy + dy < thr_y2) ){
                                        continue;
                                    }
                                }else if(UseDCamera){
                                    //深度相机左下角和右下角的两个空区域不要
                                    //左区域直线方程 y=-(3/2)x+248 右区域 y=(3/2)x -50.5

                                    int thr_y1 = -1.5*(indx + dx) +250;
                                    int thr_y2 = 1.5*(indx + dx) -50.5;

                                    if( ( indy + dy < thr_y1 ) || (indy + dy < thr_y2) ){
                                        continue;
                                    }

                                }

                                if (GridCloudNodeMap[indx + dx][indy + dy]->ishole == 0) {
                                    dis = sqrt(0.01*((indx + dx - GridCloudHalfWidth)*(indx + dx - GridCloudHalfWidth) + (indy + dy - GridCloudHalfWidth)*(indy + dy - GridCloudHalfWidth)));
//                                    cout<<"indx + dx: "<<indx + dx<<" indy + dy: "<<indy + dy<<" dis: "<<dis<<endl;

                                    if(dis > max_dis){max_dis = dis;}

                                    hole_is_far =true;
                                    neighbour.insert(pair<int, int>(indx + dx, indy + dy));
                                    nodelneighbour.insert(pair<int, int>(indx + dx, indy + dy));
                                    GridCloudNodeMap[indx + dx][indy + dy]->ishole = 1;
                                    GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = clusterlabel;

                                }
                            }
                        }
                    }
                }
            }
            autoEmptyGridNum = 1 + 2*max_dis*max_dis;
            if(autoEmptyGridNum < 3*3 && UseDCamera){
                autoEmptyGridNum = 3*3;
            }
//            autoEmptyGridNum = max_dis;

//            if(autoEmptyGridNum > 20){autoEmptyGridNum = 20;}
//            if ((nodelneighbour.size() >= minEmptyGridNum && hole_is_far == false) ||
//                (nodelneighbour.size() >= farhole_minEmptyGridNum && hole_is_far == true)) {

            if (nodelneighbour.size() >= autoEmptyGridNum) {
//                cout<<"max_dis: "<<max_dis<<" autoEmptyGridNum: "<<autoEmptyGridNum<<endl;


                if (hole_is_far == false) {
                    if (is_print_message_) cout << "EmptyLabel: " << clusterlabel << " gridsize: " << nodelneighbour.size() << endl;
                } else {
                    if (is_print_message_) cout << "EmptyLabel: " << clusterlabel << " far hole gridsize: " << nodelneighbour.size()
                         << endl;

                }
                AllEmptyneighbour[clusterlabel] = nodelneighbour;
                clusterlabel++;

            }
            else{
                while (!nodelneighbour.empty()) {
                    // 将之前聚类的空栅格重新变成0
                    int indx = nodelneighbour.begin()->first;
                    int indy = nodelneighbour.begin()->second;
                    nodelneighbour.erase(pair<int, int>(indx, indy));
//                    cout << "cluster fail indx: " << indx << " indy:" << indy << "EmptyLabel: " << clusterlabel
//                         << " gridsize: " << nodelneighbour.size() << endl;

//                        int indx = thisgrid.first;
//                        int indy = thisgrid.second;
                    GridCloudNodeMap[indx][indy]->cluster_flag = 0;

                }

            }

        }
    }


}
void ObjSegGrid_obj::SlopeBFS() {

    set<pair<int, int>> nodelneighbour;
    vector<pair<int, int>> neighbour;
    int min_slope_area_count = 100; //100格子为1m^2
    bool slopeangle = 0;

    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = (GridCloudWidth - 1) / 2 - 10; indY < GridCloudWidth; indY++) {

            // 如果flag==1（有障碍物）
            if (GridCloudNodeMap[indX][indY]->slopeflag > 0 && GridCloudNodeMap[indX][indY]->cluster_flag == 0) {
                // 存入当前ID到neighbour
                nodelneighbour.clear();
                neighbour.clear();
                neighbour.emplace_back(pair<int, int>(indX, indY));
                nodelneighbour.insert(pair<int, int>(indX, indY));
                slopeangle = GridCloudNodeMap[indX][indY]->slopeangle;

                if(isslope){
                    GridCloudNodeMap[indX][indY]->cluster_flag = clusterlabel;

                }else{
                    GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                }

                int slope_area_count = 1;

                // 如果不为空
                while (!neighbour.empty()) {
                    // 提取出这个栅格
                    pair<int, int> thisgrid = neighbour.back();
                    int indx = thisgrid.first;
                    int indy = thisgrid.second;
                    neighbour.pop_back();

                    // 查找前后左右的8个栅格
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            // 如果这个栅格不等于1，即等于0或者2，即没有障碍物或研究遍历过。
                            // 或者就是当前这个栅格
                            // 或者待选的栅格过界
                            // 则跳过这个栅格
                            if ((dx == 0 && dy == 0) ||
                                indx + dx < GridCloudHalfWidth - lidarXAxis * GridSizeInverse ||
                                indx + dx > GridCloudHalfWidth + lidarXAxis * GridSizeInverse ||
                                indy + dy < (GridCloudWidth - 1) / 2 ||
                                indy + dy > GridCloudWidth - 1 ){
                                continue;
                            }
//                            if(GridCloudNodeMap[indx][indy]->slopeflag == 1 && GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag == 0){
//
//                            }
                            // 否则将这个栅格加入待查找列表
                            if (GridCloudNodeMap[indx + dx][indy + dy]->slopeflag > 0 && GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag == 0
                                && ((slopeangle > 0 && GridCloudNodeMap[indx + dx][indy + dy]->slopeangle>0) || (slopeangle < 0 && GridCloudNodeMap[indx + dx][indy + dy]->slopeangle<0))) {
                                neighbour.emplace_back(pair<int, int>(indx + dx, indy + dy));
                                nodelneighbour.insert(pair<int, int>(indx + dx, indy + dy));
                                if(isslope){
                                    GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = clusterlabel;

                                }else{
                                    GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = 1;
                                }
                                slope_area_count++;
                            }
                        }
                    }
                }
//                cout<<"nodelneighbour.size() "<<nodelneighbour.size()<<endl;

                //斜坡面积没有达到要求
                if (nodelneighbour.size() < min_slope_area_count) {
//                    cout<<"nodelneighbour.size() "<<nodelneighbour.size()<<endl;
                    while (!nodelneighbour.empty()) {
                        // 将之前聚类的空栅格重新变成0
                        int indx = nodelneighbour.begin()->first;
                        int indy = nodelneighbour.begin()->second;
                        nodelneighbour.erase(pair<int, int>(indx, indy));
                        GridCloudNodeMap[indx][indy]->cluster_flag = 0;
                        GridCloudNodeMap[indx][indy]->slopeflag = 0;
//                        cout<<"nodelneighbour indx "<<indx<<"indy "<<indy<<endl;

                    }
                }else{
                    if(isslope){
                        cout<<"clusterlabel: "<<clusterlabel<<"nodelneighbour.size() "<<nodelneighbour.size()<<endl;
                        slopeclusterflag.insert(clusterlabel);
                        clusterlabel++;
                    }
                }

            }
        }
    }
}
void ObjSegGrid_obj::ComputeGridFeature2() {

    clusterlabel = 2; // 聚类后的flag，会递增，一个类别用一个标签

//    if(isslope==false){
//        for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
//             indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
//            for (int indY = (GridCloudWidth - 1) / 2; indY < GridCloudWidth; indY++) {
//                //            SlopeCheck(indX,indY);
//
//                SlopeCheckv2(indX, indY);
//            }
//        }
//        SlopeBFS();
//    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 遍历每个栅格
    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth - lidarYAxis * GridSizeInverse;
             indY < GridCloudHalfWidth + lidarYAxis * GridSizeInverse; indY++) {




            //未满足点数要求的均设置平坦度为1,防止误检
//            GridCloudNodeMap[indX][indY]->ground_z_vec = 1;

            // 如果栅格内点云大于最低点云要求
            if (GridCloudNodeMap[indX][indY]->Cloud->points.size() > minGridCloudNum ||
                GridCloudNodeMap[indX][indY]->ground_z_elevation > grasshigh_mid) {
//                cout << "indX: " << indX << "indY: " << indY << "cloudsize: "
//                     << GridCloudNodeMap[indX][indY]->Cloud->points.size() << " ground_z_elev: "
//                     << GridCloudNodeMap[indX][indY]->ground_z_elevation << endl;
                tempCloud->clear();
                int invalid_count = 0;
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        // 如果这个栅格不等于1，即等于0或者2，即没有障碍物或研究遍历过。
                        // 或者就是当前这个栅格
                        // 或者待选的栅格过界
                        // 则跳过这个栅格
                        if (indX + dx < 0 || indX + dx > GridCloudWidth - 1 || indY + dy > GridCloudWidth - 1 ||
                            indY + dy < GridCloudHalfWidth)
                            continue;
                        if (GridCloudNodeMap[indX + dx][indY + dy]->Cloud->points.size() == 0)
                            invalid_count++;

//                          *tempCloud += *GridCloudNodeMap[indX + dx][indY + dy]->Cloud;
                    }
                }
                GridCloudNodeMap[indX][indY]->invalid_count = invalid_count;


                // 提取栅格内的地面和非地面
                extract_piecewiseground(*GridCloudNodeMap[indX][indY]->Cloud, *ground_cloud);
                // 地面点的法向量
                const double ground_z_vec = abs(normal_(2, 0));
                GridCloudNodeMap[indX][indY]->ground_z_vec = ground_z_vec;


                //如果栅格在点云边界处 todo 之后考虑解决这个补丁



                ComputeScorev2(indX, indY);

                //如果在水泥地或者无高草草地上
                if (plane == true && isslope == false) {
                    int bias = 2 * GridSizeInverse;//TODO 之后考虑修补这个问题
                    if (UseDCamera)
                        bias = 2.5 * GridSizeInverse;
                    if (indY < GridCloudHalfWidth + bias) {
                        if (GridCloudNodeMap[indX][indY]->Cloud->points.size() > minGridCloudNum &&
                            ground_z_vec < uprightness_thr_ && invalid_count < 3 && GridCloudNodeMap[indX][indY]->ground_z_elevation > 0.02) {
                            //如果栅格在点云边界处 todo 之后考虑解决这个补丁
//                            cout << "indX: " << indX << "indY: " << indY << "ground_z_vec: " << ground_z_vec
//                                 << endl;
                            GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                            if (is_print_message_) cout<<"uprightness_thr plane object indX: "<<indX<<" indY: "<<indY<<endl;

                        }

                    }
//                    if (GridCloudNodeMap[indX][indY]->total_score > total_score_thr) {
//                        GridCloudNodeMap[indX][indY]->cluster_flag = 1;
//                    }
                    //平地模式相对高度0.05m则检出
                    if (GridCloudNodeMap[indX][indY]->elev_diff_score > 0.05 && isslope==false) {
                        GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                        if (is_print_message_) cout<<"relative_diff_thr plane object indX: "<<indX<<" indY: "<<indY<<endl;

                    }

                    if (GridCloudNodeMap[indX][indY]->ground_z_elevation > elevation_thr_) {
                        GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                        if (is_print_message_) cout<<"elevation_thr plane object indX: "<<indX<<" indY: "<<indY<<endl;

                        continue;
                    }

                    continue;

                }

                if (Grass == true && isslope == false) {

//                    if(isslope){
//                        int bias = 2.5 * GridSizeInverse;//TODO 之后考虑修补这个问题
//
//                        if (indY < GridCloudHalfWidth + bias) {
//                            if (GridCloudNodeMap[indX][indY]->Cloud->points.size() > minGridCloudNum &&
//                                ground_z_vec < uprightness_thr_ && invalid_count < 3) {
//                                //如果栅格在点云边界处 todo 之后考虑解决这个补丁
////                            cout << "indX: " << indX << "indY: " << indY << "ground_z_vec: " << ground_z_vec
////                                 << endl;
//                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
//                                if (is_print_message_) cout<<"uprightness_thr plane object indX: "<<indX<<" indY: "<<indY<<endl;
//
//                            }
//
//                        }
//                    }



//                    if (GridCloudNodeMap[indX][indY]->total_score > total_score_thr) {
//                        GridCloudNodeMap[indX][indY]->cluster_flag = 1;
//                    }
                    if (GridCloudNodeMap[indX][indY]->ground_z_elevation > elevation_thr_) {
                        GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                        if (is_print_message_) cout<<"elevation_thr grass object indX: "<<indX<<" indY: "<<indY<<endl;
                        continue;
                    }
                    if (GridCloudNodeMap[indX][indY]->elev_diff_score > relative_diff_thr ) {
                        GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                        if (is_print_message_) cout<<"relative_diff_thr grass object indX: "<<indX<<" indY: "<<indY<<endl;

                    }
                    continue;

                }

                if(isslope){
                    if(GridCloudNodeMap[indX][indY]->slopeflag == 1){
                        //当前栅格是斜坡栅格
                        int bias = 2.5 * GridSizeInverse;
                        if (indY < GridCloudHalfWidth + bias) {
                            if (GridCloudNodeMap[indX][indY]->Cloud->points.size() > minGridCloudNum &&
                                ground_z_vec < uprightness_thr_ && invalid_count < 3) {

                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"uprightness_thr slope object indX: "<<indX<<" indY: "<<indY<<endl;

                            }

                        }
                        if (GridCloudNodeMap[indX][indY]->ground_z_elevation > elevation_thr_) {
                            GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                            if (is_print_message_) cout<<"elevation_thr grass object indX: "<<indX<<" indY: "<<indY<<endl;
                        }
                        if (GridCloudNodeMap[indX][indY]->elev_diff_score > 0.05 ) {
                            GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                            if (is_print_message_) cout<<"relative_diff_thr grass object indX: "<<indX<<" indY: "<<indY<<endl;
                        }

                    }else {
                        bool base_on_wheel_bottom;
                        for (int dx = -1; dx <= 1; dx++) {
                            for (int dy = -1; dy <= 1; dy++) {
                                // 如果这个栅格不等于1，即等于0或者2，即没有障碍物或研究遍历过。
                                // 或者就是当前这个栅格
                                // 或者待选的栅格过界
                                // 则跳过这个栅格
                                if ((dx == 0 && dy == 0) ||
                                    indX + dx < 0 || indX + dx > GridCloudWidth - 1 || indY + dy < 0 ||
                                    indY + dy > GridCloudWidth - 1)
                                    continue;
                                if (GridCloudNodeMap[indX][indY]->base_pt.z > SensorHigh - 0.1 &&
                                    GridCloudNodeMap[indX][indY]->base_pt.z < SensorHigh + 0.1){
                                    base_on_wheel_bottom = true;
                                }
                            }
                        }


                        if(base_on_wheel_bottom){
                            //物体的接地点在车轮子附近
                            //当前栅格是斜坡栅格
                        int bias = 2.5 * GridSizeInverse;
                        if (indY < GridCloudHalfWidth + bias) {
                            if (GridCloudNodeMap[indX][indY]->Cloud->points.size() > minGridCloudNum &&
                                ground_z_vec < uprightness_thr_ && invalid_count < 3) {

                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"uprightness_thr slope object indX: "<<indX<<" indY: "<<indY<<endl;

                            }

                        }
                            if (GridCloudNodeMap[indX][indY]->ground_z_elevation > elevation_thr_) {
                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"elevation_thr grass object indX: "<<indX<<" indY: "<<indY<<endl;
                            }
                            if (GridCloudNodeMap[indX][indY]->elev_diff_score > relative_diff_thr) {
                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"relative_diff_thr grass object indX: "<<indX<<" indY: "<<indY<<endl;

                            }

                        }else{
                            //不是斜坡，并且最低点不在车轮附近的点
                            if (GridCloudNodeMap[indX][indY]->ground_z_elevation > elevation_thr_) {
                                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                                if (is_print_message_) cout<<"elevation_thr grass object indX: "<<indX<<" indY: "<<indY<<endl;
                            }
                        }
                    }

                }



                if (HighGrass == true) {

                    if (GridCloudNodeMap[indX][indY]->ground_z_elevation > elevation_thr_) {
                        GridCloudNodeMap[indX][indY]->cluster_flag = 1;
                        continue;
                    }
//                      if (GridCloudNodeMap[indX][indY]->total_score > total_score_thr) {
//                          GridCloudNodeMap[indX][indY]->cluster_flag = 1;
//                      }
                }

                continue;




                // 如果跨度小于0.05,即全在一个平面
                //todo 考虑小矮墙

                // 如果平面法向量角度小于30度,则可能有障碍物
//          if ((ground_z_vec < uprightness_thr_) && (ground_z_elevation > min_z_elevation))
//          { //判断垂直度
//            GridCloudNodeMap[indX][indY]->cluster_flag = 1;
//          }

            }
        }
    }

    if (isslope == false) {
//        ComputeBigHole();
//        ComputeBigHolev2();
    }



}


void ObjSegGrid_obj::DrivableArea() {
    // 先遍历(-0.75<x<0.75)(0<y<5),找到最小的可行驶区域
    int miny = GridCloudHalfWidth ;
    int maxy = GridCloudHalfWidth + 50;
    int minx = GridCloudHalfWidth - 25;
    int maxx = GridCloudHalfWidth + 25;

    // 矩形区域两个对角点表示，用左下角点xy，右上角点xy表示
    float leftx = 0.0;
    float lefty = 0.0;
    float rightx = 0.0;
    float righty = 0.0;

    int break_flag = 0;
    for (int indY = miny; indY <= maxy; indY++) {
        for (int indX = GridCloudHalfWidth - 1; indX <= GridCloudHalfWidth + 1; indX++) {
            if (GridCloudNodeMap[indX][indY]->cluster_flag != 0) {
                maxy = indY - 1;
                break_flag = 1;
                break;
            }
        }
        if (break_flag == 1) {
            break;
        }
    }
    // 如果maxy没有有效值，则给初始值
    if (maxy == GridCloudHalfWidth - 1) {
        maxy = GridCloudHalfWidth;
    }

    // 再以maxy为y方向最大格子，分别向左、向右扩展，最大向左右扩展8个格子
    // 向左找最小栅格位置
    break_flag = 0;
    for (int indX = GridCloudHalfWidth - 2; indX >= minx; indX--) {
        for (int indY = miny; indY <= maxy; indY++) {
            if (GridCloudNodeMap[indX][indY]->cluster_flag != 0) {
                minx = indX + 1;
                break_flag = 1;
                break;
            }
        }
        if (break_flag == 1) {
            break;
        }
    }

    // 向右找最小栅格位置
    break_flag = 0;
    for (int indX = GridCloudHalfWidth + 2; indX <= maxx; indX++) {
        for (int indY = GridCloudHalfWidth; indY <= maxy; indY++) {
            if (GridCloudNodeMap[indX][indY]->cluster_flag != 0) {
                maxx = indX - 1;
                break_flag = 1;
                break;
            }
        }
        if (break_flag == 1) {
            break;
        }
    }

    // 将栅格索引转换为真实坐标
    leftx = (minx - GridCloudHalfWidth) * GridSize - 0.5 * GridSize;
    // lefty = 0.5;
    lefty = (miny - GridCloudHalfWidth) * GridSize - 0.5 * GridSize;
    rightx = (maxx - GridCloudHalfWidth) * GridSize + 0.5 * GridSize;
    righty = (maxy - GridCloudHalfWidth) * GridSize + 0.5 * GridSize;
    if (is_print_message_)
        if (is_print_message_) cout << "leftx:" << leftx << " lefty:" << lefty << " rightx" << rightx << " righty" << righty << endl;
    // 可视化

    visualization_msgs::Marker area_marker;
    visualization_msgs::MarkerArray area_marker_array;

    area_marker.header.frame_id = "camera_init";
    area_marker.ns = "";
    area_marker.color.r = 0.0f;
    area_marker.color.g = 1.0f;
    area_marker.color.b = 0.0f;
    area_marker.color.a = 0.5;
    area_marker.lifetime = ros::Duration();
    area_marker.frame_locked = true;
    area_marker.type = visualization_msgs::Marker::CUBE;
    area_marker.action = visualization_msgs::Marker::ADD;
    // area_marker.id = flag_intensity;

    area_marker.id = 1;
    area_marker.pose.position.x = (leftx + rightx) / 2.0;
    area_marker.pose.position.y = (lefty + righty) / 2.0;
    area_marker.pose.position.z = -0.5;
    area_marker.scale.x = abs(rightx - leftx);
    area_marker.scale.y = abs(righty - lefty);
    area_marker.scale.z = 0.1;
    area_marker.header.stamp = laser_Stamp.stamp;
    area_marker_array.markers.emplace_back(area_marker);

    pub_MarkerArrayArea.publish(area_marker_array);
}

/*

void ObjSegGrid_obj::ComputerFlatness() {
    // 先遍历(-0.75<x<0.75)(0.4<y<5),计算此区域内的平整度
    int miny = GridCloudHalfWidth + 2;
    int maxy = GridCloudHalfWidth + 10;
    int minx = GridCloudHalfWidth - 2;
    int maxx = GridCloudHalfWidth + 2;


    pcl::PointCloud<pcl::PointXYZ>::Ptr SelectPoints(new pcl::PointCloud<pcl::PointXYZ>());
    for (int indY = miny; indY <= maxy; indY++) {
        for (int indX = minx; indX <= maxx; indX++) {
            for (int i = 0; i < GridCloudNodeMap[indX][indY]->Cloud->points.size(); i++) {
                SelectPoints->points.push_back(GridCloudNodeMap[indX][indY]->Cloud->points[i]);
            }
        }
    }

    // 求点云的z方向的方差
    Eigen::Matrix3f covariance;                    //创建3×3协方差矩阵存储对象
    Eigen::Vector4f centeroid;                    //创建用于计算协方差矩阵的点云质心对象
    pcl::compute3DCentroid(*SelectPoints, centeroid);    //计算点云质心
    pcl::computeCovarianceMatrix(*SelectPoints, centeroid, covariance);    //计算点云协方差矩阵
    // cout << "@@@@@" << endl;
    // cout << covariance(2,2) << endl;

    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = "camera_init";
    marker_text.header.stamp = laser_Stamp.stamp;
    marker_text.ns = "";
    marker_text.color.r = 0;
    marker_text.color.g = 1;
    marker_text.color.b = 0;
    marker_text.color.a = 1;
    marker_text.scale.z = 0.1;
    marker_text.scale.x = 0.1;
    marker_text.scale.y = 0.1;
    marker_text.pose.position.z = -0.4;
    marker_text.pose.position.x = 0;
    marker_text.pose.position.y = 1;

    marker_text.lifetime = ros::Duration();
//    bbox_marker.frame_locked = true;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.id = 0;
    marker_text.text = "Var: " + to_string(covariance(2, 2));

    pub_Flatness_.publish(marker_text);
}

*/

void ObjSegGrid_obj::SORT() {
    //TODO 太大的物体不进行sort跟踪
    std::map<float, pair<int, pair<float, float>>> clustered_obj_center;
    std::map<float, pair<float, float>> clustered_obj_minmax_z;
    set<float> match_label;
    vector<Eigen::VectorXd> predict_ls;
    int maxlivecount = 3;

//    cout<<"SensorXAxis: "<<SensorXAxis<<
    for (int indX = GridCloudHalfWidth - SensorXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + SensorXAxis * GridSizeInverse; indX++) {
        for (int indY = (GridCloudWidth - 1) / 2; indY < GridCloudHalfWidth + SensorYAxis * GridSizeInverse; indY++) {

//            if(GridCloudNodeMap[indX][indY]->cluster_flag >= 2 -FLT_MIN){
//                cout<<"GridCloudNodeMap[indX][indY]->cluster_flag: "<<GridCloudNodeMap[indX][indY]->cluster_flag<<" emptyholestartlabel: "<<emptyholestartlabel<<endl;
//
//                if(smallholeintensity.find(GridCloudNodeMap[indX][indY]->cluster_flag) != smallholeintensity.end()){
//                    cout<<"smallholeintensity"<<endl;
//                }
//                else if(slopeclusterflag.find(GridCloudNodeMap[indX][indY]->cluster_flag) != slopeclusterflag.end()){
//                    cout<<"slopeclusterflag"<<endl;
//
//                }
//            }

            if (GridCloudNodeMap[indX][indY]->cluster_flag >= 2 && GridCloudNodeMap[indX][indY]->cluster_flag < emptyholestartlabel &&
                smallholeintensity.find(GridCloudNodeMap[indX][indY]->cluster_flag) == smallholeintensity.end() &&
                slopeclusterflag.find(GridCloudNodeMap[indX][indY]->cluster_flag) == slopeclusterflag.end()) { //小凹坑,斜坡,空栅格不参与Bfs聚类
//                if (GridCloudNodeMap[indX][indY]->cluster_flag >= 2) { //普通bfs聚类
                    if (clustered_obj_center.find(GridCloudNodeMap[indX][indY]->cluster_flag) !=clustered_obj_center.end()){

                    //如果当前键已经存在
                    auto value = clustered_obj_center.at(GridCloudNodeMap[indX][indY]->cluster_flag);
                    value.first += 1;
                    value.second.first += indX;
                    value.second.second += indY;
                    clustered_obj_center.at(GridCloudNodeMap[indX][indY]->cluster_flag) = value;
//                    cout << "label: " << int(GridCloudNodeMap[indX][indY]->cluster_flag) << " count: " << value.first
//                         << "sum indX:" << value.second.first << "sum indY: " << value.second.second << endl;

                    float min_z = clustered_obj_minmax_z[GridCloudNodeMap[indX][indY]->cluster_flag].first;
                    float max_z = clustered_obj_minmax_z[GridCloudNodeMap[indX][indY]->cluster_flag].second;
                    for (auto it = GridCloudNodeMap[indX][indY]->Cloud->points.begin();
                         it != GridCloudNodeMap[indX][indY]->Cloud->points.end(); it++) {
                        if (it->z > max_z) {
                            max_z = it->z;
                        } else if (it->z < min_z) {
                            min_z = it->z;
                        }
                    }
                    clustered_obj_minmax_z[GridCloudNodeMap[indX][indY]->cluster_flag] = pair(min_z, max_z);

                } else {
                        //当前键不存在
                    pair<int, pair<float, float>> value;
                    value.first = 1;
                    value.second.first = indX;
                    value.second.second = indY;
                    clustered_obj_center[GridCloudNodeMap[indX][indY]->cluster_flag] = value;
//                    cout << " indX: " << indX << " indY: " << indY << endl;
//                    cout << "first label: " << int(GridCloudNodeMap[indX][indY]->cluster_flag) << " count: " << value.first
//                         << "sum indX:" << value.second.first << "sum indY: " << value.second.second << endl;

                    float max_z = -99;
                    float min_z = 99;
                    for (auto it = GridCloudNodeMap[indX][indY]->Cloud->points.begin();
                         it != GridCloudNodeMap[indX][indY]->Cloud->points.end(); it++) {
                        if (it->z > max_z) {
                            max_z = it->z;
                        } else if (it->z < min_z) {
                            min_z = it->z;
                        }
                    }
                    clustered_obj_minmax_z[GridCloudNodeMap[indX][indY]->cluster_flag] = pair(min_z, max_z);
                }
            }
        }
    }

    //对高度和范围进行限制,忽略过远或者过高的障碍物
    std::vector<Object> objects;
    Object thisObject;
    set<float> ignore_objects;
//    for(auto iter = clustered_obj_center.begin(); iter != clustered_obj_center.end(); iter++){cout<<"all_label: "<<iter->first;}cout<<endl;

    for (auto iter = clustered_obj_center.begin(); iter != clustered_obj_center.end(); iter++) {
        //计算每个障碍物的中心
        int count = iter->second.first;
        thisObject.cx = (iter->second.second.first / count -GridCloudHalfWidth)*0.1; //sum(indX)/count
        thisObject.cy = (iter->second.second.second / count - GridCloudHalfWidth)*0.1; //sum(indY)/count
        thisObject.label = iter->first;
//        cout<<"iter->second.second.second: "<<iter->second.second.second<<" count: "<<count<<" thisObject.cx: "<<thisObject.cy<<endl;
//        cout<<"thisObject.cx: "<<thisObject.cx<<" thisObject.cy: "<<thisObject.cy<<" thisObject.label: "<<thisObject.label<<endl;

        //过大的障碍物不匹配
        if(count > max_gridcount2track){
            ignore_objects.insert(iter->first);
//            cout<<"add_ignore_label_tofar: "<<iter->first<<endl;
            continue;
        }
        //过远的障碍物不匹配
        else if (thisObject.cy > GridCloudHalfWidth + sort_min_distance_to_track * GridSizeInverse) {
//            cout<<"thisObject.cy: "<<thisObject.cy<<"GridCloudHalfWidth + 3*GridSizeInverse"<<GridCloudHalfWidth + 3*GridSizeInverse<<endl;
            ignore_objects.insert(iter->first);
//            cout<<"add_ignore_label_tofar: "<<iter->first<<endl;
            continue;

        }
        //过高的障碍物不匹配
        else {
            float label = iter->first;
            float diff_high = clustered_obj_minmax_z[label].second - clustered_obj_minmax_z[label].first;
            if (diff_high > sort_min_diff_high_to_track) {
//                cout<<" diff_high: "<<diff_high<<endl;

                ignore_objects.insert(iter->first);
//                cout<<"add_ignore_label_tohigh:"<<iter->first<<endl;
//                if(iter == clustered_obj_center.begin()){clustered_obj_center.erase(iter->first);continue;}
//                iter = clustered_obj_center.erase(iter--);
//                iter--;
                continue;
            }
        }
        objects.emplace_back(thisObject);
//        cout<<"add_objects: "<<thisObject.label<<endl;

    }

    set<int> unmatchedDetections;

    if (filters.size() == 0) // the first frame met
    {
        if (objects.size() > 0) {
            // 根据第一次的检测框初始化滤波器
            for (unsigned int i = 0; i < objects.size(); i++) {
                Eigen::VectorXd x(4);
                // 中心点 x y
                float cx = objects[i].cx;
                float cy = objects[i].cy;
                // 变换为中心点xy
                x << cx, cy, 0, -wheel_dx*0.1; //TODO 加入速度 ，中心点，xy方向速度
                // 初始化卡尔曼滤波器
                KalmanFilter kf = KalmanFilter(x);
                filters.emplace_back(kf);
            }

//            if(ignore_objects.size() ==0){
//                cout << "SimSORT STEP 1 === INIT SimSORT ENDS" << endl;
//                return;//第一次遇到的障碍物注册，但是不输出
//            }
            goto processObjCloud;
        }

    }


    for (auto it = filters.begin(); it != filters.end();) {
//        cout<<"before_pred cx: "<<it->x[0]<<" cy :"<<it->x[1]<<endl;

        Eigen::Vector2d predict_z = (*it).predict();
//        cout<<"after_pred cx: "<<it->x[0]<<" cy :"<<it->x[1]<<endl;

        if (predict_z[0] >= -SensorXAxis && predict_z[0] <= SensorXAxis && predict_z[1] >= 0 && predict_z[1] < SensorYAxis) {
            predict_ls.emplace_back(predict_z);
            it++;
        } else {
//            cout<<"delete_filter cx: "<<it->x[0]<<" cy :"<<it->x[1]<<endl;
            it = filters.erase(it);
        }
    }

    // 滤波器被删除后仅有0个
    if (filters.size() == 0) // the first frame met
    {
        if(objects.size()>0){
            for (unsigned int i = 0; i < objects.size(); i++) {
                Eigen::VectorXd x(4);
                // 中心点 x y
                float cx = objects[i].cx;
                float cy = objects[i].cy;
                // 变换为中心点xy
                x << cx, cy, 0, -wheel_dx*0.1; //TODO 加入速度 ，中心点，xy方向速度
                // 初始化卡尔曼滤波器
                KalmanFilter kf = KalmanFilter(x);
                filters.emplace_back(kf);
            }
        }
        goto processObjCloud;

    }

//    cout<<"filters_size: "<<filters.size()<<endl;

    if (filters.size() != 0) {
        // 计算预测框和检测框之间的距离代价矩阵
        int trkNum = predict_ls.size();
        int detNum = objects.size();
        vector<vector<double>> costMatrix;
        costMatrix.resize(trkNum, vector<double>(detNum, 0));
        for (unsigned int i = 0; i < trkNum; i++) {
            for (unsigned int j = 0; j < detNum; j++) {
                float cx1 = predict_ls[i][0];
                float cy1 = predict_ls[i][1];
                float cx2 = objects[j].cx;
                float cy2 = objects[j].cy;
                costMatrix[i][j] = sqrt((cx2 - cx1) * (cx2 - cx1)  +
                                        (cy2 - cy1) * (cy2 - cy1));
            }
        }


        // 利用匈牙利算法解决当前分配问题
        // the resulting assignment is [track(prediction) : detection], with len=preNum
        HungarianAlgorithm HungAlgo;
        vector<int> assignment;
        assignment.clear();
        HungAlgo.Solve(costMatrix, assignment);
        // 已匹配, 未匹配检测框，未匹配预测框
        set<int> allItems;
        set<int> matchedItems;
        set<int> unmatchedTrajectories;

        if (detNum > trkNum)  // 未匹配检测框
        {
            for (unsigned int n = 0; n < detNum; n++)
                allItems.insert(n);

            for (unsigned int i = 0; i < trkNum; ++i)
                matchedItems.insert(assignment[i]);
            set_difference(allItems.begin(), allItems.end(),
                           matchedItems.begin(), matchedItems.end(),
                           insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
        } else if (detNum < trkNum) // 未匹配预测框
        {
            for (unsigned int i = 0; i < trkNum; ++i)
                // unassigned label will be set as -1 in the assignment algorithm
                if (assignment[i] == -1)
                    unmatchedTrajectories.insert(i);
        } 


        // 过滤掉距离过远的匹配对
        vector<cv::Point> matchedPairs;
        for (unsigned int i = 0; i < trkNum; ++i) {
            if (assignment[i] == -1) // pass over invalid values
                continue;
//        float cx = objects[assignment[i]].cx;
//        float cy = objects[assignment[i]].cy;
            if (costMatrix[i][assignment[i]] > sort_min_disance_to_match) //两个障碍物之间距离大于0.3m,2m/s下障碍物中心最多移动2gen2m
            {
                unmatchedTrajectories.insert(i);
                unmatchedDetections.insert(assignment[i]);
            } else {
                if (filters[i].is_tracked == false) {
                    filters[i].is_tracked = true;
                    Eigen::VectorXd z(2);
                    float cx = objects[assignment[i]].cx;
                    float cy = objects[assignment[i]].cy;
                    z << cx, cy;
                    z = filters[i].update(z);
//                filters[i]
//                double during_time = ros::Time::now().toSec() - filters[i].time_now;
//                if(during_time > 0.15){
//                    unmatchedTrajectories.insert(i);
//                    unmatchedDetections.insert(assignment[i]);
//                }


                } else if (filters[i].is_tracked == true) {
                    matchedPairs.emplace_back(cv::Point(i, assignment[i]));

                }
            }
        }

        // 匹配检测框->更新卡尔曼滤波器
        int detIdx, trkIdx;
        for (unsigned int i = 0; i < matchedPairs.size(); i++) {
            trkIdx = matchedPairs[i].x;
            detIdx = matchedPairs[i].y;
            Eigen::VectorXd z(2);
            float cx = objects[detIdx].cx;
            float cy = objects[detIdx].cy;
//            z << cx, cy;
//            z = filters[trkIdx].update(z);
//        objects[detIdx].cx = z[0];
//        objects[detIdx].cy = z[1];
            filters[trkIdx].x[0] = cx;
            filters[trkIdx].x[1] = cy;
            filters[trkIdx].x[2] = 0;
            filters[trkIdx].x[3] = -wheel_dx*0.1;
            filters[trkIdx].live_count += 1;
            filters[trkIdx].tracked_count += 1;

            match_label.insert(objects[detIdx].label);
        }

        // 未匹配预测框->删除卡尔曼滤波器
        int moved = 0;
        for (auto umt : unmatchedTrajectories) {
            int i = 0;
            for (auto it = filters.begin(); it != filters.end();) {
                if (i == umt - moved){
                    it->live_count += 1;
                    if(it->live_count == maxlivecount){
                        if(it->tracked_count < maxlivecount - 1){
                            //勾叉叉
                            it = filters.erase(it);

                        }else if(it->tracked_count == maxlivecount - 1){
                            //勾勾叉 live_count = 3 tracked_count = 2情况

                            //更新为勾叉live_count = 2 tracked_count = 1
                            it->live_count -= 1;
                            it->tracked_count -= 1;
                            it->x[3] = -wheel_dx*0.1; //更新速度

                        }

                    }

                }
                else{
                    ++it;
                }
                i++;
            }
            moved++;
        }

        // 未匹配检测框->初始化卡尔曼滤波器
        for (auto umd : unmatchedDetections) {
            Eigen::VectorXd x(4);
            float cx = objects[umd].cx;
            float cy = objects[umd].cy;
            x << cx, cy, 0, -wheel_dx*0.1;
            KalmanFilter filter = KalmanFilter(x);
            filters.emplace_back(filter);
        }




    }

//    cout << endl;
//    set<float> all_label;
//    set<float> unmatch_label;
//    for (auto it = unmatchedDetections.begin(); it != unmatchedDetections.end(); it++) { cout << "unmatchid: " << *it; }
//    cout << endl;
//    for (auto it = match_label.begin(); it != match_label.end(); it++) { cout << "match_label: " << *it; }
//    cout << endl;
//    for (auto it = ignore_objects.begin(); it != ignore_objects.end(); it++) { cout << "ignore_objects: " << *it; }
//    cout << endl;



    processObjCloud:
    //更新label
    pcl::PointXYZI thispoint;
    map_objectcloud.clear();
    for (int indX = GridCloudHalfWidth - SensorXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + SensorXAxis * GridSizeInverse; indX++) {
        for (int indY = (GridCloudWidth - 1) / 2; indY < GridCloudHalfWidth +SensorYAxis*GridSizeInverse; indY++) {
            if (GridCloudNodeMap[indX][indY]->cluster_flag >= 2 - FLT_MIN) {
//                cout << "in1 flag: "<<GridCloudNodeMap[indX][indY]->cluster_flag<<endl;
//                for (auto it = match_label.begin(); it != match_label.end(); it++) { cout << "match_label: " << *it; }
//                cout << endl;
//                for (auto it = ignore_objects.begin(); it != ignore_objects.end(); it++) { cout << "ignore_objects: " << *it; }
//                cout << endl;
//                cout<<"match_label.count(GridCloudNodeMap[indX][indY]->cluster_flag)"<<match_label.count(GridCloudNodeMap[indX][indY]->cluster_flag)<<endl;
//                cout<<"ignore_objects.count(GridCloudNodeMap[indX][indY]->cluster_flag)"<<ignore_objects.count(GridCloudNodeMap[indX][indY]->cluster_flag)<<endl;

                if (match_label.count(GridCloudNodeMap[indX][indY]->cluster_flag) ||
                    ignore_objects.count(GridCloudNodeMap[indX][indY]->cluster_flag)||
                    smallholeintensity.find(GridCloudNodeMap[indX][indY]->cluster_flag) != smallholeintensity.end() ||
                        slopeclusterflag.find(GridCloudNodeMap[indX][indY]->cluster_flag) != slopeclusterflag.end()) {

//                    cout << "in flag: " << GridCloudNodeMap[indX][indY]->cluster_flag << "match_label ?"
//                         << match_label.count(GridCloudNodeMap[indX][indY]->cluster_flag) << " ignore_objects?"
//                         << ignore_objects.count(GridCloudNodeMap[indX][indY]->cluster_flag) << endl;

                    if (map_objectcloud.find(GridCloudNodeMap[indX][indY]->cluster_flag) != map_objectcloud.end()) {
//                    auto this_point_cloud = map_objectcloud.at(GridCloudNodeMap[indX][indY]->cluster_flag);
                        auto this_point_cloud = map_objectcloud[GridCloudNodeMap[indX][indY]->cluster_flag];

//                        cout << "before add" << "indX:" << indX << " indY: " << indY << "label : "
//                             << GridCloudNodeMap[indX][indY]->cluster_flag << " count: " << this_point_cloud.size()
//                             << endl;

//                    //TODO
                        for (int i = 0; i < GridCloudNodeMap[indX][indY]->Cloud->size(); i++) {
                            thispoint.x = GridCloudNodeMap[indX][indY]->Cloud->points[i].x;
                            thispoint.y = GridCloudNodeMap[indX][indY]->Cloud->points[i].y;
                            thispoint.z = GridCloudNodeMap[indX][indY]->Cloud->points[i].z;
                            thispoint.intensity = GridCloudNodeMap[indX][indY]->cluster_flag;
                            this_point_cloud.points.emplace_back(thispoint);
                        }
                        map_objectcloud[GridCloudNodeMap[indX][indY]->cluster_flag] = this_point_cloud;

//                        cout << "after add" << "indX:" << indX << " indY: " << indY << "label : "
//                             << GridCloudNodeMap[indX][indY]->cluster_flag << " count: " << this_point_cloud.size()
//                             << endl;


                    } else {
                        //TODO 不知道这样写会不会有问题
                        pcl::PointCloud<pcl::PointXYZI> this_point_cloud;
                        this_point_cloud.clear();
                        for (int i = 0; i < GridCloudNodeMap[indX][indY]->Cloud->size(); i++) {
                            thispoint.x = GridCloudNodeMap[indX][indY]->Cloud->points[i].x;
                            thispoint.y = GridCloudNodeMap[indX][indY]->Cloud->points[i].y;
                            thispoint.z = GridCloudNodeMap[indX][indY]->Cloud->points[i].z;
                            thispoint.intensity = GridCloudNodeMap[indX][indY]->cluster_flag;
                            this_point_cloud.points.emplace_back(thispoint);
                        }
                        map_objectcloud[GridCloudNodeMap[indX][indY]->cluster_flag] = this_point_cloud;

//                        cout << "first add" << "indX:" << indX << " indY: " << indY << "label : "
//                             << GridCloudNodeMap[indX][indY]->cluster_flag << " count: " << this_point_cloud.size()
//                             << endl;

                    }


                }
#ifdef Debug
                else {
                    //没跟踪上的点可视化
//                    cout<<"mis_match_label: "<<GridCloudNodeMap[indX][indY]->cluster_flag;
                    GridCloudNodeMap[indX][indY]->cluster_flag = 0;//未跟踪上的点设置clusterflag=0
                    for (int i = 0; i < GridCloudNodeMap[indX][indY]->Cloud->size(); i++) {
                        thispoint.x = GridCloudNodeMap[indX][indY]->Cloud->points[i].x;
                        thispoint.y = GridCloudNodeMap[indX][indY]->Cloud->points[i].y;
                        thispoint.z = GridCloudNodeMap[indX][indY]->Cloud->points[i].z;
                        thispoint.intensity = 0;
                        mismatchObjectCloud->points.emplace_back(thispoint);
                    }
//                    cout<<"indx: "<<indX<<" indY: "<<indY<<" pointsize: "<<mismatchObjectCloud->size()<<endl;


                }
#endif

            }
        }
    }

    for (auto it = map_objectcloud.begin(); it != map_objectcloud.end(); it++) {
        auto thispointcloud = it->second;
//        cout<<"thispointcloud label:"<<thispointcloud.points[0].intensity<<" thispointcloud size: "<<thispointcloud.points.size()<<endl;
        *ObjPointCloud += thispointcloud;
    }


    return;

}

//对栅格进行栅格聚类
void ObjSegGrid_obj::ClusterAndPubObjectGrid() {
    ObjPointCloud->clear();
    //栅格BFS聚类
    pcl::PointXYZI thispoint;
    bool last_cluster = false;
    // 遍历每一个栅格
    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth - lidarYAxis * GridSizeInverse;
             indY < GridCloudHalfWidth + lidarYAxis * GridSizeInverse; indY++) {
            // 如果flag==1（有障碍物）
            if (GridCloudNodeMap[indX][indY]->cluster_flag == 1) {
                vector<pair<int, int>> neighbour;
                // 存入当前ID到neighbour
                neighbour.emplace_back(pair<int, int>(indX, indY));
                GridCloudNodeMap[indX][indY]->cluster_flag = clusterlabel;

                //标记斜坡栅格的聚类索引
                if(GridCloudNodeMap[indX][indY]->slopeflag > 0){
                    slopeclusterflag.insert(clusterlabel);
                }
                if(GridCloudNodeMap[indX][indY]->is_small_hole == true){
                    smallholeintensity.insert(clusterlabel);
                }


                // 如果不为空
                while (!neighbour.empty()) {
                    // 提取出这个栅格
                    pair<int, int> thisgrid = neighbour.back();
                    neighbour.pop_back();
                    int indx = thisgrid.first;
                    int indy = thisgrid.second;

                    //标记斜坡栅格的聚类索引
                    if(GridCloudNodeMap[indx][indy]->slopeflag > 0){
                        slopeclusterflag.insert(clusterlabel);
                    }
                    if(GridCloudNodeMap[indx][indy]->is_small_hole == true){
                        smallholeintensity.insert(clusterlabel);
                    }

//            cout<<"indx:"<<indx<<"indy:"<<indy<<"intensity"<<label<<endl;
                    // 获取栅格内点云的数量
                    int CloudSize = GridCloudNodeMap[indx][indy]->Cloud->size();

                    // 遍历每一个点，将反射率I赋予一类的标签，存入障碍物点云
                    if(use_sort == false){
                        for (int i = 0; i < CloudSize; i++) {
                            thispoint.x = GridCloudNodeMap[indx][indy]->Cloud->points[i].x;
                            thispoint.y = GridCloudNodeMap[indx][indy]->Cloud->points[i].y;
                            thispoint.z = GridCloudNodeMap[indx][indy]->Cloud->points[i].z;
                            thispoint.intensity = clusterlabel;
                            ObjPointCloud->points.emplace_back(thispoint);
                        }
//                        if(GridCloudNodeMap[indx][indy]->is_small_hole == true){
//                            smallholeintensity.insert(clusterlabel);
//                        }

                    }

                    // 查找前后左右的8个栅格
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            // 如果这个栅格不等于1，即等于0或者2，即没有障碍物或研究遍历过。
                            // 或者就是当前这个栅格
                            // 或者待选的栅格过界
                            // 则跳过这个栅格
                            if ((dx == 0 && dy == 0) ||
                                indx + dx < 0 || indx + dx > GridCloudWidth - 1 || indy + dy < 0 ||
                                indy + dy > GridCloudWidth - 1 ||
                                (GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag != 1 &&
                                 GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag != 0.5))
                                continue;
                            // 否则将这个栅格加入待查找列表
//                cout<<"indx+dx:"<<indx+dx<<"indy+dy:"<<indy+dy<<"clusterflag:"<<GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag<<endl;
                            neighbour.emplace_back(pair<int, int>(indx + dx, indy + dy));
                            GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = clusterlabel;

                        }
                    }
                }
                if (is_print_message_) cout<<"object bfs label: "<<clusterlabel<<endl;
                clusterlabel++;
            }

        }
    }
    if(is_print_message_){
        for(auto it = slopeclusterflag.begin();it != slopeclusterflag.end() ;it++){
            cout<<"slopeclusterflag: "<<*it<<endl;
        }

        for(auto it = smallholeintensity.begin();it != smallholeintensity.end() ;it++){
            cout<<"smallholeintensity: "<<*it<<endl;
        }
    }


//    max_label = clusterlabel;

}

void ObjSegGrid_obj::ComputeBoundingPt(const pcl::PointCloud<pcl::PointXYZ>::Ptr BoundCloud,
                                       int flag_intensity) {


    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = "camera_init";
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 1.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.scale.x = 0.2;
    bbox_marker.scale.y = 0.2;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::POINTS;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    bbox_marker.id = flag_intensity;

    pcl::PointCloud<pcl::PointXYZ>::Ptr XYBoundingCloud_single(new pcl::PointCloud<pcl::PointXYZ>());

//    sort(BoundCloud->begin(), BoundCloud->end(), comparez);
//    float meanGroundHigh = 0;
//    for(int i =0;i<num_lpr_;i++){meanGroundHigh += BoundCloud->points[i].z;}
//    meanGroundHigh = meanGroundHigh/float(num_lpr_) + grasshigh_real;
////    cout<<"meanGroundHigh"<<meanGroundHigh<<endl;
//
//    //过滤地面点
//    if(HighGrass ==false){
//        pcl::PassThrough<pcl::PointXYZ> passthrough;
//        passthrough.setInputCloud(BoundCloud);//输入点云
//        passthrough.setFilterFieldName("z");//对z轴进行操作
//        passthrough.setFilterLimits(meanGroundHigh, 10);//设置直通滤波器操作范围
////    passthrough.setFilterLimitsNegative(true);//true表示保留范围外，false表示保留范围内
//        passthrough.filter(*BoundCloud);//
//    }


//    if(BoundCloud->points.size() < 5 ){
//        return;
//    }


    float resolution = 0.1;

    pcl::PointXYZ Xmin, Xmax, Ymin, Ymax;
    Xmin.x = 99;
    Xmax.x = -99;
    Ymin.y = 99;
    Ymax.y = -99;

    for (auto iter = BoundCloud->begin(); iter != BoundCloud->end(); iter++) {
        if (iter->x < Xmin.x) {
            Xmin.x = iter->x;
            Xmin.y = iter->y;
            Xmin.z = iter->z;
        }
        if (iter->x > Xmax.x) {
            Xmax.x = iter->x;
            Xmax.y = iter->y;
            Xmax.z = iter->z;
        }
        if (iter->y < Ymin.y) {
            Ymin.x = iter->x;
            Ymin.y = iter->y;
            Ymin.z = iter->z;
        }
        if (iter->y > Ymax.y) {
            Ymax.x = iter->x;
            Ymax.y = iter->y;
            Ymax.z = iter->z;
        }
    }
    // cout<<"Xmin.x:"<<Xmin.x<<" Xmax.x"<<Xmax.x<<" Ymin.y:"<<Ymin.y<<" Ymax.y"<<Ymax.y<<endl;

    // pcl::PointXYZ min,max;
    pcl::PointXYZI ptxyzi;
    pcl::PointXYZ ptxyz;
    geometry_msgs::Point maxpt, minpt;
    geometry_msgs::Point leftpt, rightpt;

    // sort(BoundCloud->begin(), BoundCloud->end(), comparex);

    // pcl::getMinMax3D(*BoundCloud, min, max); //在聚类的坐标系中计算最大最小值
//    cout<<"id:"<<flag_intensity<<"position:(right)"<<setprecision(3)<<min.x<<"(left)"<<setprecision(3)<<max.x<<"(front)"<<setprecision(3)<<min.y-0.13<<"scal:(x)"<<setprecision(3)<<max.x-min.x<<endl;
    // cout<<"min.x:"<<min.x<<" max.x"<<max.x<<" min.y:"<<min.y<<" max.y"<<max.y<<endl;

    int seg_count = ceil((Xmax.x - Xmin.x) / resolution);

    double step = (Xmax.x - Xmin.x) / seg_count;
    int cloudsize = BoundCloud->points.size();


    leftpt.z = -SensorHigh;
    rightpt.z = -SensorHigh;
    leftpt.x = Xmin.x;
    leftpt.y = Xmin.y;
    rightpt.x = Xmax.x;
    rightpt.y = Xmax.y;


    ptxyzi.x = leftpt.x;
    ptxyzi.y = leftpt.y;
    ptxyzi.z = leftpt.z;
    ptxyzi.intensity = flag_intensity;
    XYBoundingCloud->points.emplace_back(ptxyzi);
    ptxyz.x = leftpt.x;
    ptxyz.y = leftpt.y;
    ptxyz.z = leftpt.z;
    XYBoundingCloud_single->points.emplace_back(ptxyz);


    bbox_marker.points.emplace_back(leftpt);


    maxpt.z = -SensorHigh;
    minpt.z = -SensorHigh;


    int pt_id = 0;
    float start;
    start = Xmin.x + resolution;
    // 遍历窗口10cm,找每个窗口y方向最大的点
    for (int i = 1; i <= seg_count; i++) {
        maxpt.y = -99;
        for (pt_id = 0; pt_id < BoundCloud->points.size(); pt_id++) {
            if (BoundCloud->points[pt_id].x < start && BoundCloud->points[pt_id].x > start - resolution && pt_id >= 0 &&
                pt_id <= cloudsize - 1) {
                if (BoundCloud->points[pt_id].y > maxpt.y) {
                    maxpt.x = BoundCloud->points[pt_id].x;
                    maxpt.y = BoundCloud->points[pt_id].y;
//                    maxpt.z = BoundCloud->points[pt_id].z;

                    //                      cout<<"maxpt.x"<<maxpt.x<<"maxpt.y"<<maxpt.y<<"maxpt.z"<<ptxyzi.z<<endl;

                }
            }
        }
        if (maxpt.y == -99)
            continue;
        bbox_marker.points.emplace_back(maxpt);
        ptxyzi.x = maxpt.x;
        ptxyzi.y = maxpt.y;
        ptxyzi.z = maxpt.z;
        ptxyzi.intensity = flag_intensity;
        //        cout<<"ptxyzi.x"<<ptxyzi.x<<"ptxyzi.y"<<ptxyzi.y<<"ptxyzi.z"<<ptxyzi.z<<endl;
        XYBoundingCloud->points.emplace_back(ptxyzi);
        ptxyz.x = maxpt.x;
        ptxyz.y = maxpt.y;
        ptxyz.z = maxpt.z;
        XYBoundingCloud_single->points.emplace_back(ptxyz);
        start += resolution;
    }

    //加入最右侧的点
    bbox_marker.points.emplace_back(rightpt);
    ptxyzi.x = rightpt.x;
    ptxyzi.y = rightpt.y;
    ptxyzi.z = rightpt.z;
    ptxyzi.intensity = flag_intensity;
//      cout<<"rightpt.x"<<leftpt.x<<"rightpt.y"<<leftpt.y<<"rightpt.z"<<leftpt.z<<endl;

    XYBoundingCloud->points.emplace_back(ptxyzi);
    ptxyz.x = rightpt.x;
    ptxyz.y = rightpt.y;
    ptxyz.z = rightpt.z;
    XYBoundingCloud_single->points.emplace_back(ptxyz);

    start = Xmax.x - resolution;
    //窗口10cm找 y方向最小点
    for (int i = 1; i <= seg_count; i++) {
        minpt.y = 99;
        for (pt_id = 0; pt_id < BoundCloud->points.size(); pt_id++) {

            if (BoundCloud->points[pt_id].x >= start && BoundCloud->points[pt_id].x <= start + resolution && pt_id >= 0 &&
                pt_id <= cloudsize - 1) {
                if (BoundCloud->points[pt_id].y < minpt.y) {
                    minpt.x = BoundCloud->points[pt_id].x;
                    minpt.y = BoundCloud->points[pt_id].y;
//                    minpt.z = BoundCloud->points[pt_id].z;

                }
                pt_id++;
            }
        }
        if (minpt.y == 99)
            continue;
        bbox_marker.points.emplace_back(minpt);
        ptxyzi.x = minpt.x;
        ptxyzi.y = minpt.y;
        ptxyzi.z = minpt.z;
        ptxyzi.intensity = flag_intensity;
//    cout<<"ptxyzi.x"<<ptxyzi.x<<"ptxyzi.y"<<ptxyzi.y<<"ptxyzi.z"<<ptxyzi.z<<endl;
        XYBoundingCloud->points.emplace_back(ptxyzi);
        ptxyz.x = minpt.x;
        ptxyz.y = minpt.y;
        ptxyz.z = minpt.z;
        XYBoundingCloud_single->points.emplace_back(ptxyz);
        start -= resolution;
    }



    //增加y方向的点
    // sort(BoundCloud->begin(), BoundCloud->end(), comparey);

    // pcl::getMinMax3D(*BoundCloud, min, max); //在聚类的坐标系中计算最大最小值
//    cout<<"id:"<<flag_intensity<<"position:(right)"<<setprecision(3)<<min.x<<"(left)"<<setprecision(3)<<max.x<<"(front)"<<setprecision(3)<<min.y-0.13<<"scal:(x)"<<setprecision(3)<<max.x-min.x<<endl;
    seg_count = floor((Ymax.y - Ymin.y) / 0.1) + 1;

    step = (Ymax.y - Ymin.y) / seg_count;
    cloudsize = BoundCloud->points.size();


    leftpt.z = -SensorHigh;
    rightpt.z = -SensorHigh;
    leftpt.x = Ymax.x;
    leftpt.y = Ymax.y;

    rightpt.x = Ymin.x;
    rightpt.y = Ymin.y;
    // cout<<"Yleftpt.y:"<<leftpt.y<<" Yrightpt.y:"<<rightpt.y<<endl;


    ptxyzi.x = leftpt.x;
    ptxyzi.y = leftpt.y;
    ptxyzi.z = leftpt.z;
    ptxyzi.intensity = flag_intensity;
    XYBoundingCloud->points.emplace_back(ptxyzi);//加入Y方向最大点
    ptxyz.x = leftpt.x;
    ptxyz.y = leftpt.y;
    ptxyz.z = leftpt.z;
    XYBoundingCloud_single->points.emplace_back(ptxyz);

    bbox_marker.points.emplace_back(leftpt);


    maxpt.z = -SensorHigh;
    minpt.z = -SensorHigh;


    pt_id = 0;
    start = Ymin.y + resolution;
    for (int i = 1; i <= seg_count; i++) {
        maxpt.x = -99;
        for (pt_id = 0; pt_id < BoundCloud->points.size(); pt_id++) {
            if (BoundCloud->points[pt_id].y < start && BoundCloud->points[pt_id].y > start - resolution && pt_id >= 0 &&
                pt_id <= cloudsize - 1) {
                if (BoundCloud->points[pt_id].x > maxpt.x) {
                    maxpt.x = BoundCloud->points[pt_id].x;
                    maxpt.y = BoundCloud->points[pt_id].y;
//                    maxpt.z = BoundCloud->points[pt_id].z;

                }
            }
        }
        if (maxpt.x == -99)
            continue;
        bbox_marker.points.emplace_back(maxpt);
        ptxyzi.x = maxpt.x;
        ptxyzi.y = maxpt.y;
        ptxyzi.z = maxpt.z;
        ptxyzi.intensity = flag_intensity;
        XYBoundingCloud->points.emplace_back(ptxyzi);
        ptxyz.x = maxpt.x;
        ptxyz.y = maxpt.y;
        ptxyz.z = maxpt.z;
        XYBoundingCloud_single->points.emplace_back(ptxyz);
        start += resolution;
    }


    bbox_marker.points.emplace_back(rightpt);
    ptxyzi.x = rightpt.x;
    ptxyzi.y = rightpt.y;
    ptxyzi.z = rightpt.z;
    ptxyzi.intensity = flag_intensity;

    XYBoundingCloud->points.emplace_back(ptxyzi);
    ptxyz.x = rightpt.x;
    ptxyz.y = rightpt.y;
    ptxyz.z = rightpt.z;
    XYBoundingCloud_single->points.emplace_back(ptxyz);

    start = Ymax.y - resolution;
    for (int i = 1; i <= seg_count; i++) {
        minpt.x = 99;
        for (pt_id = 0; pt_id < BoundCloud->points.size(); pt_id++) {

            if (BoundCloud->points[pt_id].y >= start && BoundCloud->points[pt_id].y <= start + resolution && pt_id >= 0 &&
                pt_id <= cloudsize - 1) {
                if (BoundCloud->points[pt_id].x < minpt.x) {
                    minpt.x = BoundCloud->points[pt_id].x;
                    minpt.y = BoundCloud->points[pt_id].y;
//                    minpt.z = BoundCloud->points[pt_id].z;

                }
                pt_id++;
            }
        }
        if (minpt.x == 99)
            continue;
        bbox_marker.points.emplace_back(minpt);
        ptxyzi.x = minpt.x;
        ptxyzi.y = minpt.y;
        ptxyzi.z = minpt.z;
        ptxyzi.intensity = flag_intensity;
        XYBoundingCloud->points.emplace_back(ptxyzi);
        ptxyz.x = minpt.x;
        ptxyz.y = minpt.y;
        ptxyz.z = minpt.z;
        XYBoundingCloud_single->points.emplace_back(ptxyz);

        start -= resolution;
    }

//    pcl::PointXYZ min, max;
//    pcl::getMinMax3D(*XYBoundingCloud_single, min, max); //在聚类的坐标系中计算最大最小值
//    if (is_print_message_)
//        cout << "id:" << flag_intensity << "position:(left)" << min.x << "(right)" << max.x << "(front)" << min.y
//             << "scal:(x)" << max.x - min.x << endl;
//    cout<<"seg_count"<<seg_count<<"XYBoundingCloudszie"<<XYBoundingCloud->points.size()<<endl;
    bounding_marker_array.markers.emplace_back(bbox_marker);

}


#ifdef Debug
void ObjSegGrid_obj::PubFeatureVis() {

    if(pub_MarkerArrayFeature_.getNumSubscribers() != 0) {

        visualization_msgs::Marker marker_text;
        visualization_msgs::MarkerArray marker_array_text;
        marker_text.header.frame_id = "camera_init";
        marker_text.header.stamp = laser_Stamp.stamp;
        marker_text.ns = "";
        marker_text.color.r = 1;
        marker_text.color.g = 0;
        marker_text.color.b = 0;
        marker_text.color.a = 1;
        marker_text.scale.z = 0.03;
        marker_text.scale.x = 0.03;
        marker_text.scale.y = 0.03;

        marker_text.lifetime = ros::Duration();
//    bbox_marker.frame_locked = true;
        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::Marker::ADD;
        marker_text.id = 0;
        geometry_msgs::Pose pose_text;
        int bias = 5 * GridSizeInverse;

        for (int indX = GridCloudHalfWidth - bias; indX < GridCloudHalfWidth + bias; indX++) {
            for (int indY = GridCloudHalfWidth - bias; indY < GridCloudHalfWidth + bias; indY++) {

                if (GridCloudNodeMap[indX][indY]->cluster_flag >= 1) {
                    marker_text.color.r = 0;
                    marker_text.color.g = 0;
                    marker_text.color.b = 1;
                    pose_text.position.x = (indX - GridCloudHalfWidth) * GridSize;
                    pose_text.position.y = (indY - GridCloudHalfWidth) * GridSize;
                    marker_text.pose = pose_text;
//                marker_text.text += to_string(GridCloudNodeMap[indX][indY]->TopCloudnum)+":"+to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100))) + ":" +to_string((int)(GridCloudNodeMap[indX][indY]->top_linearity*100)) +":"+ to_string((int)(GridCloudNodeMap[indX][indY]->top_planarity*100));
//                marker_text.text += to_string(((int)(GridCloudNodeMap[indX][indY]->total_score*100)))+":"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100)))+ "\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->elevation_score*100)))+ ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->appear_score*100)))+ "\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->elev_diff_score*100))) + ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->planarity_score*100))) +"\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->top_planarity*100))) + ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_vec*100)));

//                marker_text.text += to_string(indX) + ":"+ to_string(indY) + "\n" + to_string((int)(GridCloudNodeMap[indX][indY]->perpendicularity*100)) + ":" +  to_string((int)(GridCloudNodeMap[indX][indY]->min_z_elev*100)) + "\n" + to_string(((int)(GridCloudNodeMap[indX][indY]->Cloud->points.size()))) + ":"+to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100)));

//                    marker_text.text += to_string(indX) + ":" + to_string(indY) + "\n" +
//                                        to_string((int) (GridCloudNodeMap[indX][indY]->Cloud->points.size())) + ":" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_elevation * 100))) +
//                                        ":" +
//                                        to_string(((GridCloudNodeMap[indX][indY]->slopeflag))) + "\n" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_vec * 100))) + ":" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->elev_diff_score * 100))) + ":" +
//                                        to_string((int) (GridCloudNodeMap[indX][indY]->cluster_flag));


                    marker_text.text += to_string(indX) + ":" + to_string(indY) + "\n" +
                                        to_string((int) (GridCloudNodeMap[indX][indY]->Cloud->points.size())) + ":" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_elevation * 100))) +
                                        ":" +
                                        to_string(((int)(GridCloudNodeMap[indX][indY]->elev_diff_score * 100))) + "\n" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->slopeflag))) + ":" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->slopeangle * 100))) + ":" +
                                        to_string((int) (GridCloudNodeMap[indX][indY]->cluster_flag));
                    marker_array_text.markers.emplace_back(marker_text);
                    marker_text.text.clear();
                    marker_text.id += 1;
                } else if (GridCloudNodeMap[indX][indY]->ground_z_elevation > min_z_elevation) {
                    marker_text.color.r = 0.5;
                    marker_text.color.g = 0.6;
                    marker_text.color.b = 0.6;
                    pose_text.position.x = (indX - GridCloudHalfWidth) * GridSize;
                    pose_text.position.y = (indY - GridCloudHalfWidth) * GridSize;
                    marker_text.pose = pose_text;
//                marker_text.text += to_string(GridCloudNodeMap[indX][indY]->TopCloudnum)+":"+to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100))) + ":" +to_string((int)(GridCloudNodeMap[indX][indY]->top_linearity*100)) +":"+ to_string((int)(GridCloudNodeMap[indX][indY]->top_planarity*100));

//                marker_text.text += to_string(((int)(GridCloudNodeMap[indX][indY]->total_score*100)))+":"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100)))+ "\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->elevation_score*100)))+ ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->appear_score*100)))+ "\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->elev_diff_score*100))) + ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->planarity_score*100))) +"\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->top_planarity*100))) + ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_vec*100)));

//                    marker_text.text += to_string(indX) + ":" + to_string(indY) + "\n" +
//                                        to_string((int) (GridCloudNodeMap[indX][indY]->Cloud->points.size())) + ":" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_elevation * 100))) +
//                                        ":" +
//                                        to_string(((GridCloudNodeMap[indX][indY]->slopeflag))) + "\n" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_vec * 100))) + ":" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->elev_diff_score * 100))) + ":" +
//                                        to_string((int) (GridCloudNodeMap[indX][indY]->cluster_flag));

                    marker_text.text += to_string(indX) + ":" + to_string(indY) + "\n" +
                                        to_string((int) (GridCloudNodeMap[indX][indY]->Cloud->points.size())) + ":" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_elevation * 100))) +
                                        ":" +
                                        to_string(((int)(GridCloudNodeMap[indX][indY]->elev_diff_score * 100))) + "\n" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->slopeflag))) + ":" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->slopeangle * 100))) + ":" +
                                        to_string((int) (GridCloudNodeMap[indX][indY]->cluster_flag));

                    marker_array_text.markers.emplace_back(marker_text);
                    marker_text.text.clear();
                    marker_text.id += 1;
                } else if (GridCloudNodeMap[indX][indY]->Cloud->size() > minGridCloudNum) {
                    marker_text.color.r = 1;
                    marker_text.color.g = 0;
                    marker_text.color.b = 0;
                    pose_text.position.x = (indX - GridCloudHalfWidth) * GridSize;
                    pose_text.position.y = (indY - GridCloudHalfWidth) * GridSize;
                    marker_text.pose = pose_text;
//                marker_text.text += to_string(GridCloudNodeMap[indX][indY]->TopCloudnum)+":"+to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100))) + ":" +to_string((int)(GridCloudNodeMap[indX][indY]->top_linearity*100)) +":"+ to_string((int)(GridCloudNodeMap[indX][indY]->top_planarity*100));

//                marker_text.text += to_string(((int)(GridCloudNodeMap[indX][indY]->total_score*100)))+":"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100)))+ "\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->elevation_score*100)))+ ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->appear_score*100)))+ "\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->elev_diff_score*100))) + ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->planarity_score*100))) +"\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->top_planarity*100))) + ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_vec*100)));

//                marker_text.text += to_string(indX) + ":"+ to_string(indY) + "\n" + to_string((int)(GridCloudNodeMap[indX][indY]->perpendicularity*100)) + ":" +  to_string((int)(GridCloudNodeMap[indX][indY]->min_z_elev*100)) + "\n" + to_string(((int)(GridCloudNodeMap[indX][indY]->Cloud->points.size()))) + ":"+to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100)));

//                    marker_text.text += to_string(indX) + ":" + to_string(indY) + "\n" +
//                                        to_string((int) (GridCloudNodeMap[indX][indY]->Cloud->points.size())) + ":" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_elevation * 100))) +
//                                        ":" +
//                                        to_string(((GridCloudNodeMap[indX][indY]->slopeflag))) + "\n" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_vec * 100))) + ":" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->elev_diff_score * 100))) + ":" +
//                                        to_string((int) (GridCloudNodeMap[indX][indY]->cluster_flag));


                    marker_text.text += to_string(indX) + ":" + to_string(indY) + "\n" +
                                        to_string((int) (GridCloudNodeMap[indX][indY]->Cloud->points.size())) + ":" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_elevation * 100))) +
                                        ":" +
                                        to_string(((int)(GridCloudNodeMap[indX][indY]->elev_diff_score * 100))) + "\n" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->slopeflag))) + ":" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->slopeangle * 100))) + ":" +
                                        to_string((int) (GridCloudNodeMap[indX][indY]->cluster_flag));

                    marker_array_text.markers.emplace_back(marker_text);
                    marker_text.text.clear();
                    marker_text.id += 1;
                } else {
                    marker_text.color.r = 0.3;
                    marker_text.color.g = 0.4;
                    marker_text.color.b = 0.4;
                    pose_text.position.x = (indX - GridCloudHalfWidth) * GridSize;
                    pose_text.position.y = (indY - GridCloudHalfWidth) * GridSize;
                    marker_text.pose = pose_text;
//                marker_text.text += to_string(GridCloudNodeMap[indX][indY]->TopCloudnum)+":"+to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100))) + ":" +to_string((int)(GridCloudNodeMap[indX][indY]->top_linearity*100)) +":"+ to_string((int)(GridCloudNodeMap[indX][indY]->top_planarity*100));

//                marker_text.text += to_string(((int)(GridCloudNodeMap[indX][indY]->total_score*100)))+":"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100)))+ "\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->elevation_score*100)))+ ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->appear_score*100)))+ "\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->elev_diff_score*100))) + ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->planarity_score*100))) +"\n"+
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->top_planarity*100))) + ":" +
//                                    to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_vec*100)));

//                marker_text.text += to_string(indX) + ":"+ to_string(indY) + "\n" + to_string((int)(GridCloudNodeMap[indX][indY]->perpendicularity*100)) + ":" +  to_string((int)(GridCloudNodeMap[indX][indY]->min_z_elev*100)) + "\n" + to_string(((int)(GridCloudNodeMap[indX][indY]->Cloud->points.size()))) + ":"+to_string(((int)(GridCloudNodeMap[indX][indY]->ground_z_elevation*100)));

//                    marker_text.text += to_string(indX) + ":" + to_string(indY) + "\n" +
//                                        to_string((int) (GridCloudNodeMap[indX][indY]->Cloud->points.size())) + ":" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_elevation * 100))) +
//                                        ":" +
//                                        to_string(((GridCloudNodeMap[indX][indY]->slopeflag))) + "\n" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_vec * 100))) + ":" +
//                                        to_string(((int) (GridCloudNodeMap[indX][indY]->elev_diff_score * 100))) + ":" +
//                                        to_string((int) (GridCloudNodeMap[indX][indY]->cluster_flag));


                    marker_text.text += to_string(indX) + ":" + to_string(indY) + "\n" +
                                        to_string((int) (GridCloudNodeMap[indX][indY]->Cloud->points.size())) + ":" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->ground_z_elevation * 100))) +
                                        ":" +
                                        to_string(((int)(GridCloudNodeMap[indX][indY]->elev_diff_score * 100))) + "\n" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->slopeflag))) + ":" +
                                        to_string(((int) (GridCloudNodeMap[indX][indY]->slopeangle * 100))) + ":" +
                                        to_string((int) (GridCloudNodeMap[indX][indY]->cluster_flag));

                    marker_array_text.markers.emplace_back(marker_text);
                    marker_text.text.clear();
                    marker_text.id += 1;
                }
            }
        }

        pub_MarkerArrayFeature_.publish(marker_array_text);
    }
}


#endif

/*

void ObjSegGrid_obj::ComputeBoundingCropHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr BoundCloud,
                                             int flag_intensity) {

    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = "camera_init";
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 1.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.scale.x = 0.2;
    bbox_marker.scale.y = 0.2;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::POINTS;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    bbox_marker.id = flag_intensity;

    pcl::PointXYZI ptxyzi;
    geometry_msgs::Point thisgeopt;

    for (int i = 0; i < BoundCloud->points.size(); i++)
        BoundCloud->points[i].z = 0;

    if (BoundCloud->points.size() >= 3) {
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(BoundCloud);//载入点云
        hull.setDimension(2);//设置凸包维度为2维
        std::vector<pcl::Vertices> polygons;//保存凸包的容器
        pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);//描述凸包形状
        hull.reconstruct(*surface_hull, polygons);//计算凸包结果
        for (int i = 0; i < surface_hull->size(); i++) {
            thisgeopt.x = surface_hull->points[i].x;
            thisgeopt.y = surface_hull->points[i].y;
            thisgeopt.z = -SensorHigh;
            bbox_marker.points.emplace_back(thisgeopt);

            ptxyzi.x = surface_hull->points[i].x;
            ptxyzi.y = surface_hull->points[i].y;
            ptxyzi.z = -SensorHigh;
            ptxyzi.intensity = flag_intensity;
            XYBoundingCloud->emplace_back(ptxyzi);
        }
    } else {
        for (int i = 0; i < BoundCloud->size(); i++) {
            thisgeopt.x = BoundCloud->points[i].x;
            thisgeopt.y = BoundCloud->points[i].y;
            thisgeopt.z = -SensorHigh;
            bbox_marker.points.emplace_back(thisgeopt);

            ptxyzi.x = BoundCloud->points[i].x;
            ptxyzi.y = BoundCloud->points[i].y;
            ptxyzi.z = -SensorHigh;
            ptxyzi.intensity = flag_intensity;
            XYBoundingCloud->emplace_back(ptxyzi);
        }
    }


    bounding_marker_array.markers.push_back(bbox_marker);

}
*/

void ObjSegGrid_obj::RemoveGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr BoundCloud,
                                  pcl::PointXYZ CenterPoint) {

//    cout<<"boundcloud_size: "<<BoundCloud->points.size()<<endl;

    float& x = CenterPoint.x;
    float& y = CenterPoint.y;
    float& z = CenterPoint.z;

//    //斜坡点云不去除草地
//    if(GridCloudNodeMap[int(x)][int(y)]->slopeflag == 1){
//        return;
//    }

    int cloudsize = BoundCloud->points.size();
    sort(BoundCloud->begin(), BoundCloud->end(), [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){return  p1.z < p2.z;});
    float elev_diff = BoundCloud->points[cloudsize-1].z - BoundCloud->points[0].z;

    if (HighGrass == false) {
        pcl::PassThrough<pcl::PointXYZ> passthrough;

        float z_elev_resolution;
        if(plane){
            z_elev_resolution = 0.02;
        }
        if(Grass){
            z_elev_resolution = 0.02;
        }

//        float grass_elev_max_high = 0.2;//草高最高19cm
        pcl::PointCloud<pcl::PointXYZ>::Ptr z_elev_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr BoundCloudcopy(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*BoundCloud,*BoundCloudcopy);

        pcl::PointXYZ minpoint,maxpoint;
        map<int,float> grass_elev_area_count;
        map<int,int> grass_elev_count;
        float grass_high_filter;

        float min_z_elev = BoundCloud->points[0].z;
//        cout<<"elev_diff: "<<elev_diff<<endl;

        ///simple v1
//        //todo 用平面拟合来提取地面
//        if(elev_diff < min_z_elevation){
//            grass_high_filter = min_z_elev + grasshigh_real;
//
//        }else{
//            grass_high_filter = min_z_elev + grasshigh_mid;
//
//        }


        ///use ransac v2
//        for(auto it = BoundCloud->points.begin();it != BoundCloud->points.end();it++){
//            if(it->z < min_z_elevation){
//                z_elev_cloud->points.emplace_back(*it);
//            }else{
//                break;
//            }
//        }
//        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//        pcl::SACSegmentation<pcl::PointXYZ> sac2d;
//        sac2d.setInputCloud(z_elev_cloud);
//        sac2d.setMethodType(pcl::SAC_RANSAC);
//        sac2d.setModelType(pcl::SACMODEL_PLANE);
//        sac2d.setDistanceThreshold(grasshigh_real);
//        sac2d.setMaxIterations(500);
//        sac2d.setProbability(0.5);
//        sac2d.setOptimizeCoefficients(true);
//        sac2d.segment(*inliers, *coefficients); // 提取出斜坡点的索引和斜坡系数
//        sac2d.setOptimizeCoefficients(true);
//        sac2d.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
//        sac2d.setEpsAngle(pcl::deg2rad(30.0));
//
////        planecoeffArray.emplace_back(coefficients);
//
//
//        float nx = coefficients->values[0];
//        float ny = coefficients->values[1];
//        float nz = coefficients->values[2];
//        float d = coefficients->values[3];//nx*X+ny*Y+nz*Z+d=0
//        if (nz < 0) {
////                cout<<"nx: "<<nx<<"ny: "<<ny<<"nz: "<<nz<<"d: "<<d<<endl;
//            nx = -nx;
//            ny = -ny;
//            nz = -nz;
//            d = -d;
////                cout<<"after "<<"nx: "<<nx<<"ny: "<<ny<<"nz: "<<nz<<"d: "<<d<<endl;
//
//        }
//        float r1 = sqrt(nx * nx + ny * ny + nz * nz);
//
//        BoundCloud->clear();
//        float value;
//        for(auto it = BoundCloudcopy->points.begin();it != BoundCloudcopy->points.end();it++){
//            value = (nx * it->x +
//                     ny * it->y +
//                     nz * it->z + d) / r1;
//
//            if(value > grasshigh_real){
//                BoundCloud->points.emplace_back(*it);
//            }
//        }


        ///complex but not robust v3
        ///complex but not robust v3.1 对过高的障碍物按照高度一定比例直接去除底部
        float max_area_grass_high,max_pointnum_grass_high;
        if(elev_diff > min_z_elevation && y > 3.5){
//            if(y > 3.5 && x > -1.5 && x < 1.5){
//                //去除95% grasshigh_mid 99% grasshigh_3sigma
//                grass_high_filter = min_z_elev + grasshigh_mid;
//            }
//            else{
                //直接对过高障碍物按一定比例去除草地，这样不会因为草地分层导致去除地面不干净
                grass_high_filter = min_z_elev + 0.1*elev_diff;
//            }


//            cout<<"elev_diff small than min_z_elevation"<<endl;

        }
        else{
//        cout<<"min_z_elev : "<<min_z_elev<<endl;
        passthrough.setInputCloud(BoundCloud);//输入点云
        for(float z_elev_lim = min_z_elev;z_elev_lim< min_z_elev+min_z_elevation;z_elev_lim +=z_elev_resolution){
            passthrough.setFilterFieldName("z");//对z轴进行操作
            passthrough.setFilterLimits(z_elev_lim, z_elev_lim + z_elev_resolution);//设置直通滤波器操作范围
//            cout<<"z_elev_lim: "<<z_elev_lim<<" z_elev_lim + z_elev_resolution: "<<z_elev_lim + z_elev_resolution<<endl;

//    passthrough.setFilterLimitsNegative(true);//true表示保留范围外，false表示保留范围内
                passthrough.filter(*z_elev_cloud);//

                pcl::getMinMax3D(*z_elev_cloud, minpoint, maxpoint); //在聚类的坐标系中计算最大最小值

                float area = (maxpoint.x - minpoint.x) * (maxpoint.y - minpoint.y);
                int key = int(100*z_elev_resolution) * ceil((z_elev_lim + z_elev_resolution) / z_elev_resolution);
//            cout<<"area key: "<<key<<" pointsize: "<<z_elev_cloud->points.size()<<endl;
                if (z_elev_cloud->points.size() > 0) {
                    grass_elev_area_count[key] = area;
//                    cout<<"grass_elev_area_count key: "<<key<<" area: "<<area<<endl;
                }
            }

            //找出面积最大的区域
            vector<pair<int, float>> z_elev_area_vector(grass_elev_area_count.begin(), grass_elev_area_count.end());
            sort(z_elev_area_vector.begin(), z_elev_area_vector.end(),
                 [](const pair<int, float> &p1, const pair<int, float> &p2) { return p1.second > p2.second; });

            for (auto &map_value: grass_elev_area_count) {
//            cout<<"area key: "<<map_value.first<<" value: "<<map_value.second<<endl;
//            grasshigh_real += (map_value.second/(float)valid_grasshigh_count)*(map_value.first*0.01);
            }
            max_area_grass_high = (z_elev_area_vector[0].first * 0.01);

            for (auto &cloudpoint: *BoundCloud) {

                if (cloudpoint.z < min_z_elev + min_z_elevation) {
                    int key = int(100*z_elev_resolution) * ceil(cloudpoint.z / z_elev_resolution);
                    grass_elev_count[key] += 1;
                }

            }

            vector<pair<int, int>> elev_value_count_vector(grass_elev_count.begin(), grass_elev_count.end());
            sort(elev_value_count_vector.begin(), elev_value_count_vector.end(),
                 [](const pair<int, int> &p1, const pair<int, int> &p2) { return p1.second > p2.second; });

            max_pointnum_grass_high = (elev_value_count_vector[0].first * 0.01);

            //todo 分层判断
//            for(auto pointnum_grass_high_value: elev_value_count_vector){
//                cout<<"pointnum_grass high: "<<pointnum_grass_high_value.first<<"pointnum: "<<pointnum_grass_high_value.second<<endl;
//            }
//            cout<<"z_elev_area_vector: "<<z_elev_area_vector.size()<<endl;
//            for(auto z_elev_area_vector_value: z_elev_area_vector){
//                cout<<"z_elev_area_vector high: "<<z_elev_area_vector_value.first<<"z_elev_area_vector num: "<<z_elev_area_vector_value.second<<endl;
//            }


            if(plane){
                //障碍物最小5cm高
                if(elev_diff < 0.05){
                    grass_high_filter = min_z_elev+0.03;

                }else{
                    grass_high_filter = min_z_elev+0.04;

                }

//                if(elev_diff < 0.05){
//                    grass_high_filter = min_z_elev+0.01;
//
//                }else if(elev_diff < 0.1){
//                    grass_high_filter = min_z_elev+grasshigh_real;
//
//                }
//                else{
//                    grass_high_filter = 0.5 * max_area_grass_high + 0.5 * max_pointnum_grass_high+grasshigh_real;
//                }

            }else{
                if(elev_diff < relative_diff_thr){
                    if(sqrt(y*y + x*x) < 1){
                        grass_high_filter = min_z_elev + grasshigh_3sigma;

                    }else if(sqrt(y*y + x*x) < 2){
                        grass_high_filter = 0.5 * max_area_grass_high + 0.5 * max_pointnum_grass_high;
                    }
                    else{
                        grass_high_filter = 0.5 * max_area_grass_high + 0.5 * max_pointnum_grass_high+grasshigh_real;

                    }
                }else{
                    grass_high_filter = 0.5 * max_area_grass_high + 0.5 * max_pointnum_grass_high+grasshigh_mid;

                }

            }

//            grass_high_filter = max_area_grass_high + grasshigh_mid;
//            float grass_high_filter = 0.5 * max_area_grass_high + 0.5 * max_pointnum_grass_high;
//        if (is_print_message_) cout<<"max_area_grass_high :"<<max_area_grass_high<<"max_pointnum_grass_high :"<<max_pointnum_grass_high<<"grass_high_filter:"<<grass_high_filter<<endl;
        }

        if(grass_high_filter > min_z_elev + min_z_elevation){
            grass_high_filter = min_z_elev + min_z_elevation;
        }


        if (is_print_message_){
            cout <<"min_z_elev: "<<min_z_elev<< "grass_high_filter: " << grass_high_filter << "max_area_grass_high: " << max_area_grass_high
                 << "max_pointnum_grass_high: " << max_pointnum_grass_high << "grasshigh_mid: " << grasshigh_mid << endl;
        }


        //过滤地面点
        passthrough.setInputCloud(BoundCloud);//输入点云
        passthrough.setFilterFieldName("z");//对z轴进行操作
        passthrough.setFilterLimits(grass_high_filter, 10);//设置直通滤波器操作范围
//    passthrough.setFilterLimitsNegative(true);//true表示保留范围外，false表示保留范围内
        passthrough.filter(*BoundCloud);//
    }


}
void ObjSegGrid_obj::ComputeEmptyHoleBound(){

    if(isslope){
        return;
    }

    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = "camera_init";
    bbox_marker.ns = "";
    bbox_marker.color.r = 1.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;


    //发布空栅格边界
    pcl::PointCloud<pcl::PointXYZ>::Ptr emptyholeobjcloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ tmp_pt;
    pcl::PointCloud<pcl::PointXYZI>::Ptr emptyholeobjcloudintensity(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointXYZI tmp_pti;

//    pcl::PointCloud<pcl::PointXYZI>::Ptr emptyholeobjcloud(new pcl::PointCloud<pcl::PointXYZI>());
//    pcl::PointXYZI tmp_pt;
//    tmp_pt.intensity = 1;

//    int flag_intensity = emptyholestartlabel;
//    cout<<"AllEmptyneighbour holesize: "<<AllEmptyneighbour.size()<<endl;
    int count=0;
    for(auto member = AllEmptyneighbour.begin();member != AllEmptyneighbour.end();member++){
        int empty_label = member->first;
        set<pair<int,int>> emptyholelist = member->second;
//        cout<<" empty_label: "<<empty_label<<" AllEmptyneighbour count: "<<++count<<" emptyholelist size: "<<emptyholelist.size()<<endl;

        emptyholeobjcloud->clear();
        if(emptyholelist.size() <2){
            continue;
        }

        int vectorsize = emptyholelist.size();

//        for(int i =0;i<vectorsize;i++){
          for(auto it = emptyholelist.begin(); it != emptyholelist.end();it++){


//            int indx = emptyholelist[i].first;
//            int indy = emptyholelist[i].second;
              int indx = it->first;
              int indy = it->second;

            tmp_pti.intensity = empty_label;

            float centerx = (indx - GridCloudHalfWidth)*GridSize;
            float centery = (indy - GridCloudHalfWidth)*GridSize;

            //将点加到ObjectCloud里面
//            tmp_pti.x = centerx + GridSize*0.5;tmp_pti.y = centery + GridSize*0.5; tmp_pti.z = -SensorHigh;
//            ObjPointCloud->points.emplace_back(tmp_pti);
//
//            tmp_pti.x = centerx - GridSize*0.5;tmp_pti.y = centery - GridSize*0.5; tmp_pti.z = -SensorHigh;
//            ObjPointCloud->points.emplace_back(tmp_pti);
//            tmp_pti.x = centerx + GridSize*0.5;tmp_pti.y = centery - GridSize*0.5; tmp_pti.z = -SensorHigh;
//            ObjPointCloud->points.emplace_back(tmp_pti);
//
//            tmp_pti.x = centerx - GridSize*0.5;tmp_pti.y = centery + GridSize*0.5; tmp_pti.z = -SensorHigh;
//            ObjPointCloud->points.emplace_back(tmp_pti);

            tmp_pti.x = centerx + GridSize*0.5;tmp_pti.y = centery; tmp_pti.z = -SensorHigh;
            ObjPointCloud->points.emplace_back(tmp_pti);

            tmp_pti.x = centerx - GridSize*0.5;tmp_pti.y = centery; tmp_pti.z = -SensorHigh;
            ObjPointCloud->points.emplace_back(tmp_pti);

            tmp_pti.x = centerx;tmp_pti.y = centery - GridSize*0.5; tmp_pti.z = -SensorHigh;
            ObjPointCloud->points.emplace_back(tmp_pti);

            tmp_pti.x = centerx;tmp_pti.y = centery + GridSize*0.5; tmp_pti.z = -SensorHigh;
            ObjPointCloud->points.emplace_back(tmp_pti);


        }
//        ComputeBoundingPt(emptyholeobjcloud, flag_intensity);
//        flag_intensity++;
//        *XYBoundingCloud += *emptyholeobjcloud;

    }
//    cout<<"XYBoundingCloud size :"<<XYBoundingCloud->points.size()<<endl;

}

void ObjSegGrid_obj::AddSmallHoleObjectCloud(){



//    map_objectcloud[GridCloudNodeMap[indX][indY]->cluster_flag] = this_point_cloud;
    std::map<float, pcl::PointCloud<pcl::PointXYZI>> map_objectcloud;
    pcl::PointXYZI thispoint;



    for (int indX = GridCloudHalfWidth - SensorXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + SensorXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth; indY < GridCloudHalfWidth + SensorYAxis * GridSizeInverse; indY++) {


            if (smallholeintensity.find(GridCloudNodeMap[indX][indY]->cluster_flag) != smallholeintensity.end()) {


//                map_objectcloud[GridCloudNodeMap[indX][indY]->cluster_flag] = this_point_cloud;

//                if (map_objectcloud.find(GridCloudNodeMap[indX][indY]->cluster_flag) != map_objectcloud.end()) {
//                    //已经存储过了key
//
//                }
                for (int i = 0; i < GridCloudNodeMap[indX][indY]->Cloud->points.size(); i++) {
                    thispoint.x = GridCloudNodeMap[indX][indY]->Cloud->points[i].x;
                    thispoint.y = GridCloudNodeMap[indX][indY]->Cloud->points[i].y;
                    thispoint.z = GridCloudNodeMap[indX][indY]->Cloud->points[i].z;
                    thispoint.intensity = clusterlabel;
//                    ObjPointCloud->points.emplace_back(thispoint);
                    map_objectcloud[GridCloudNodeMap[indX][indY]->cluster_flag].points.emplace_back(thispoint);
                }

            }

        }
        for (auto it = map_objectcloud.begin(); it != map_objectcloud.end(); it++) {
            *ObjPointCloud += it->second;
        }


    }



}

//计算聚类后障碍物的Marker和四个角点点云
void ObjSegGrid_obj::PubBoundingBox() {
    bounding_marker_array.markers.clear();
    bbox_marker_array.markers.clear();

    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = "camera_init";
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 1.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    pcl::PointXYZ CenterPoint, temppoint;

//    size_t CntNum = 0;
//    int marker_id = 1;
    pcl::PointXYZ min; //xyz的最小值
    pcl::PointXYZ max; //xyz的最大值
    static size_t max_marker_size_ = 0;

//    AddSmallHoleObjectCloud();
    ComputeEmptyHoleBound();

    // point_transform.topLeftCorner<3,3>() = point_transform.topLeftCorner<3,3>().inverse();
//    cout<<"ObjPointCloudsize: "<<ObjPointCloud->points.size()<<endl;
    if (ObjPointCloud->points.size() == 0) {



        //防止无障碍物的时候最后一个框还存在,可视化代码
//      for (int i = 1; i <= max_marker_size_; ++i)
//      {
//        bbox_marker.id = i;
//        bbox_marker.color.a = 0;
//        bbox_marker.pose.position.x = 0;
//        bbox_marker.pose.position.y = 0;
//        bbox_marker.pose.position.z = 0;
//        bbox_marker.scale.x = 0;
//        bbox_marker.scale.y = 0;
//        bbox_marker.scale.z = 0;
//        bbox_marker_array.markers.push_back(bbox_marker);
//        ++marker_id;
//      }
//      pub_MarkerArray_.publish(bbox_marker_array);
        return;
    }


    //  cout<<"---------------bbox_position_clusteredBox---------------------"<<endl;
    pcl::PointXYZI temppt;
    temppt.x = 0;
    temppt.y = 0;
    temppt.z = 0;
    temppt.intensity = 999;
    ObjPointCloud->points.emplace_back(temppt);//最后插入一个点，防止最后一个障碍物块提取不出来

    int flag_intensity = ObjPointCloud->begin()->intensity;
//    int flag_intensity = emptyholestartlabel;

    size_t cntcloudsize = 0;
    TempCloud->clear();
    for (auto iter = ObjPointCloud->begin(); iter != ObjPointCloud->end(); iter++) {
        if (iter->intensity == flag_intensity) {

            if (iter->x == 0 && iter->y == 0 && iter->z == 0) {
                continue;
            }

//            SumPoint.x += iter->x;
//            SumPoint.y += iter->y;
//            SumPoint.z += iter->z;

            temppoint.x = iter->x;
            temppoint.y = iter->y;
            temppoint.z = iter->z;

            TempCloud->points.emplace_back(temppoint);

//            CntNum++;
            cntcloudsize++;
            continue;
        }
//        cout<<"flag_intensity: "<<flag_intensity<<" TempCloud size: "<<TempCloud->points.size()<<endl;
        pcl::getMinMax3D(*TempCloud, min, max); //在聚类的坐标系中计算最大最小值

        CenterPoint.x = (max.x+min.x)/2;
        CenterPoint.y = (max.y+min.y)/2;
        CenterPoint.z = (max.z+min.z)/2;


        if(flag_intensity < emptyholestartlabel){
            //只有非空洞栅格才需要去地面
//            if (slopeclusterflag.find(flag_intensity) == slopeclusterflag.end() &&
//                smallholeintensity.find(flag_intensity) == smallholeintensity.end()) {
//                //如果是斜坡栅格或者小凹坑，则也不去除地面
//                if(is_print_message_) cout<<"RemoveGround  intensity: "<<flag_intensity<<endl;
//                RemoveGround(TempCloud,CenterPoint);
//
//            }

            if (TempCloud->points.size() < minGridCloudNum) {
                flag_intensity = iter->intensity;
                iter--;
//        marker_id++;
//                CntNum = 0;
//                SumPoint.x = 0;
//                SumPoint.y = 0;
//                SumPoint.z = 0;
                TempCloud->clear();
                continue;
            }
        }



        for (int j = 0; j < TempCloud->points.size(); j++) {
            temppt.x = TempCloud->points[j].x;
            temppt.y = TempCloud->points[j].y;
            temppt.z = TempCloud->points[j].z;
            temppt.intensity = flag_intensity;
            ObjCloudNoGround->points.emplace_back(temppt);
        }

//        cout<<"TempCloud1:"<<TempCloud->points[0].x<<" "<<TempCloud->points[0].y<<" "<<TempCloud->points[0].z<<endl;
        // pcl::transformPointCloud(*TempCloud, *TempCloud, point_transform);
//        cout<<"TempCloud2:"<<TempCloud->points[0].x<<" "<<TempCloud->points[0].y<<" "<<TempCloud->points[0].z<<endl;

        ComputeBoundingPt(TempCloud, flag_intensity);
//    ComputeBoundingCropHull(TempCloud,flag_intensity);




        bbox_marker.id = flag_intensity;
        bbox_marker.color.a = 0.5;
//        bbox_marker.pose.position.x = SumPoint.x / CntNum;
//        bbox_marker.pose.position.y = SumPoint.y / CntNum;
        bbox_marker.pose.position.x = (max.x + min.x)/2;
        bbox_marker.pose.position.y = (max.y + min.y)/2;

        if(smallholeintensity.find(flag_intensity) != smallholeintensity.end()){
            //凹坑的bbox
            bbox_marker.pose.position.z = -1;
            bbox_marker.color.g = 1.0f;
//            cout<<"smallholeintensity flag_intensity :"<<flag_intensity<<endl;

        }
        else if(flag_intensity >= emptyholestartlabel){
            //空栅格bbox
            bbox_marker.pose.position.z = -2;
            bbox_marker.color.g = 0.5f;
            bbox_marker.color.r = 0.5f;
//            cout<<"emptyholestartlabel flag_intensity :"<<flag_intensity<<endl;

        }
        else{
            //点云障碍物bbox
//            bbox_marker.pose.position.z = SumPoint.z / CntNum;
            bbox_marker.pose.position.z = (max.z + min.z)/2;
//            cout<<"normal flag_intensity :"<<flag_intensity<<endl;

        }

        bbox_marker.scale.x = max.x - min.x;
        bbox_marker.scale.y = max.y - min.y;
        bbox_marker.scale.z = max.z - min.z;
        bbox_marker.header.stamp = laser_Stamp.stamp;
        bbox_marker_array.markers.emplace_back(bbox_marker);
        //  cout<<"lidarFrame_bbox_id:"<<marker_id<<"  x:"<< bbox_marker.pose.position.x<<" y:"<<bbox_marker.pose.position.y<<endl;

        flag_intensity = iter->intensity;
        iter--;
//        marker_id++;
//        CntNum = 0;
//        SumPoint.x = 0;
//        SumPoint.y = 0;
//        SumPoint.z = 0;
        TempCloud->clear();
    }
    //发布没有空占位的框
//    pub_MarkerArray_.publish(bbox_marker_array);
//    XYBoundingMarkArray_pub_.publish(bounding_marker_array);


    pcl::PointCloud<pcl::PointXYZI>::iterator index3 = ObjPointCloud->end();
    index3--;
    ObjPointCloud->erase(index3);

//    if (bbox_marker_array.markers.size() > max_marker_size_)
//    {
//      max_marker_size_ = bbox_marker_array.markers.size();
//    }
//    else
//    {
//      for (int i = marker_id; i <= max_marker_size_; ++i)
//      {
//        bbox_marker.id = i;
//        bbox_marker.color.a = 0;
//        bbox_marker.pose.position.x = 0;
//        bbox_marker.pose.position.y = 0;
//        bbox_marker.pose.position.z = 0;
//        bbox_marker.scale.x = 0;
//        bbox_marker.scale.y = 0;
//        bbox_marker.scale.z = 0;
//        bbox_marker_array.markers.push_back(bbox_marker);
//        ++marker_id;
//      }
//    }
//
//    //发布有空占位的框，用于可视化
//      pub_MarkerArray_.publish(bbox_marker_array);
}



//计算聚类后障碍物的Marker和四个角点点云
void ObjSegGrid_obj::PubBoundingBox_WORLD() {
    bounding_marker_array.markers.clear();
    bbox_marker_array.markers.clear();

    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = "camera_init";   // "zvision_lidar";
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 0.0f;
    bbox_marker.color.b = 1.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    pcl::PointXYZ CenterPoint, temppoint;

//    size_t CntNum = 0;
//    int marker_id = 1;
    pcl::PointXYZ min; //xyz的最小值
    pcl::PointXYZ max; //xyz的最大值
    static size_t max_marker_size_ = 0;

//    AddSmallHoleObjectCloud();
    ComputeEmptyHoleBound();

//    cout<<"ObjPointCloudsize: "<<ObjPointCloud->points.size()<<endl;
    if (ObjPointCloud->points.size() == 0) {
        return;
    }


    //  cout<<"---------------bbox_position_clusteredBox---------------------"<<endl;
    pcl::PointXYZI temppt;
    temppt.x = 0;
    temppt.y = 0;
    temppt.z = 0;
    temppt.intensity = 999;
    ObjPointCloud->points.emplace_back(temppt);//最后插入一个点，防止最后一个障碍物块提取不出来

    //////////////////////////////////////////
    // // 对障碍物点云进行处理，首先变换到IMU坐标系中
    // Eigen::Matrix4d I_T_L;
    // I_T_L << 1, 0, 0, -0.011,
    //    0, -1, 0, -0.04171,
    //    0, 0, -1, -0.05388,
    //    0, 0, 0, 1;
    // // cerr << "I_T_L: " << I_T_L << endl;
    // pcl::transformPointCloud(*ObjPointCloud, *ObjPointCloud, I_T_L);    
    // 然后变换到栅格地图坐标系中
    pcl::transformPointCloud(*ObjPointCloud, *ObjPointCloud, Pose_trans);
    /////////////////////////////////////////////////

    int flag_intensity = ObjPointCloud->begin()->intensity;
//    int flag_intensity = emptyholestartlabel;

    size_t cntcloudsize = 0;
    TempCloud->clear();
    for (auto iter = ObjPointCloud->begin(); iter != ObjPointCloud->end(); iter++) {
        if (iter->intensity == flag_intensity) {

            if (iter->x == 0 && iter->y == 0 && iter->z == 0) {
                continue;
            }

            temppoint.x = iter->x;
            temppoint.y = iter->y;
            temppoint.z = iter->z;

            TempCloud->points.emplace_back(temppoint);

//            CntNum++;
            cntcloudsize++;
            continue;
        }
//        cout<<"flag_intensity: "<<flag_intensity<<" TempCloud size: "<<TempCloud->points.size()<<endl;
        pcl::getMinMax3D(*TempCloud, min, max); //在聚类的坐标系中计算最大最小值

        CenterPoint.x = (max.x+min.x)/2;
        CenterPoint.y = (max.y+min.y)/2;
        CenterPoint.z = (max.z+min.z)/2;


        if(flag_intensity < emptyholestartlabel){


            if (TempCloud->points.size() < minGridCloudNum) {
                flag_intensity = iter->intensity;
                iter--;
                TempCloud->clear();
                continue;
            }
        }



        for (int j = 0; j < TempCloud->points.size(); j++) {
            temppt.x = TempCloud->points[j].x;
            temppt.y = TempCloud->points[j].y;
            temppt.z = TempCloud->points[j].z;
            temppt.intensity = flag_intensity;
            ObjCloudNoGround->points.emplace_back(temppt);
        }

        ComputeBoundingPt(TempCloud, flag_intensity);
//    ComputeBoundingCropHull(TempCloud,flag_intensity);




        bbox_marker.id = flag_intensity;
        bbox_marker.color.a = 0.5;
//        bbox_marker.pose.position.x = SumPoint.x / CntNum;
//        bbox_marker.pose.position.y = SumPoint.y / CntNum;
        bbox_marker.pose.position.x = (max.x + min.x)/2;
        bbox_marker.pose.position.y = (max.y + min.y)/2;

        if(smallholeintensity.find(flag_intensity) != smallholeintensity.end()){
            //凹坑的bbox
            bbox_marker.pose.position.z = -1;
            bbox_marker.color.g = 1.0f;
//            cout<<"smallholeintensity flag_intensity :"<<flag_intensity<<endl;

        }
        else if(flag_intensity >= emptyholestartlabel){
            //空栅格bbox
            bbox_marker.pose.position.z = -2;
            bbox_marker.color.g = 0.5f;
            bbox_marker.color.r = 0.5f;
//            cout<<"emptyholestartlabel flag_intensity :"<<flag_intensity<<endl;

        }
        else{
            //点云障碍物bbox
//            bbox_marker.pose.position.z = SumPoint.z / CntNum;
            bbox_marker.pose.position.z = (max.z + min.z)/2;
//            cout<<"normal flag_intensity :"<<flag_intensity<<endl;

        }

        bbox_marker.scale.x = max.x - min.x;
        bbox_marker.scale.y = max.y - min.y;
        bbox_marker.scale.z = max.z - min.z;
        bbox_marker.header.stamp = laser_Stamp.stamp;
        bbox_marker_array.markers.emplace_back(bbox_marker);
        //  cout<<"lidarFrame_bbox_id:"<<marker_id<<"  x:"<< bbox_marker.pose.position.x<<" y:"<<bbox_marker.pose.position.y<<endl;

        flag_intensity = iter->intensity;
        iter--;
        TempCloud->clear();
    }

    pcl::PointCloud<pcl::PointXYZI>::iterator index3 = ObjPointCloud->end();
    index3--;
    ObjPointCloud->erase(index3);
}


/*


//计算每个栅格的障碍物框
void ObjSegGrid_obj::PubBoundingBox2(std::string ObjectType) {
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = "camera_init";
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    visualization_msgs::MarkerArray marker_array;
//    pcl::PointCloud<pcl::PointXYZ> TempCloud;
//    pcl::PointXYZ SumPoint, temppoint;
//    size_t CntNum = 0;
    int marker_id = 1;
    pcl::PointXYZ min; //xyz的最小值
    pcl::PointXYZ max; //xyz的最大值
//    static size_t max_marker_size_ = 0;

    if (ObjPointCloud->points.size() == 0) {



 cout<<"empty--------------------"<<endl;
      //防止无障碍物的时候最后一个框还存在
      for (int i = 1; i <= max_marker_size_; ++i)
      {
        bbox_marker.id = i;
        bbox_marker.color.a = 0;
        bbox_marker.pose.position.x = 0;
        bbox_marker.pose.position.y = 0;
        bbox_marker.pose.position.z = 0;
        bbox_marker.scale.x = 0;
        bbox_marker.scale.y = 0;
        bbox_marker.scale.z = 0;
        marker_array.markers.push_back(bbox_marker);
        ++marker_id;
      }
      pub_MarkerArray_vis_.publish(marker_array);

        return;
    }


    // cout<<"---------------bbox_position_GridBox---------------------"<<endl;
    //        fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.label, obj.prob,
//                obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);
    Eigen::Vector4f centeroid;
    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = (GridCloudWidth - 1) / 2; indY < GridCloudWidth; indY++) {
            // 如果flag==1（有障碍物）
            if (GridCloudNodeMap[indX][indY]->cluster_flag >= 1) {
                pcl::getMinMax3D(*GridCloudNodeMap[indX][indY]->Cloud, min, max);
                pcl::compute3DCentroid(*GridCloudNodeMap[indX][indY]->Cloud, centeroid);
                bbox_marker.id = marker_id;
                bbox_marker.color.a = 0.5;
                bbox_marker.pose.position.x = centeroid(0);
                bbox_marker.pose.position.y = centeroid(1);
                bbox_marker.pose.position.z = centeroid(2);
                bbox_marker.scale.x = max.x - min.x;
                bbox_marker.scale.y = max.y - min.y;
                bbox_marker.scale.z = max.z - min.z;
                bbox_marker.header.stamp = laser_Stamp.stamp;
                marker_array.markers.emplace_back(bbox_marker);
                // cout<<"lidarFrame_bbox_id:"<<marker_id<<"  x:"<< bbox_marker.pose.position.x<<" y:"<<bbox_marker.pose.position.y<<endl;
                marker_id++;

            }
        }
    }
    pub_MarkerArrayGrid_.publish(marker_array);
}

*/

void ObjSegGrid_obj::ClearAll() {
//      Update_appear_score();

    for (int i = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         i < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; i++)
        for (int j = GridCloudHalfWidth - lidarYAxis * GridSizeInverse; j < GridCloudWidth; j++)
            GridCloudNodeMap[i][j]->ClearCloud();


    for(pcl::PointCloud<pcl::PointXYZ>::Ptr &element : CloudVector){
        element->clear();
    }

//    for (int indX = 0;
//         indX < GridCloudWidth; indX++) {
//        for (int indY = (GridCloudWidth - 1) / 2 - 10; indY < GridCloudWidth; indY++) {
//            GridCloudNodeMap[indX][indY]->ClearCloud();
//
//        }
//    }
//    for(auto it = slopeclusterflag.begin();it != slopeclusterflag.end();it++){
//        cout<<"slopeflag: "<<*it<<endl;
//    }
    slopeclusterflag.clear();
    smallholeintensity.clear();

     laserCloudCrop->clear();
    // laserCloudTemp->clear();
    // laserCloudTemp_->clear();
    ground_cloud->clear();
    total_ground_cloud->clear();
    ObjPointCloud->clear();
    // CameraDeepCloud->clear();
//    tmpCloud1->clear();
//    tmpCloud2.clear();
    TempCloud->clear();
    XYBoundingCloud->clear();
//    EdgeCloud->clear();
    ObjCloudNoGround->clear();
    obj_on_slopecloud->clear();
    slopecloud->clear();


#ifdef Debug
    UnderGroundCloud->clear();
    revertnoisecloud->clear();
    noise_cloud->clear();
    mismatchObjectCloud->clear();


#endif

}

void ObjSegGrid_obj::PublishCloud() {

    if (run_alone){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0, 0, 0));
        transform.setRotation(tf::createIdentityQuaternion());
        br.sendTransform(tf::StampedTransform(transform, sensorCloudTemp.header.stamp, "camera_init", "livox_frame"));        
    }


    if (XYBoundingMarkArray_pub_.getNumSubscribers() != 0) {
        XYBoundingMarkArray_pub_.publish(bounding_marker_array);
    }

    if (pub_MarkerArray_.getNumSubscribers() != 0) {
        pub_MarkerArray_.publish(bbox_marker_array);
    }

    if (full_cloud_pub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*laserCloudCrop, sensorCloudTemp);
        // sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        full_cloud_pub_.publish(sensorCloudTemp);
    }


    if (XYBoundingCloud_pub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*XYBoundingCloud, sensorCloudTemp);
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        XYBoundingCloud_pub_.publish(sensorCloudTemp);
    }

    if (ObjCloudNoGroundPub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*ObjCloudNoGround, sensorCloudTemp);
        // sensorCloudTemp.header.stamp = ros::Time::now();
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        ObjCloudNoGroundPub_.publish(sensorCloudTemp);
    }

    if (ObjCloudGroundSeg.getNumSubscribers() != 0) {
        pcl::toROSMsg(*total_ground_cloud, sensorCloudTemp);
        // sensorCloudTemp.header.stamp = ros::Time::now();
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        ObjCloudGroundSeg.publish(sensorCloudTemp);
    }

#ifdef Debug

    if (ObjPointCloudPub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*ObjPointCloud, sensorCloudTemp);
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        ObjPointCloudPub_.publish(sensorCloudTemp);
    }

    if (Tempviscloud_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*slopecloud, sensorCloudTemp);
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        Tempviscloud_.publish(sensorCloudTemp);
    }

    if (pub_noisecloud_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*noise_cloud, sensorCloudTemp);
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        pub_noisecloud_.publish(sensorCloudTemp);
    }


    if (pub_mismatch_objectcloud_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*mismatchObjectCloud, sensorCloudTemp);
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        pub_mismatch_objectcloud_.publish(sensorCloudTemp);
    }


    if (revertnoisecloud_pub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*revertnoisecloud, sensorCloudTemp);
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        revertnoisecloud_pub_.publish(sensorCloudTemp);
    }


    if (UnderGroundCloudPub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*UnderGroundCloud, sensorCloudTemp);
        sensorCloudTemp.header.stamp = laser_Stamp.stamp;
        sensorCloudTemp.header.frame_id = "camera_init";
        UnderGroundCloudPub_.publish(sensorCloudTemp);
    }
#endif


}

void ObjSegGrid_obj::AddSurroundGrass() {
    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth - lidarYAxis * GridSizeInverse;
             indY < GridCloudHalfWidth + lidarYAxis * GridSizeInverse; indY++) {
            // 如果栅格内点云大于最低点云要求
            if (GridCloudNodeMap[indX][indY]->cluster_flag == 1) {
                vector<pair<int, int>> neighbour;
                // 存入当前ID到neighbour
                neighbour.emplace_back(pair<int, int>(indX, indY));

                // 如果不为空
                while (!neighbour.empty()) {
                    // 提取出这个栅格
                    pair<int, int> thisgrid = neighbour.back();
                    neighbour.pop_back();
                    int indx = thisgrid.first;
                    int indy = thisgrid.second;

                    ///todo v2 限制草地搜索范围
//                    float max_grass_clusterflag = 0.5;//最多在clusterflag周围搜索0.5m
//                    float temp_max_grass_clusterflag = 0;
//                    for (int dx = -1; dx <= 1; dx++) {
//                        for (int dy = -1; dy <= 1; dy++) {
//
//                            if (indx + dx < 0 || indx + dx > GridCloudWidth - 1 || indy + dy < 0 ||
//                                indy + dy > GridCloudWidth - 1)
//                                continue;
//
//                            //计算当前栅格的最大草地clusterflag
//                            if(GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag > 0 && GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag <= 1){
//                                if (
//                                    //GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag != 1 &&
//                                        GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag > temp_max_grass_clusterflag) {
//                                    temp_max_grass_clusterflag = GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag;
//                                }
//                            }
//                        }
//                    }

                    // 查找前后左右的8个栅格
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            // 如果这个栅格不等于1，即等于0或者2，即没有障碍物或研究遍历过。
                            // 或者就是当前这个栅格
                            // 或者待选的栅格过界
                            // 则跳过这个栅格
                            if ((dx == 0 && dy == 0) ||
                                indx + dx < 0 || indx + dx > GridCloudWidth - 1 || indy + dy < 0 ||
                                indy + dy > GridCloudWidth - 1)
                                continue;

                            ///v1 不限制草地搜索范围 否则将这个栅格加入待查找列表,但是限制最小草地高度
//                            if(GridCloudNodeMap[indx][indy]->slopeflag == 1){
//                                //斜坡周围一圈草地加进来
//                                GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = 0.5;
//                            }
                            if (Grass && GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag == 0 &&
                                GridCloudNodeMap[indx + dx][indy + dy]->ground_z_elevation > grasshigh_3sigma) //高于地面5cm或者高于99%的草地高度
                                 {//这个栅格比草地高
                                GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = 0.5;
//                                neighbour.emplace_back(pair<int, int>(indx + dx, indy + dy));

                            }
//                            else if (GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag == 0 &&
//                                       GridCloudNodeMap[indx + dx][indy + dy]->ground_z_elevation <grasshigh_3sigma) {//这个栅格比草地矮
//                                    GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = 0.5;
//                            }

                            ///todo v2限制草地搜索范围
//                            if(temp_max_grass_clusterflag == 1){
//                                //周围有草地，并且没有clusterflag=1的障碍物
//                                if(GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag == 0){
//                                    if(GridCloudNodeMap[indx + dx][indy + dy]->ground_z_elevation > grasshigh_3sigma){
//                                        GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = 1;
//                                    }else{
//                                        GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = max_grass_clusterflag;
//
//                                    }
//                                    neighbour.push_back(pair<int, int>(indx + dx, indy + dy));
//                                }
//                            }else if(temp_max_grass_clusterflag < 1 && temp_max_grass_clusterflag > 0.1 +FLT_MIN){
//                                if(GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag == 0){
//                                    if(GridCloudNodeMap[indx + dx][indy + dy]->ground_z_elevation > grasshigh_mid ){
//                                        GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = temp_max_grass_clusterflag - 0.1;
//                                    }else{
//                                        GridCloudNodeMap[indx + dx][indy + dy]->cluster_flag = 0.1;
//
//                                    }
//                                    neighbour.push_back(pair<int, int>(indx + dx, indy + dy));
//                                }
//                            }


                        }
                    }
                }
            }
        }
    }

}

void ObjSegGrid_obj::computeplaneuseSVD(const pcl::PointCloud<pcl::PointXYZ>::Ptr src,
                                        Eigen::RowVector3d &normal,
                                        float &d) {

//    if (is_print_message_) cout << "src->points.size()" << src->points.size() << endl;
    Eigen::MatrixXd m_cloud(src->points.size(), 3);
    int j = 0;
    for (auto &p : src->points) {
        m_cloud.row(j++) << p.x, p.y, p.z;
    }
    // 1、计算质心
    Eigen::RowVector3d centroid = m_cloud.colwise().mean();
    // 2、去质心
    Eigen::MatrixXd demean = m_cloud;
    demean.rowwise() -= centroid;
    // 3、SVD分解求解协方差矩阵的特征值特征向量
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(demean, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(demean, Eigen::ComputeFullV);
    Eigen::Matrix3d V = svd.matrixV();
//    Eigen::MatrixXd U = svd.matrixU();
//    Eigen::Matrix3d S = U.inverse() * demean * V.transpose().inverse();


    // 5、平面的法向量a,b,c
//    Eigen::RowVector3d normal;
    normal << V(0, 2), V(1, 2), V(2, 2);
    // 6、原点到平面的距离d
//    double d = -normal * centroid.transpose();
    d = -normal * centroid.transpose();
    // 7、获取拟合平面的参数a,b,c,d和质心x,y,z。
//    cout << normal, d, centroid;
}



void ObjSegGrid_obj::RemoveCloudWithSlopeLineAndPlaneCoff(){

    sloperemove_lock.lock();
//    cout<< "\033[33m" <<"IN_RemoveCloudWithSlopeLineAndPlaneCoff"<< "\033[0m"<<endl;

//    cout<< "\033[33m" << "hasline: " << hasline << " lineusecount: " << lineusecount << " hasplanecoff: " << hasplanecoff
//        << " planecoffusecount: " << planecoffusecount<< "\033[0m" << endl;

    static float y_axis_offset = 0;

    if ((hasline ||  lineusecount > 0) && (hasplanecoff || planecoffusecount > 0)) {

        if(hasline || hasplanecoff){
            y_axis_offset = 0;
        }
//        cout<<"\033[33m" <<"in_RemoveCloudWithSlopeLineAndPlaneCoff"<< "\033[0m"<<endl;
//        cout<<"x_min_limit: "<<x_min_limit<<"x_max_limit: "<<x_max_limit<<"y_min_limit: "<<y_min_limit<<"y_max_limit: "<<y_max_limit<<endl;
//        cout<<"hasline: "<<hasline<<" lineusecount: "<<lineusecount<<" hasplanecoff: "<<hasplanecoff<<" planecoffusecount: "<<planecoffusecount<<"SensorHigh"<<SensorHigh<<endl;

//        cout<< "\033[33m" << "hasline: " << hasline << " lineusecount: " << lineusecount << " hasplanecoff: " << hasplanecoff
//             << " planecoffusecount: " << planecoffusecount<< "\033[0m" << endl;
        slope_box_filter_.setMin(Eigen::Vector4f(x_min_limit-0.4, y_min_limit-0.2, -SensorHigh - 2, 1.0));
        slope_box_filter_.setMax(Eigen::Vector4f(x_max_limit+0.4, y_max_limit+0.2, 0.5, 1.0));

//        cout<<"hasline :"<<hasline<<endl;
//        cout<<"x_min_limit: "<<x_min_limit<<"x_max_limit: "<<x_max_limit<<"y_min_limit: "<<y_min_limit<<"y_max_limit: "<<y_max_limit<<endl;
//        if (hasline == true) {
//        cout<<"before_laserCloudCropsize: "<<laserCloudCrop->points.size()<<endl;

//        cout<<"laserCloudCrop_size: "<<laserCloudCrop->points.size()<<endl;

        slope_box_filter_.setInputCloud(laserCloudCrop);
        slope_box_filter_.setNegative(false);
        slope_box_filter_.filter(*slopecloud);
        slope_box_filter_.setNegative(true);
        slope_box_filter_.filter(*laserCloudCrop);
//        cout<<"slopecloud_size: "<<slopecloud->points.size()<<endl;
//        cout<<"after_laserCloudCropsize: "<<laserCloudCrop->points.size()<<endl;

//        cout<<"slopecloudsize: "<<slopecloud->points.size()<<endl;
//        }

//        if(UseLidar){
//            planecoff.orientation.w -=wheel_dx*0.1;//车往前开，但是四个角点却没有变，斜坡平面方程也没有更新，所以相当于斜坡上升
//
//        }else if(UseDCamera){
//            planecoff.orientation.w -=0.15;
//
//        }

        //更新平面方程
//        planecoff.orientation.y = planecoff.orientation.y - 0.1*wheel_dx;

        //增加轮速计导致的平面方程的平移，理论上应该是0.1,但是0.2的话效果更好一些，0.1更大概率产生物件
        y_axis_offset +=  0.2*wheel_dx;

        float nx = planecoff.orientation.x;
        float ny = planecoff.orientation.y;
        float nz = planecoff.orientation.z;
        float d = planecoff.orientation.w;

//        cout<<"nx: "<<nx<<" ny: "<<ny<<" nz: "<<nz<<" d: "<<d<<" wheel_dx: "<<wheel_dx<<" y_axis_offset: "<<y_axis_offset<<endl;
        if (nz < 0) {
            nx = -nx;
            ny = -ny;
            nz = -nz;
            d = -d;
        }
//         cout<< "\033[33m"<<"nx: "<<nx<<"ny: "<<ny<<"nz: "<<nz<<"d: "<<d<< "\033[0m"<<endl;




        float r1 = sqrt(nx * nx + ny * ny + nz * nz);

        float value;
        float offset = 0.1;
        for (auto &point_ptr: slopecloud->points) {

            if(point_ptr.x >= x_min_limit+offset && point_ptr.x <= x_max_limit-offset && point_ptr.y >= y_min_limit+offset && point_ptr.y <= y_max_limit-offset){
                value = (nx * point_ptr.x +
                         ny * (point_ptr.y + y_axis_offset) +
                         nz * point_ptr.z + d) / r1;

                if ((UseLidar && value > mindistoslope) ||(UseDCamera && value > mindistoslope)) {
                    obj_on_slopecloud->points.emplace_back(point_ptr);
                }
            }
        }

        //不将斜坡上的点云加入为障碍物点晕
//        *laserCloudCrop += *obj_on_slopecloud;

//        if(obj_on_slopecloud->points.size()> 20){
//            if(is_print_message_) cout<<" objects on the slope"<<endl;
//
//            for (int i = 0; i < obj_on_slopecloud->points.size(); i++) {
//                // fixme 栅格的中心点好像有偏置 将点云除以单位栅格的长度，得到栅格的id
//                int indX = floor(GridSizeInverse * (obj_on_slopecloud->points[i].x + GridSize / 2.0)) + GridCloudHalfWidth;
//                int indY = floor(GridSizeInverse * (obj_on_slopecloud->points[i].y + GridSize / 2.0)) + GridCloudHalfWidth;
//                // 如果点云不在栅格内，则跳过这个点
//                if (indY < 0 || indX < 0 || indX > GridCloudWidth - 1 ||  indY > GridCloudWidth - 1)
//                    continue;
//                // 向栅格内填充点
//                GridCloudNodeMap[indX][indY]->cluster_flag = 1;
//                GridCloudNodeMap[indX][indY]->slopeflag = 1;
//
//                GridCloudNodeMap[indX][indY]->Cloud->points.emplace_back(obj_on_slopecloud->points[i]);
//
//            }
//
//        }


        lineusecount--;
        planecoffusecount--;

//        cout<< "\033[33m"<<"lineusecount: "<<lineusecount<< "\033[0m"<<endl;
//        cout<< "\033[33m"<<"laserCloudCropsize: "<<laserCloudCrop->points.size()<< "\033[0m"<<endl;



    }

    hasline = false;
    hasplanecoff = false;

    sloperemove_lock.unlock();
    return;

}


void ObjSegGrid_obj::check_noise_postprocessv2(){

    if(isslope && UseDCamera){
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr noisecloud(new pcl::PointCloud<pcl::PointXYZ>());

    //拿出所有的噪声点
    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth - lidarYAxis * GridSizeInverse;
             indY < GridCloudHalfWidth + lidarYAxis * GridSizeInverse; indY++) {

            if(GridCloudNodeMap[indX][indY]->noiseCloud->points.size() == 0){
                continue;
            }
            *noisecloud += *GridCloudNodeMap[indX][indY]->noiseCloud;
        }
    }

    if(noisecloud->points.size() < 30){
        return;
    }
//    cout<<"noisecloudsize:"<<noisecloud->points.size()<<endl;
    //噪声点聚类
    vector<pcl::PointIndices> ece_inlier;
    search::KdTree<pcl::PointXYZ>::Ptr tree(new search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    ece.setInputCloud(noisecloud);
    if(isslope){
        ece.setClusterTolerance(0.1);
    }else{
        ece.setClusterTolerance(0.2);

    }
    ece.setMinClusterSize(30);
    ece.setMaxClusterSize(15000);
    ece.setSearchMethod(tree);
    ece.extract(ece_inlier);


    if(ece_inlier.empty()){
        return;
    }
    if (is_print_message_) cout<<"EuclideanClusterExtraction ok"<<endl;


//    cout<<"noisecloud_size: "<<noisecloud->points.size()<<endl;
//     找出聚类的最大的点云
    int cloudsize;
    //    int maxcloudsize = 0;
    int indX,indY;
    for(int i = 0;i<ece_inlier.size();i++){

        cloudsize = ece_inlier[i].indices.size();
//        cout<<"cloudsize i:"<<i<<"  "<<cloudsize<<endl;
        // 如果其中类别i的点云数大于最大点云数
        if(ece_inlier[i].indices.size() > 30){
            for(int j = 0;j<ece_inlier[i].indices.size();j++){
//                cout<<"j : "<<j<<" index: "<<ece_inlier[i].indices[j]<<endl;
                // fixme 栅格的中心点好像有偏置 将点云除以单位栅格的长度，得到栅格的id
                indX = floor(GridSizeInverse * (noisecloud->points[ece_inlier[i].indices[j]].x + GridSize / 2.0)) + GridCloudHalfWidth;
                indY = floor(GridSizeInverse * (noisecloud->points[ece_inlier[i].indices[j]].y + GridSize / 2.0)) + GridCloudHalfWidth;
                // 向栅格内填充点

                GridCloudNodeMap[indX][indY]->Cloud->points.emplace_back(noisecloud->points[ece_inlier[i].indices[j]]);

#ifdef Debug
                revertnoisecloud->points.emplace_back(noisecloud->points[ece_inlier[i].indices[j]]);
#endif

            }
        }
    }
    vector<float> lowest_point;

    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth - lidarYAxis * GridSizeInverse;
             indY < GridCloudHalfWidth + lidarYAxis * GridSizeInverse; indY++) {

            sort(GridCloudNodeMap[indX][indY]->Cloud->points.begin(),
                 GridCloudNodeMap[indX][indY]->Cloud->points.end(),
                 [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) { return p1.z < p2.z; });

            if (GridCloudNodeMap[indX][indY]->Cloud->points.size() > 0) {
                //todo 这段后面可以删掉，因为异常点都在天上，不在底下
                float base_z = GridCloudNodeMap[indX][indY]->Cloud->points[0].z;
                GridCloudNodeMap[indX][indY]->base_z = base_z;
                pcl::PointXYZ base_pt = GridCloudNodeMap[indX][indY]->Cloud->points[0];
                GridCloudNodeMap[indX][indY]->base_pt = base_pt;
//                cout<<"checkpt x:"<<GridCloudNodeMap[indX][indY]->base_pt.x<<" y: "<<GridCloudNodeMap[indX][indY]->base_pt.y<<" z: "<<GridCloudNodeMap[indX][indY]->base_pt.z<<endl;
            }

            if (GridCloudNodeMap[indX][indY]->Cloud->points.size() >= minGridCloudNum) {

                int cloudsize = GridCloudNodeMap[indX][indY]->Cloud->points.size();
                double ground_z_elevation = GridCloudNodeMap[indX][indY]->Cloud->points[cloudsize - 1].z -
                                            GridCloudNodeMap[indX][indY]->Cloud->points[0].z;

                GridCloudNodeMap[indX][indY]->ground_z_elevation = ground_z_elevation;
                GridCloudNodeMap[indX][indY]->base_z = GridCloudNodeMap[indX][indY]->Cloud->points[0].z;
                GridCloudNodeMap[indX][indY]->base_pt = GridCloudNodeMap[indX][indY]->Cloud->points[0];
                GridCloudNodeMap[indX][indY]->max_pt = GridCloudNodeMap[indX][indY]->Cloud->points[GridCloudNodeMap[indX][indY]->Cloud->points.size()-1];

            }


        }



    }
    }

void ObjSegGrid_obj::check_noise_postprocess() {

    if(isslope){
        return;
    }
    //如果障碍物周围有噪声点，那就把该噪声点认为是障碍物
    bool inverse_noise_flag = false;
    for (int indX = GridCloudHalfWidth - lidarXAxis * GridSizeInverse;
         indX < GridCloudHalfWidth + lidarXAxis * GridSizeInverse; indX++) {
        for (int indY = GridCloudHalfWidth - lidarYAxis * GridSizeInverse;
             indY < GridCloudHalfWidth + lidarYAxis * GridSizeInverse; indY++) {
            inverse_noise_flag = false;
            //首先判断这个栅格内是否包含噪声
            if(GridCloudNodeMap[indX][indY]->noiseCloud->points.size() > 0){
                //判断周围栅格，判断是否包含障碍物
                for (int dx = -3; dx <= 3; dx++) {
                    for (int dy = -3; dy <= 3; dy++) {
                        if ((dx == 0 && dy == 0) ||
                            indX + dx < 0 || indX + dx > GridCloudWidth - 1 || indY + dy < 0 ||
                            indY + dy > GridCloudWidth - 1) {
                            continue;
                        }
                        if(GridCloudNodeMap[indX +dx][indY +dy]->cluster_flag == 1
                           && GridCloudNodeMap[indX +dx][indY +dy]->slopeflag ==0){
                            inverse_noise_flag = true;
                            break;
                        }
                    }
                    if(inverse_noise_flag == true){
                        break;
                    }
                }

            }
            if(inverse_noise_flag == true){
                *GridCloudNodeMap[indX][indY]->Cloud += *GridCloudNodeMap[indX][indY]->noiseCloud;
#ifdef Debug
                *revertnoisecloud += *GridCloudNodeMap[indX][indY]->noiseCloud;
#endif
            }

        }
    }
}

void ObjSegGrid_obj::UpdateSensorHigh() {

    if(isslope){
        if(UseDCamera){
            SensorHigh = DeepCameraHigh;
        }else if(UseLidar){
            SensorHigh = LidarHigh;
        }
        return;
    }

    //更新车轮高度
    int startindx=95,endindx=105,startindy=105,endindy=106;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZ>());

    for(int indx = startindx;indx <= endindx;indx++){
        for(int indy = startindy;indy <=endindy;indy++){
            *tmpcloud += *GridCloudNodeMap[indx][indy]->Cloud;
        }
    }

    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*tmpcloud, cov, pc_mean);

//    SensorHigh = 0.5* (SensorHigh + (-pc_mean.z()) );
    //0.8^5 = 0.32 0.7^5 = 0.16
    SensorHigh = 0.7* SensorHigh + 0.3*(-pc_mean.z());
    //todo AI_01 20s 还是不能很好解决小凹坑误检
    if (is_print_message_) cout<<"\033[31;1m"<<"SensorHigh: "<<SensorHigh<<"\033[0m"<<endl;

}


/*

void ObjSegGrid_obj::ComputeHoleTheory(){

    float Length,Depth,Radius;
    float CarHigh = 0.6;
    float angle_resolution = 0.4375/180*M_PI;
    cout<<fixed<<setprecision(6)<<"angle_resolution: "<<angle_resolution<<endl;
    int col,row,max_pt;
    float incidence_depth;
    float lenth_crosswise;
    float total_pt,this_angle_length;
    float crosswise_start_angle,crosswise_end_angle;
    cout<<"Length     Radius     max_incidence_depth     max_pt"<<endl;
//    cout<<"Length     Radius     max_incidence_depth"<<endl;

    for(Length = 1;Length <= 4;Length += 0.5){
        for(Radius = 0.05;Radius <= 1;Radius += 0.01){
//            cout<<fixed<<setprecision(6)<<endl;
            total_pt = 0;
            float min_length = Length - Radius;
            float max_length = Length + Radius;

            float angle = atan2(CarHigh,min_length);
            float incidence_angle = floor(angle/angle_resolution)*angle_resolution;
            float max_incidence_depth = max_length*tan(incidence_angle);
            row = 0;
            max_pt = 0;
            if(max_incidence_depth > CarHigh){

                incidence_depth = max_incidence_depth;
            }
            while(incidence_depth > CarHigh){
                crosswise_start_angle = atan2(Radius,max_length);
//                crosswise_start_angle = floor(crosswise_start_angle/angle_resolution)*angle_resolution;
                crosswise_end_angle = atan2(-Radius,max_length);
//                crosswise_end_angle = ceil(crosswise_end_angle/angle_resolution)*angle_resolution;
                   float col = floor((crosswise_start_angle - crosswise_end_angle)/angle_resolution);

//                float this_row_pt;
//                float startRadius = max_length*tan(crosswise_start_angle);
//                crosswise_start_angle +=angle_resolution;
//                for(float crosswise_angle = crosswise_start_angle;crosswise_angle > crosswise_end_angle; crosswise_angle -= angle_resolution){
//
//                    this_angle_length = sqrt(Radius*Radius +max_length*max_length);
//                    incidence_depth = this_angle_length*tan(incidence_angle);
//
//
//
//                }


                max_pt +=col;
                row += 1;
                incidence_angle +=angle_resolution;
                incidence_depth = max_length*tan(incidence_angle);
            }
//            col = floor(2*Radius/(max_length * angle_resolution));
//            max_pt = col*row;
            if(max_incidence_depth-CarHigh > 0.07){
//                cout<<"col: "<<col<<" row: "<<row<<" max_length * angle_resolution:"<<max_length * angle_resolution<<endl;
                cout<<fixed<<setprecision(2)<<Length<<"       "<<Radius<<"       "<<max_incidence_depth-CarHigh<<"                  "<<max_pt<<" col: "<<col<<" row :"<<row<<endl;
//                cout<<fixed<<setprecision(2)<<Length<<"       "<<Radius<<"       "<<max_incidence_depth-CarHigh<<endl;

                break;
            }
        }
    }

    cout<<"************************************************"<<endl;
}

*/

void ObjSegGrid_obj::DrivableAreav2(){

        // 提取栅格内的地面和非地面
    extract_piecewiseground(*laserCloudCrop, *total_ground_cloud);

    pcl::CropBox<pcl::PointXYZ> range_filter_;
    range_filter_.setMin(Eigen::Vector4f(-2.5, 5, -SensorHigh - 2, 1.0));
    range_filter_.setMax(Eigen::Vector4f(2.5, 0, 0.5, 1.0));


}

void ObjSegGrid_obj::CalculateIMURot(){

    imu_sub_ptr_->ParseData(imu_data_buff);

    IMUData imu_data;
    Eigen::Matrix3d G_R_I;

//    while (!raw_imu_.empty()) {
//        IMUData imu_in = raw_imu_.front();
//        IMUData imuSelf = OriEst::preIntegrate(imu_in, orientation_estimator);
//        unsynced_imu_.push_back(imuSelf);
//        raw_imu_.pop_front();
//    }

//    cout<<"imu_data_buff_size:"<<imu_data_buff.size()<<endl;
    while (!imu_data_buff.empty()) {
        imu_data = imu_data_buff.front();
        double timestamp = imu_data.time;

//        const double timestamp = msg->header.stamp.toSec();
        Eigen::Vector3d acc(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z);
        Eigen::Vector3d gyro(imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z);

        OriEst::Status status = orientation_estimator->Estimate(imu_data.time, gyro, acc, &G_R_I);
        imu_data_buff.pop_front();

    }


//    Eigen::Affine3d point_transform;
//     const auto angleaxis = Eigen::AngleAxisd(G_R_I);
//     Eigen::Vector3d omega = angleaxis.axis() * angleaxis.angle();
// //    Eigen::Matrix4d point_transform;
//     point_transform.setIdentity();
//     point_transform.topLeftCorner<3,3>() = Eigen::AngleAxisd( -omega(2), Eigen::Vector3d::UnitZ()) * G_R_I;

// //    cout<<"point_transform: \n"<<point_transform<<endl;
//     pcl::transformPointCloud(*laserCloudCrop, *laserCloudCrop, point_transform);



//    Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
//    transform_2.rotate(yawAngle.toRotationMatrix().inverse());
//    pcl::transformPointCloud(*laserCloudCrop, *laserCloudCrop, transform_2);


}

// 运行函数
void ObjSegGrid_obj::ObjSegGridRun() {


    // 如果收到新数据
    if (newCameraCloud || newlaserCloud) {
        if (newCameraCloud)
            newCameraCloud = false;
        // 如果新点云，则标志位false，表示已经使用
        if (newlaserCloud)
            newlaserCloud = false;
    } else {
        return;
    }
    ClearAll();

    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();



    // 计时
    static float avg_ms = 0;
    static float avg_proj = 0;
    static float avg_feature = 0;
    static float avg_cluster = 0;

//    if(isslope == true && UseLidar){
//        nearby_box_filter_.setMin(Eigen::Vector4f(-lidarXAxis, MinCarDis, -SensorHigh - 0.03, 1.0));
//        nearby_box_filter_.setMax(Eigen::Vector4f(lidarXAxis, lidarYAxis, 0.5, 1.0));
//    }else if(isslope == false && UseLidar){
//        nearby_box_filter_.setMin(Eigen::Vector4f(-lidarXAxis, MinCarDis, -SensorHigh - 2, 1.0));
//        nearby_box_filter_.setMax(Eigen::Vector4f(lidarXAxis, lidarYAxis, 0.5, 1.0));
//    }

    // 计数
    static int loop = 1;
//    ComputeHoleTheory();


    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    // 将带点云和深度图放在一个坐标系下laserCloudTemp
    // laserCloudCrop->clear();
    if (is_print_message_){
        cout << "obj_ UseLidar:" << UseLidar << "UseDCamera" << UseDCamera << "AutoDirection" << AutoDirection << endl;
    }

    if (UseLidar && LidarCloud->points.size() > 10) {
        *laserCloudCrop += *LidarCloud;
    } else if (UseDCamera && CameraDeepCloud->size() > 10) {
        *laserCloudCrop += *CameraDeepCloud;
    } else return;



    CalculateIMURot();

    // *laserCloudCrop += *CameraDeepCloud;

    // 输出全部点云数量
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

//    if(isslope){
//        RemoveCloudWithSlopeLineAndPlaneCoff();
//    }

    // 将点云存到栅格中
    ProjectCloudToGrid();
    check_noise_postprocessv2();
//    UpdateSensorHigh();
//
    std::chrono::high_resolution_clock::time_point end2 = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
//
    // 计算点云特征,给每个格子赋予flag，是不是障碍物
    ComputeGridFeature2();
    check_noise_postprocess();



    std::chrono::high_resolution_clock::time_point end3 = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
    if (isslope == false){
        AddSurroundGrass();
    }
    std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();

//    if(isslope){
//        DrivableArea();
//    }
    DrivableAreav2();

    std::chrono::high_resolution_clock::time_point t6 = std::chrono::high_resolution_clock::now();
//    ComputerFlatness();

    // 聚类发布栅格
    ClusterAndPubObjectGrid();

    emptyholestartlabel = clusterlabel; // 聚类后的flag，会递增，一个类别用一个标签
//    EmptyHoleBFS();

    if(use_sort)SORT();

    std::chrono::high_resolution_clock::time_point t7 = std::chrono::high_resolution_clock::now();


//    if (HighGrass){
//        cloudsegmetation();
//    }

    std::chrono::high_resolution_clock::time_point t8 = std::chrono::high_resolution_clock::now();

    // EdgeDet();
    std::chrono::high_resolution_clock::time_point t9 = std::chrono::high_resolution_clock::now();

    std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();

#ifdef Debug
    PubFeatureVis();


#endif

    if (run_alone)
        PubBoundingBox();
    else
        PubBoundingBox_WORLD();
        

//    PubBoundingBox2();
//      SuperVoxel();
    std::chrono::high_resolution_clock::time_point t10 = std::chrono::high_resolution_clock::now();


    // std::cout << "Clustertime: " << fp_ms.count() << " avg_proj time:" << avg_proj / loop << " avg_feature time:" << avg_feature / loop << " avg_cluster time:" << avg_cluster / loop << " avg time:" << avg_ms / loop <<endl;
    loop++;

    PublishCloud();
    // SaveObj();
    std::chrono::high_resolution_clock::time_point t11 = std::chrono::high_resolution_clock::now();


//    ClearAll();
    std::chrono::high_resolution_clock::time_point t12 = std::chrono::high_resolution_clock::now();

//    if(is_print_message_){
//        std::chrono::duration<double, std::milli> d01 = t1 - t0;
//        std::chrono::duration<double, std::milli> d12 = t2 - t1;
//        std::chrono::duration<double, std::milli> d23 = t3 - t2;
//        std::chrono::duration<double, std::milli> d34 = t4 - t3;
//        std::chrono::duration<double, std::milli> d45 = t5 - t4;
//        std::chrono::duration<double, std::milli> d56 = t6 - t5;
//        std::chrono::duration<double, std::milli> d67 = t7 - t6;
//        std::chrono::duration<double, std::milli> d78 = t8 - t7;
//        std::chrono::duration<double, std::milli> d89 = t9 - t8;
//        std::chrono::duration<double, std::milli> d910 = t10 - t9;
//        std::chrono::duration<double, std::milli> d1011 = t11 - t10;
//        std::chrono::duration<double, std::milli> d1112 = t12 - t11;
//        std::chrono::duration<double, std::milli> dall = t12 - t0;
//        std::cout << "d01:" << d01.count() << " d12:" << d12.count()
//                  << " d23:" << d23.count() << " d34:" << d34.count()
//                  << " d45:" << d45.count() << " d56:" << d56.count()
//                  << " d67:" << d67.count() << " d78:" << d78.count()
//                  << " d89:" << d89.count() << " d910:" << d910.count()
//                  << " d1011:" << d1011.count() << " d1112:" << d1112.count()
//                  << " dall:" << dall.count() << endl;
//    }

//     avg_ms += dall.count();
//     loop++;
//     cout<<"avg_ms :"<<avg_ms/(double)loop<<endl;
}

