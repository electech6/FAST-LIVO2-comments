/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <vikit/camera_loader.h>

/**
 * @brief 主SLAM类 - 实现快速直接的LiDAR-惯性-视觉里程计
 * 
 * 这个类集成了LiDAR、IMU和相机数据，实现多传感器融合的SLAM系统
 */
class LIVMapper
{
public:
  // 构造函数和析构函数
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();
  
  // 初始化相关函数
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);  // 初始化ROS订阅器和发布器
  void initializeComponents();      // 初始化系统组件
  void initializeFiles();           // 初始化输出文件
  void run();                       // 主运行循环
  
  // 核心处理函数
  void gravityAlignment();          // 重力对齐，初始化IMU姿态
  void handleFirstFrame();          // 处理第一帧数据
  void stateEstimationAndMapping(); // 状态估计和建图主函数
  void handleVIO();                 // 视觉惯性里程计处理
  void handleLIO();                 // 激光惯性里程计处理
  void savePCD();                   // 保存点云数据
  void processImu();                // IMU数据处理
  
  // 数据同步和处理
  bool sync_packages(LidarMeasureGroup &meas);  // 数据包同步
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);  // 单次IMU传播
  void imu_prop_callback(const ros::TimerEvent &e);  // IMU传播定时器回调
  
  // 坐标变换函数
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);  // LiDAR点云变换
  void pointBodyToWorld(const PointType &pi, PointType &po);  // 点从机体坐标系到世界坐标系
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);  // RGB点坐标变换
  
  // 传感器数据回调函数
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);  // 标准点云回调
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);  // Livox点云回调
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);  // IMU数据回调
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);  // 图像数据回调
  
  // 数据发布函数
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);  // 发布RGB图像
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);  // 发布世界坐标系下的点云帧
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);  // 发布视觉子地图
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);  // 发布有效点云
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);  // 发布里程计
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);  // 发布MAVROS兼容的位姿
  void publish_path(const ros::Publisher pubPath);  // 发布路径
  
  // 工具函数
  void readParameters(ros::NodeHandle &nh);  // 读取参数
  template <typename T> void set_posestamp(T &out);  // 设置位姿戳模板函数
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);  // 模板坐标变换
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);  // 模板坐标变换返回值版本
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);  // 从ROS消息获取图像

  // 线程同步相关
  std::mutex mtx_buffer, mtx_buffer_imu_prop;  // 缓冲区互斥锁
  std::condition_variable sig_buffer;           // 缓冲区条件变量

  // SLAM模式和地图
  SLAM_MODE slam_mode_;  // SLAM工作模式
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;  // 体素地图数据结构
  
  // 配置参数
  string root_dir;        // 根目录
  string lid_topic, imu_topic, seq_name, img_topic;  // ROS话题名称
  V3D extT;              // 外部平移参数
  M3D extR;              // 外部旋转参数

  // 算法参数
  int feats_down_size = 0, max_iterations = 0;  // 特征点下采样大小和最大迭代次数
  double res_mean_last = 0.05;                  // 残差均值
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;  // 陀螺仪、加速度计、逆曝光协方差
  double blind_rgb_points = 0.0;                // 盲区RGB点阈值
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;  // 最后时间戳
  double filter_size_surf_min = 0;              // 最小表面滤波尺寸
  double filter_size_pcd = 0;                   // 点云滤波尺寸
  double _first_lidar_time = 0.0;               // 第一帧LiDAR时间
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;  // 各阶段耗时统计

  // 系统状态标志
  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int pcd_save_interval = -1, pcd_index = 0;    // PCD保存间隔和索引
  int pub_scan_num = 1;                         // 发布扫描数量

  // 状态估计相关
  StatesGroup imu_propagate, latest_ekf_state;  // IMU传播状态和最新EKF状态
  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;  // 状态标志
  deque<sensor_msgs::Imu> prop_imu_buffer;      // IMU传播缓冲区
  sensor_msgs::Imu newest_imu;                  // 最新IMU数据
  double latest_ekf_time;                       // 最新EKF时间
  nav_msgs::Odometry imu_prop_odom;             // IMU传播里程计
  ros::Publisher pubImuPropOdom;                // IMU传播里程计发布器
  double imu_time_offset = 0.0;                 // IMU时间偏移
  double lidar_time_offset = 0.0;               // LiDAR时间偏移

  // 重力对齐相关
  bool gravity_align_en = false, gravity_align_finished = false;  // 重力对齐使能和完成标志
  bool sync_jump_flag = false;                   // 同步跳跃标志

  // 传感器使能标志
  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;                     // 稠密地图使能
  int img_en = 1, imu_int_frame = 3;             // 图像使能和IMU积分帧数
  bool normal_en = true;                         // 法向量计算使能
  bool exposure_estimate_en = false;             // 曝光估计使能
  double exposure_time_init = 0.0;               // 初始曝光时间
  bool inverse_composition_en = false;           // 逆合成法使能
  bool raycast_en = false;                       // 光线投射使能
  int lidar_en = 1;                              // LiDAR使能
  bool is_first_frame = false;                   // 第一帧标志

  // 图像处理参数
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;  // 网格和块参数
  double outlier_threshold;                      // 异常值阈值
  double plot_time;                              // 绘图时间
  int frame_cnt;                                 // 帧计数器
  double img_time_offset = 0.0;                  // 图像时间偏移

  // 数据缓冲区
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;  // 原始LiDAR数据缓冲区
  deque<double> lid_header_time_buffer;          // LiDAR头时间缓冲区
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;  // IMU数据缓冲区
  deque<cv::Mat> img_buffer;                     // 图像缓冲区
  deque<double> img_time_buffer;                 // 图像时间缓冲区
  
  // 点云和协方差相关
  vector<pointWithVar> _pv_list;                 // 带方差点列表
  vector<double> extrinT;                        // 外部平移参数向量
  vector<double> extrinR;                        // 外部旋转参数向量
  vector<double> cameraextrinT;                  // 相机外部平移
  vector<double> cameraextrinR;                  // 相机外部旋转
  double IMG_POINT_COV;                          // 图像点协方差

  // 点云数据容器
  PointCloudXYZI::Ptr visual_sub_map;            // 视觉子地图
  PointCloudXYZI::Ptr feats_undistort;           // 未失真特征点
  PointCloudXYZI::Ptr feats_down_body;           // 机体坐标系下采样特征点
  PointCloudXYZI::Ptr feats_down_world;          // 世界坐标系下采样特征点
  PointCloudXYZI::Ptr pcl_w_wait_pub;            // 等待发布的带权重点云
  PointCloudXYZI::Ptr pcl_wait_pub;              // 等待发布的点云
  PointCloudXYZRGB::Ptr pcl_wait_save;           // 等待保存的RGB点云
  PointCloudXYZI::Ptr pcl_wait_save_intensity;   // 等待保存的强度点云

  // 文件输出
  ofstream fout_pre, fout_out, fout_pcd_pos, fout_points;  // 各种输出文件流

  // 点云处理
  pcl::VoxelGrid<PointType> downSizeFilterSurf;  // 表面下采样滤波器

  // 当前欧拉角
  V3D euler_cur;

  // 测量组和状态
  LidarMeasureGroup LidarMeasures;               // LiDAR测量组
  StatesGroup _state;                            // 当前状态
  StatesGroup  state_propagat;                   // 传播状态

  // ROS消息容器
  nav_msgs::Path path;                           // 路径消息
  nav_msgs::Odometry odomAftMapped;              // 建图后里程计
  geometry_msgs::Quaternion geoQuat;             // 四元数
  geometry_msgs::PoseStamped msg_body_pose;      // 机体位姿

  // 管理器指针
  PreprocessPtr p_pre;                           // 预处理器
  ImuProcessPtr p_imu;                           // IMU处理器
  VoxelMapManagerPtr voxelmap_manager;           // 体素地图管理器
  VIOManagerPtr vio_manager;                     // VIO管理器

  // ROS发布器和订阅器
  ros::Publisher plane_pub;                      // 平面发布器
  ros::Publisher voxel_pub;                      // 体素发布器
  ros::Subscriber sub_pcl;                       // 点云订阅器
  ros::Subscriber sub_imu;                       // IMU订阅器
  ros::Subscriber sub_img;                       // 图像订阅器
  ros::Publisher pubLaserCloudFullRes;           // 全分辨率点云发布器
  ros::Publisher pubNormal;                      // 法向量发布器
  ros::Publisher pubSubVisualMap;                // 视觉子地图发布器
  ros::Publisher pubLaserCloudEffect;            // 有效点云发布器
  ros::Publisher pubLaserCloudMap;               // 点云地图发布器
  ros::Publisher pubOdomAftMapped;               // 建图后里程计发布器
  ros::Publisher pubPath;                        // 路径发布器
  ros::Publisher pubLaserCloudDyn;               // 动态点云发布器
  ros::Publisher pubLaserCloudDynRmed;           // 移除动态点云发布器
  ros::Publisher pubLaserCloudDynDbg;            // 动态点云调试发布器
  image_transport::Publisher pubImage;           // 图像发布器
  ros::Publisher mavros_pose_publisher;          // MAVROS位姿发布器
  ros::Timer imu_prop_timer;                     // IMU传播定时器

  // 性能统计
  int frame_num = 0;                             // 帧数统计
  double aver_time_consu = 0;                    // 平均耗时
  double aver_time_icp = 0;                      // 平均ICP耗时
  double aver_time_map_inre = 0;                 // 平均地图增量耗时
  bool colmap_output_en = false;                 // COLMAP输出使能
};

#endif