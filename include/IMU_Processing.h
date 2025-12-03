/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include "common_lib.h"
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <utils/so3_math.h>
#include <fstream>

// 点云时间排序函数：按曲率（存储时间戳）升序排列
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

/// *************IMU处理与点云去畸变类
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐宏

  // 构造函数与析构函数
  ImuProcess();
  ~ImuProcess();

  // 重置函数
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  
  // 外参设置函数（多种重载形式）
  void set_extrinsic(const V3D &transl, const M3D &rot);  // 设置平移和旋转外参
  void set_extrinsic(const V3D &transl);                  // 仅设置平移外参
  void set_extrinsic(const MD(4, 4) & T);                 // 通过4x4变换矩阵设置外参
  
  // 噪声参数设置函数
  void set_gyr_cov_scale(const V3D &scaler);      // 设置陀螺仪噪声协方差缩放因子
  void set_acc_cov_scale(const V3D &scaler);       // 设置加速度计噪声协方差缩放因子
  void set_gyr_bias_cov(const V3D &b_g);           // 设置陀螺仪偏置协方差
  void set_acc_bias_cov(const V3D &b_a);           // 设置加速度计偏置协方差
  void set_inv_expo_cov(const double &inv_expo);   // 设置逆曝光时间协方差
  void set_imu_init_frame_num(const int &num);     // 设置IMU初始化所需帧数
  
  // 功能禁用函数
  void disable_imu();              // 禁用IMU功能
  void disable_gravity_est();      // 禁用重力估计
  void disable_bias_est();         // 禁用偏置估计
  void disable_exposure_est();     // 禁用曝光时间估计
  
  // 主要处理函数
  void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
  void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

  // 公共成员变量
  ofstream fout_imu;               // IMU数据输出文件流
  double IMU_mean_acc_norm;        // IMU加速度均值模长
  V3D unbiased_gyr;                // 无偏角速度

  // 噪声协方差参数
  V3D cov_acc;                     // 加速度计噪声协方差
  V3D cov_gyr;                     // 陀螺仪噪声协方差
  V3D cov_bias_gyr;                // 陀螺仪偏置噪声协方差
  V3D cov_bias_acc;                // 加速度计偏置噪声协方差
  double cov_inv_expo;             // 逆曝光时间协方差
  
  // 时间相关变量
  double first_lidar_time;         // 第一帧LiDAR时间
  bool imu_time_init = false;      // IMU时间初始化标志
  bool imu_need_init = true;       // IMU需要初始化标志
  
  // 常用数学常量
  M3D Eye3d;                       // 3x3单位矩阵
  V3D Zero3d;                      // 3维零向量
  
  int lidar_type;                  // LiDAR类型标识

private:
  // 私有成员函数
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);  // IMU初始化
  void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);  // 无IMU前向传播

  // 私有成员变量
  PointCloudXYZI pcl_wait_proc;                    // 等待处理的点云
  sensor_msgs::ImuConstPtr last_imu;               // 上一帧IMU数据
  PointCloudXYZI::Ptr cur_pcl_un_;                 // 当前去畸变点云指针
  vector<Pose6D> IMUpose;                          // IMU位姿序列（6自由度位姿）
  
  // 外参参数
  M3D Lid_rot_to_IMU;                              // LiDAR到IMU的旋转矩阵
  V3D Lid_offset_to_IMU;                           // LiDAR到IMU的平移向量
  
  // 统计参数
  V3D mean_acc;                                    // 加速度均值
  V3D mean_gyr;                                    // 角速度均值
  V3D angvel_last;                                 // 上一时刻角速度
  V3D acc_s_last;                                  // 上一时刻加速度
  
  // 时间相关变量
  double last_prop_end_time;                       // 上一次传播结束时间
  double time_last_scan;                           // 上一次扫描时间
  
  // 初始化参数
  int init_iter_num = 1;                           // 初始化迭代次数
  int MAX_INI_COUNT = 20;                          // 最大初始化计数
  
  // 状态标志
  bool b_first_frame = true;                       // 是否为第一帧标志
  bool imu_en = true;                              // IMU使能标志
  bool gravity_est_en = true;                      // 重力估计使能标志
  bool ba_bg_est_en = true;                        // 偏置估计使能标志
  bool exposure_estimate_en = true;                // 曝光时间估计使能标志
};

// IMU处理类智能指针类型定义
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;

#endif