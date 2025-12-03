/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "IMU_Processing.h"

// 构造函数：初始化IMU处理类的各种参数和状态
ImuProcess::ImuProcess() : Eye3d(M3D::Identity()),
                           Zero3d(0, 0, 0), b_first_frame(true), imu_need_init(true)
{
  init_iter_num = 1;
  cov_acc = V3D(0.1, 0.1, 0.1);            // 加速度计噪声协方差
  cov_gyr = V3D(0.1, 0.1, 0.1);            // 陀螺仪噪声协方差
  cov_bias_gyr = V3D(0.1, 0.1, 0.1);       // 陀螺仪偏置噪声协方差
  cov_bias_acc = V3D(0.1, 0.1, 0.1);       // 加速度计偏置噪声协方差
  cov_inv_expo = 0.2;                      // 逆曝光时间协方差
  mean_acc = V3D(0, 0, -1.0);              // 初始加速度均值（假设重力向下）
  mean_gyr = V3D(0, 0, 0);                 // 初始角速度均值
  angvel_last = Zero3d;                    // 上一时刻角速度
  acc_s_last = Zero3d;                     // 上一时刻加速度
  Lid_offset_to_IMU = Zero3d;              // LiDAR到IMU的平移外参
  Lid_rot_to_IMU = Eye3d;                  // LiDAR到IMU的旋转外参
  last_imu.reset(new sensor_msgs::Imu());  // 上一帧IMU数据
  cur_pcl_un_.reset(new PointCloudXYZI()); // 当前去畸变点云
}

// 析构函数
ImuProcess::~ImuProcess() {}

// 重置IMU处理状态
void ImuProcess::Reset()
{
  ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init = true;        // 需要重新初始化
  init_iter_num = 1;
  IMUpose.clear();             // 清空IMU位姿队列
  last_imu.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

// 禁用IMU功能
void ImuProcess::disable_imu()
{
  cout << "IMU Disabled !!!!!" << endl;
  imu_en = false;
  imu_need_init = false;
}

// 禁用重力估计
void ImuProcess::disable_gravity_est()
{
  cout << "Online Gravity Estimation Disabled !!!!!" << endl;
  gravity_est_en = false;
}

// 禁用偏置估计
void ImuProcess::disable_bias_est()
{
  cout << "Bias Estimation Disabled !!!!!" << endl;
  ba_bg_est_en = false;
}

// 禁用曝光时间估计
void ImuProcess::disable_exposure_est()
{
  cout << "Online Time Offset Estimation Disabled !!!!!" << endl;
  exposure_estimate_en = false;
}

// 设置外参（4x4变换矩阵）
void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
  Lid_offset_to_IMU = T.block<3, 1>(0, 3);  // 提取平移部分
  Lid_rot_to_IMU = T.block<3, 3>(0, 0);     // 提取旋转部分
}

// 设置外参（仅平移）
void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU.setIdentity();  // 旋转设为单位矩阵
}

// 设置外参（平移和旋转）
void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU = rot;
}

// 设置陀螺仪噪声协方差缩放因子
void ImuProcess::set_gyr_cov_scale(const V3D &scaler) { cov_gyr = scaler; }

// 设置加速度计噪声协方差缩放因子
void ImuProcess::set_acc_cov_scale(const V3D &scaler) { cov_acc = scaler; }

// 设置陀螺仪偏置协方差
void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

// 设置逆曝光时间协方差
void ImuProcess::set_inv_expo_cov(const double &inv_expo) { cov_inv_expo = inv_expo; }

// 设置加速度计偏置协方差
void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

// 设置IMU初始化所需帧数
void ImuProcess::set_imu_init_frame_num(const int &num) { MAX_INI_COUNT = num; }

// IMU初始化：计算重力方向、陀螺仪偏置等初始参数
void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{
  /** 1. 初始化重力、陀螺仪偏置、加速度和角速度协方差
   ** 2. 将加速度测量归一化为单位重力 **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;

  if (b_first_frame)  // 第一帧初始化
  {
    Reset();
    N = 1;
    b_first_frame = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  // 遍历所有IMU测量值，计算均值和协方差
  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    // 递推计算均值
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    N++;
  }
  
  // 计算重力方向和初始状态
  IMU_mean_acc_norm = mean_acc.norm();
  state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;  // 重力向量
  state_inout.rot_end = Eye3d;     // 初始旋转为单位矩阵
  state_inout.bias_g = Zero3d;     // 初始陀螺仪偏置为零

  last_imu = meas.imu.back();  // 保存最后一帧IMU数据
}

// 不使用IMU的前向传播（仅用于无IMU模式）
void ImuProcess::Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  pcl_out = *(meas.lidar);
  /*** 按时间偏移对点云排序 ***/
  const double &pcl_beg_time = meas.lidar_frame_beg_time;
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  meas.last_lio_update_time = pcl_end_time;
  const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);

  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt = 0;

  // 计算时间间隔
  if (b_first_frame)
  {
    dt = 0.1;
    b_first_frame = false;
  }
  else { dt = pcl_beg_time - time_last_scan; }

  time_last_scan = pcl_beg_time;

  /* 协方差传播 */
  M3D Exp_f = Exp(state_inout.bias_g, dt);  // 指数映射计算旋转变化

  F_x.setIdentity();
  cov_w.setZero();

  // 状态转移矩阵
  F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
  F_x.block<3, 3>(0, 10) = Eye3d * dt;
  F_x.block<3, 3>(3, 7) = Eye3d * dt;

  // 过程噪声协方差
  cov_w.block<3, 3>(10, 10).diagonal() = cov_gyr * dt * dt;
  cov_w.block<3, 3>(7, 7).diagonal() = cov_acc * dt * dt;

  // 协方差传播
  state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

  // 状态传播
  state_inout.rot_end = state_inout.rot_end * Exp_f;
  state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;

  // 点云去畸变（L515以外的LiDAR）
  if (lidar_type != L515)
  {
    auto it_pcl = pcl_out.points.end() - 1;
    double dt_j = 0.0;
    for(; it_pcl != pcl_out.points.begin(); it_pcl--)
    {
        dt_j= pcl_end_offset_time - it_pcl->curvature/double(1000);
        M3D R_jk(Exp(state_inout.bias_g, - dt_j));
        V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
        
        // 使用旋转和平移去畸变
        V3D p_jk = - state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;
        V3D P_compensate =  R_jk * P_j + p_jk;
  
        // 保存去畸变后的点
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);
    }
  }
}

// 点云去畸变处理（主要函数）
void ImuProcess::UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  double t0 = omp_get_wtime();
  pcl_out.clear();
  
  /*** 将上一帧尾部的IMU数据添加到当前帧头部 ***/
  MeasureGroup &meas = lidar_meas.measures.back();
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu);
  
  // 获取时间信息
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double prop_beg_time = last_prop_end_time;
  const double prop_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;

  // 准备待处理的点云数据
  if (lidar_meas.lio_vio_flg == LIO)
  {
    pcl_wait_proc.resize(lidar_meas.pcl_proc_cur->points.size());
    pcl_wait_proc = *(lidar_meas.pcl_proc_cur);
    lidar_meas.lidar_scan_index_now = 0;
    // 保存初始IMU位姿
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));
  }

  /*** 初始化IMU位姿 ***/
  V3D acc_imu(acc_s_last), angvel_avr(angvel_last), acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  M3D R_imu(state_inout.rot_end);
  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt, dt_all = 0.0;
  double offs_t;
  double tau;
  
  // 初始化逆曝光时间
  if (!imu_time_init)
  {
    tau = 1.0;
    imu_time_init = true;
  }
  else
  {
    tau = state_inout.inv_expo_time;
  }

  // 根据LIO/VIO模式进行前向传播
  switch (lidar_meas.lio_vio_flg)
  {
  case LIO:
  case VIO:
    dt = 0;
    // 遍历IMU数据并进行前向传播
    for (int i = 0; i < v_imu.size() - 1; i++)
    {
      auto head = v_imu[i];
      auto tail = v_imu[i + 1];

      if (tail->header.stamp.toSec() < prop_beg_time) continue;

      // 计算平均角速度和加速度
      angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x), 
                    0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                    0.5 * (head->angular_velocity.z + tail->angular_velocity.z);

      acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x), 
                 0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                 0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

      // 记录IMU数据到文件
      fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

      // 去除偏置的影响
      angvel_avr -= state_inout.bias_g;
      acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

      // 计算时间间隔
      if (head->header.stamp.toSec() < prop_beg_time)
      {
        dt = tail->header.stamp.toSec() - last_prop_end_time;
        offs_t = tail->header.stamp.toSec() - prop_beg_time;
      }
      else if (i != v_imu.size() - 2)
      {
        dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        offs_t = tail->header.stamp.toSec() - prop_beg_time;
      }
      else
      {
        dt = prop_end_time - head->header.stamp.toSec();
        offs_t = prop_end_time - prop_beg_time;
      }

      dt_all += dt;

      /* 协方差传播 */
      M3D acc_avr_skew;
      M3D Exp_f = Exp(angvel_avr, dt);
      acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

      F_x.setIdentity();
      cov_w.setZero();

      // 构建状态转移矩阵
      F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
      if (ba_bg_est_en) F_x.block<3, 3>(0, 10) = -Eye3d * dt;
      F_x.block<3, 3>(3, 7) = Eye3d * dt;
      F_x.block<3, 3>(7, 0) = -R_imu * acc_avr_skew * dt;
      if (ba_bg_est_en) F_x.block<3, 3>(7, 13) = -R_imu * dt;
      if (gravity_est_en) F_x.block<3, 3>(7, 16) = Eye3d * dt;

      // 过程噪声协方差
      if (exposure_estimate_en) cov_w(6, 6) = cov_inv_expo * dt * dt;
      cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
      cov_w.block<3, 3>(7, 7) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
      cov_w.block<3, 3>(10, 10).diagonal() = cov_bias_gyr * dt * dt;
      cov_w.block<3, 3>(13, 13).diagonal() = cov_bias_acc * dt * dt;

      // 协方差传播
      state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

      /* IMU姿态传播 */
      R_imu = R_imu * Exp_f;

      /* 计算IMU在全局坐标系下的加速度 */
      acc_imu = R_imu * acc_avr + state_inout.gravity;

      /* IMU位置和速度传播 */
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
      vel_imu = vel_imu + acc_imu * dt;

      /* 保存每个IMU测量时刻的位姿 */
      angvel_last = angvel_avr;
      acc_s_last = acc_imu;

      IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }

    lidar_meas.last_lio_update_time = prop_end_time;
    break;
  }

  // 更新最终状态
  state_inout.vel_end = vel_imu;
  state_inout.rot_end = R_imu;
  state_inout.pos_end = pos_imu;
  state_inout.inv_expo_time = tau;

  last_imu = v_imu.back();
  last_prop_end_time = prop_end_time;

  double t1 = omp_get_wtime();

  if (pcl_wait_proc.points.size() < 1) return;

  /*** 对每个LiDAR点进行去畸变（反向传播），仅用于LIO更新 ***/
  if (lidar_meas.lio_vio_flg == LIO)
  {
    auto it_pcl = pcl_wait_proc.points.end() - 1;
    // 计算外参变换
    M3D extR_Ri(Lid_rot_to_IMU.transpose() * state_inout.rot_end.transpose());
    V3D exrR_extT(Lid_rot_to_IMU.transpose() * Lid_offset_to_IMU);
    
    // 反向遍历IMU位姿
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << MAT_FROM_ARRAY(head->rot);
      acc_imu << VEC_FROM_ARRAY(head->acc);
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      angvel_avr << VEC_FROM_ARRAY(head->gyr);

      // 处理在当前IMU时间段内的点云
      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
      {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        /* 变换到'end'坐标系 */
        M3D R_i(R_imu * Exp(angvel_avr, dt));
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - state_inout.pos_end);

        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        // 计算补偿后的点坐标
        V3D P_compensate = (extR_Ri * (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei) - exrR_extT);

        // 保存去畸变后的点
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_wait_proc.points.begin()) break;
      }
    }
    pcl_out = pcl_wait_proc;
    pcl_wait_proc.clear();
    IMUpose.clear();
  }
}

// 主要的IMU处理函数
void ImuProcess::Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1, t2, t3;
  t1 = omp_get_wtime();
  ROS_ASSERT(lidar_meas.lidar != nullptr);
  
  // 如果IMU被禁用，使用无IMU的前向传播
  if (!imu_en)
  {
    Forward_without_imu(lidar_meas, stat, *cur_pcl_un_);
    return;
  }

  MeasureGroup meas = lidar_meas.measures.back();

  // IMU初始化阶段
  if (imu_need_init)
  {
    double pcl_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;

    if (meas.imu.empty()) { return; };
    
    /// 第一个LiDAR帧，进行IMU初始化
    IMU_init(meas, stat, init_iter_num);

    imu_need_init = true;
    last_imu = meas.imu.back();

    // 当初始化迭代次数达到要求时，完成初始化
    if (init_iter_num > MAX_INI_COUNT)
    {
      imu_need_init = false;
      ROS_INFO("IMU Initials: Gravity: %.4f %.4f %.4f %.4f; acc covarience: "
               "%.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f \n",
               stat.gravity[0], stat.gravity[1], stat.gravity[2], mean_acc.norm(), cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1],
               cov_gyr[2]);
      ROS_INFO("IMU Initials: ba covarience: %.8f %.8f %.8f; bg covarience: "
               "%.8f %.8f %.8f",
               cov_bias_acc[0], cov_bias_acc[1], cov_bias_acc[2], cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);  // 打开IMU数据记录文件
    }

    return;
  }

  // 执行点云去畸变
  UndistortPcl(lidar_meas, stat, *cur_pcl_un_);
}