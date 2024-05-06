#ifndef FASTER_LIO_IMU_PROCESSING_H
#define FASTER_LIO_IMU_PROCESSING_H

#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <deque>
#include <fstream>

#include "common_lib.h"
#include "so3_math.h"
#include "use-ikfom.hpp"
#include "utils.h"

namespace faster_lio {

constexpr int MAX_INI_COUNT = 20;

bool time_list(const PointType &x, const PointType &y) { return (x.curvature < y.curvature); };

struct StateCovInit
{
    std::vector<double> position = std::vector<double>(3);
    std::vector<double> rotation = std::vector<double>(3);
    std::vector<double> extrinsic_R_L_I = std::vector<double>(3);
    std::vector<double> extrinsic_T_L_I = std::vector<double>(3);
    std::vector<double> velocity = std::vector<double>(3);
    std::vector<double> bg = std::vector<double>(3);
    std::vector<double> ba = std::vector<double>(3);
    std::vector<double> gravity = std::vector<double>(2);
};

/// IMU Process and undistortion
class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void SetExtrinsic(const common::V3D &transl, const common::M3D &rot);
    void SetGyrCov(const common::V3D &scaler);
    void SetAccCov(const common::V3D &scaler);
    void SetGyrBiasCov(const common::V3D &b_g);
    void SetAccBiasCov(const common::V3D &b_a);
    void SetInitStateCov(const StateCovInit &state_cov);
    void GetAngVel(common::V3D &ang_vel);
    void Process(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                 PointCloudType::Ptr pcl_un_);

    // std::ofstream fout_imu_;
    Eigen::Matrix<double, 12, 12> Q_;
    common::V3D cov_acc_;
    common::V3D cov_gyr_;
    common::V3D cov_acc_scale_;
    common::V3D cov_gyr_scale_;
    common::V3D cov_bias_gyr_;
    common::V3D cov_bias_acc_;

   private:
    void IMUInit(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
    void UndistortPcl(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                      PointCloudType &pcl_out);

    PointCloudType::Ptr cur_pcl_un_;
    sensor_msgs::ImuConstPtr last_imu_;
    std::deque<sensor_msgs::ImuConstPtr> v_imu_;
    std::vector<common::Pose6D> IMUpose_;
    std::vector<common::M3D> v_rot_pcl_;
    common::M3D Lidar_R_wrt_IMU_;
    common::V3D Lidar_T_wrt_IMU_;
    common::V3D mean_acc_;
    common::V3D mean_gyr_;
    common::V3D angvel_last_;
    common::V3D acc_s_last_;
    double last_lidar_end_time_ = 0;
    int init_iter_num_ = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
    StateCovInit init_state_cov_;
};

ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_ = process_noise_cov();
    cov_acc_ = common::V3D(0.1, 0.1, 0.1);
    cov_gyr_ = common::V3D(0.1, 0.1, 0.1);
    cov_bias_gyr_ = common::V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = common::V3D(0.0001, 0.0001, 0.0001);
    mean_acc_ = common::V3D(0, 0, -1.0);
    mean_gyr_ = common::V3D(0, 0, 0);
    angvel_last_ = common::Zero3d;
    Lidar_T_wrt_IMU_ = common::Zero3d;
    Lidar_R_wrt_IMU_ = common::Eye3d;
    last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    mean_acc_ = common::V3D(0, 0, -1.0);
    mean_gyr_ = common::V3D(0, 0, 0);
    angvel_last_ = common::Zero3d;
    imu_need_init_ = true;
    init_iter_num_ = 1;
    v_imu_.clear();
    IMUpose_.clear();
    last_imu_.reset(new sensor_msgs::Imu());
    cur_pcl_un_.reset(new PointCloudType());
}

void ImuProcess::SetExtrinsic(const common::V3D &transl, const common::M3D &rot) {
    Lidar_T_wrt_IMU_ = transl;
    Lidar_R_wrt_IMU_ = rot;
}

void ImuProcess::SetGyrCov(const common::V3D &scaler) { cov_gyr_scale_ = scaler; }

void ImuProcess::SetAccCov(const common::V3D &scaler) { cov_acc_scale_ = scaler; }

void ImuProcess::SetGyrBiasCov(const common::V3D &b_g) { cov_bias_gyr_ = b_g; }

void ImuProcess::SetAccBiasCov(const common::V3D &b_a) { cov_bias_acc_ = b_a; }

void ImuProcess::SetInitStateCov(const StateCovInit &state_cov) { init_state_cov_ = state_cov; }

void ImuProcess::GetAngVel(common::V3D &ang_vel) { ang_vel = angvel_last_; }

void ImuProcess::IMUInit(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         int &N) {
    /** 1. initializing the gravity_, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity_ **/

    common::V3D cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu_.front()->linear_acceleration;
        const auto &imu_gyr = meas.imu_.front()->angular_velocity;
        mean_acc_ << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr_ << imu_gyr.x, imu_gyr.y, imu_gyr.z;
    }

    for (const auto &imu : meas.imu_) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &imu_gyr = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << imu_gyr.x, imu_gyr.y, imu_gyr.z;

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        cov_acc_ =
            cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ =
            cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        N++;
    }
    state_ikfom init_state = kf_state.get_x();
    init_state.grav = S2(-mean_acc_ / mean_acc_.norm() * common::G_m_s2);
    init_state.bg = mean_gyr_;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU_;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU_;

    // (roll, pitch, raw) are Euler angles expressed in the order ZYX
    // roll = atan(ay/az)
    // pitch = -atan(ax,sqrt(ay^2+az^2))
    // due to z-axis up is positive, change the sign for mapping
    double roll  = atan2(-init_state.grav.vec(1), 
                         -init_state.grav.vec(2));

    double pitch = -atan2(-init_state.grav.vec(0), 
                          sqrt(init_state.grav.vec(1)*init_state.grav.vec(1)+
                               init_state.grav.vec(2)*init_state.grav.vec(2)));
    Eigen::Quaterniond q_grav = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());

    common::V3D euler_angle_pre = 180/M_PI*init_state.rot.matrix().eulerAngles(2, 1, 0);
    ROS_ERROR_STREAM("===========transform the initial frame to gavity aligned frame===========");
    ROS_ERROR_STREAM("previous grav is: " << init_state.grav.vec(0) <<  " " << init_state.grav.vec(1) << " " << init_state.grav.vec(2));
    ROS_ERROR_STREAM("previous rota is: " << euler_angle_pre[0] << " " << euler_angle_pre[1] << " " << euler_angle_pre[2]);
    init_state.rot  = q_grav;
    init_state.grav = q_grav.toRotationMatrix() * S2(-mean_acc_ / mean_acc_.norm() * common::G_m_s2);
    common::V3D euler_angle_cur = 180/M_PI*init_state.rot.matrix().eulerAngles(2, 1, 0);
    ROS_ERROR_STREAM("current grav is: " << init_state.grav.vec(0) <<  " " << init_state.grav.vec(1) << " " << init_state.grav.vec(2));
    ROS_ERROR_STREAM("current rota is: " << euler_angle_cur[0] << " " << euler_angle_cur[1] << " " << euler_angle_cur[2]);
    ROS_ERROR_STREAM("=========================================================================");
    kf_state.change_x(init_state);
    ROS_WARN_STREAM("initial pos is: "  << init_state.pos);
    ROS_WARN_STREAM("initial rot is: "  << init_state.rot);
    ROS_WARN_STREAM("initial vel is: "  << init_state.vel);
    ROS_WARN_STREAM("initial gra is: "  << init_state.grav);
    ROS_WARN_STREAM("initial ba is: "   << init_state.ba);
    ROS_WARN_STREAM("initial bg is: "   << init_state.bg);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P(0, 0) = init_state_cov_.position[0];
    init_P(1, 1) = init_state_cov_.position[1];
    init_P(2, 2) = init_state_cov_.position[2];

    init_P(3, 3) = init_state_cov_.rotation[0];
    init_P(4, 4) = init_state_cov_.rotation[1];
    init_P(5, 5) = init_state_cov_.rotation[2]; 

    init_P(6, 6) = init_state_cov_.extrinsic_R_L_I[0]; 
    init_P(7, 7) = init_state_cov_.extrinsic_R_L_I[1];
    init_P(8, 8) = init_state_cov_.extrinsic_R_L_I[2];

    init_P(9, 9)   = init_state_cov_.extrinsic_T_L_I[0]; 
    init_P(10, 10) = init_state_cov_.extrinsic_T_L_I[1];
    init_P(11, 11) = init_state_cov_.extrinsic_T_L_I[2];

    init_P(12, 12) = init_state_cov_.velocity[0]; 
    init_P(13, 13) = init_state_cov_.velocity[1];
    init_P(14, 14) = init_state_cov_.velocity[2];

    init_P(15, 15) = init_state_cov_.bg[0]; 
    init_P(16, 16) = init_state_cov_.bg[1];
    init_P(17, 17) = init_state_cov_.bg[2];

    init_P(18, 18) = init_state_cov_.ba[0]; 
    init_P(19, 19) = init_state_cov_.ba[1];
    init_P(20, 20) = init_state_cov_.ba[2];

    init_P(21,21) = init_state_cov_.gravity[0];
    init_P(22,22) = init_state_cov_.gravity[1];

    kf_state.change_P(init_P);
    last_imu_ = meas.imu_.back();
}

void ImuProcess::UndistortPcl(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                              PointCloudType &pcl_out) {
    /*** add the imu_ of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu_;
    v_imu.push_front(last_imu_);
    const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
    const double &imu_end_time = v_imu.back()->header.stamp.toSec();
    const double &pcl_beg_time = meas.lidar_bag_time_;
    const double &pcl_end_time = meas.lidar_end_time_;

    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar_);
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    IMUpose_.clear();
    IMUpose_.push_back(common::set_pose6d(0.0, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                          imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu_ point ***/
    common::V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    common::M3D R_imu;

    double dt = 0;

    input_ikfom in;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (tail->header.stamp.toSec() < last_lidar_end_time_) {
            continue;
        }

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                      0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                      0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                   0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                   0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        acc_avr = acc_avr * common::G_m_s2 / mean_acc_.norm();  // - state_inout.ba;

        if (head->header.stamp.toSec() < last_lidar_end_time_) {
            dt = tail->header.stamp.toSec() - last_lidar_end_time_;
        } else {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        }

        in.acc = acc_avr;
        in.gyro = angvel_avr;
        Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
        Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
        Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
        kf_state.predict(dt, Q_, in);

        /* save the poses at each IMU measurements */
        imu_state = kf_state.get_x();
        angvel_last_ = angvel_avr - imu_state.bg;
        acc_s_last_ = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++) {
            acc_s_last_[i] += imu_state.grav[i];
        }

        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        IMUpose_.emplace_back(common::set_pose6d(offs_t, acc_s_last_, angvel_last_, imu_state.vel, imu_state.pos,
                                                 imu_state.rot.toRotationMatrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.predict(dt, Q_, in);

    imu_state = kf_state.get_x();
    last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out.points.empty()) {
        return;
    }
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose_.end() - 1; it_kp != IMUpose_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = common::MatFromArray(head->rot);
        vel_imu = common::VecFromArray(head->vel);
        pos_imu = common::VecFromArray(head->pos);
        acc_imu = common::VecFromArray(tail->acc);
        angvel_avr = common::VecFromArray(tail->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
            dt = it_pcl->curvature / double(1000) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * p_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            common::M3D R_i(R_imu * Exp(angvel_avr, dt));

            common::V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            common::V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
            common::V3D p_compensate =
                imu_state.offset_R_L_I.conjugate() *
                (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
                 imu_state.offset_T_L_I);  // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            if (it_pcl == pcl_out.points.begin()) {
                break;
            }
        }
    }
}

void ImuProcess::Process(const common::MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         PointCloudType::Ptr cur_pcl_un_) {
    if (meas.imu_.empty()) {
        return;
    }

    ROS_ASSERT(meas.lidar_ != nullptr);

    if (imu_need_init_) {
        /// The very first lidar frame
        IMUInit(meas, kf_state, init_iter_num_);

        imu_need_init_ = true;

        last_imu_ = meas.imu_.back();

        state_ikfom imu_state = kf_state.get_x();
        if (init_iter_num_ > MAX_INI_COUNT) {
            cov_acc_ *= pow(common::G_m_s2 / mean_acc_.norm(), 2);
            imu_need_init_ = false;

            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            LOG(INFO) << "IMU Initial Done";
            // fout_imu_.open(common::DEBUG_FILE_DIR("imu_.txt"), std::ios::out);
        }

        return;
    }

    Timer::Evaluate([&, this]() { UndistortPcl(meas, kf_state, *cur_pcl_un_); }, "Undistort Pcl");
}
}  // namespace faster_lio

#endif
