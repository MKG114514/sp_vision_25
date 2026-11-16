#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  // 2026前哨站高度自学习：第一圈用PnP观测值记录各板实际高度，
  // 学习完成后用正确偏移替代同高假设，消除3板高度差对EKF的影响
  bool outpost_heights_learned_ = false;
  int outpost_relearn_count_ = 0;
  double outpost_learned_offsets_[3] = {0.0, 0.0, 0.0};
  double outpost_z_accum_[3] = {0.0, 0.0, 0.0};
  int outpost_z_count_[3] = {0, 0, 0};
  static constexpr int OUTPOST_MIN_Z_OBS = 3;
  static constexpr int OUTPOST_RELEARN_INTERVAL = 400;  //400帧
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP