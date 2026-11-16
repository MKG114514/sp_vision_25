#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(921600);

    // 串口配置（换串口有时候要用）
    // serial_.setFlowcontrol(serial::flowcontrol_none);
    // serial_.setParity(serial::parity_none);
    // serial_.setStopbits(serial::stopbits_one);
    // serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  {
    std::unique_lock<std::mutex> lock(q_mutex_);
    q_cv_.wait(lock, [this] { return !q_buffer_.empty() || quit_; });
  }
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  q_cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  std::unique_lock<std::mutex> lock(q_mutex_);
  while (!quit_) {
    q_cv_.wait(lock, [this] { return !q_buffer_.empty() || quit_; });
    if (quit_) break;

    const auto & [q_a, t_a] = q_buffer_.front();
    if (t <= t_a) {
      tools::logger()->warn(
        "[Gimbal::q] t <= t_a (diff: {:.3f} ms, buf: {})", tools::delta_time(t_a, t) * 1e3,
        q_buffer_.size());
      return q_a;
    }

    if (q_buffer_.size() < 2) {
      q_cv_.wait(lock, [this] { return q_buffer_.size() >= 2 || quit_; });
      continue;
    }

    const auto & [q_b, t_b] = q_buffer_[1];
    if (t > t_b) {
      q_buffer_.pop_front();
      continue;
    }

    auto t_ab = tools::delta_time(t_b, t_a);
    if (t_ab <= 1e-6) {
      tools::logger()->warn("[Gimbal::q] t_ab too small: {:.6f} s", t_ab);
      return q_b;
    }

    auto t_ac = tools::delta_time(t, t_a);
    auto k = std::clamp(t_ac / t_ab, 0.0, 1.0);
    //插值是否正常
    // tools::logger()->debug(
    //   "[Gimbal::q] slerp k: {:.3f} (t_ac: {:.3f} ms, t_ab: {:.3f} ms, buf: {})", k, t_ac * 1e3,
    //   t_ab * 1e3, q_buffer_.size());
    return q_a.slerp(k, q_b).normalized();
  }
  return Eigen::Quaterniond::Identity();
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = - pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 150) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    uint8_t head[2] = {0, 0};
    if (!read(&head[0], 1)) {
      error_count++;
      continue;
    }
    if (head[0] != 'S') {
      error_count++;
      continue;
    }

    while (!quit_) {
      if (!read(&head[1], 1)) {
        error_count++;
        break;
      }
      if (head[1] == 'P') break;
      if (head[1] != 'S') {
        error_count++;
        break;
      }
    }
    if (head[1] != 'P') continue;

    rx_data_.head[0] = 'S';
    rx_data_.head[1] = 'P';
    const size_t body_size = sizeof(rx_data_) - sizeof(rx_data_.head);
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head), body_size)) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      error_count++;
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    // 复制到局部变量以避免 packed 字段引用问题
    uint8_t mode = rx_data_.mode;
    float yaw = rx_data_.yaw;
    float pitch = rx_data_.pitch;
    float yaw_vel = rx_data_.yaw_vel;
    float pitch_vel = rx_data_.pitch_vel;
    float bullet_speed = rx_data_.bullet_speed;
    uint16_t bullet_count = rx_data_.bullet_count;
    float q0 = rx_data_.q[0], q1 = rx_data_.q[1], q2 = rx_data_.q[2], q3 = rx_data_.q[3];

    // tools::logger()->debug(
    //     "[Gimbal] mode:{}, yaw:{:.2f}, pitch:{:.2f}, yaw_vel:{:.2f}, pitch_vel:{:.2f}, "
    //     "bullet_speed:{:.2f}, bullet_count:{}, q:wxyz({:.3f},{:.3f},{:.3f},{:.3f})",
    //     mode, yaw, pitch, yaw_vel, pitch_vel, bullet_speed, bullet_count,
    //     q0, q1, q2, q3);

    auto t = std::chrono::steady_clock::now();

    error_count = 0;
    // 使用 yaw/pitch 计算四元数（ZYX 欧拉角顺序）
    Eigen::Quaterniond qyaw(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond qpitch(Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()));
    Eigen::Quaterniond q = qyaw * qpitch;
    // Eigen::Quaterniond q(rx_data_.q[3], rx_data_.q[0], rx_data_.q[1], rx_data_.q[2]);
    {
      std::lock_guard<std::mutex> lock(q_mutex_);
      if (q_buffer_.size() >= 1000) {
        q_buffer_.pop_front();
      }
      q_buffer_.push_back({q, t});
    }
    q_cv_.notify_all();

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    switch (mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      {
        std::lock_guard<std::mutex> lock(q_mutex_);
        q_buffer_.clear();
      }
      q_cv_.notify_all();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io