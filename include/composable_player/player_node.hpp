#ifndef COMPOSABLE_PLAYER__PLAYER_NODE_HPP_
#define COMPOSABLE_PLAYER__PLAYER_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace composable_player
{

class PlayerNode : public rclcpp::Node
{
public:
  explicit PlayerNode(const rclcpp::NodeOptions & options);
  ~PlayerNode() override;

private:
  void open_bag();
  void setup_publishers();
  void playback_callback();
  void clock_callback();
  void publish_clock(int64_t bag_time_ns);
  void reset_playback();

  void on_pause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void on_resume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  int64_t get_current_playback_time() const;

  // Parameters
  std::string bag_uri_;
  std::string storage_id_;
  double rate_;
  bool loop_;
  bool start_paused_;
  std::vector<std::string> topic_filter_;
  bool publish_clock_;
  double clock_frequency_;

  // Playback state
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  rosbag2_storage::StorageOptions storage_options_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publishers_;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_msg_;
  int64_t bag_start_time_;
  int64_t last_bag_time_ns_;
  std::chrono::steady_clock::time_point playback_wall_start_;
  bool paused_;
  bool finished_;
  std::chrono::steady_clock::time_point pause_wall_time_;
  std::mutex mutex_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr playback_timer_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
};

}  // namespace composable_player

#endif  // COMPOSABLE_PLAYER__PLAYER_NODE_HPP_
