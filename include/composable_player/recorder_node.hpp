#ifndef COMPOSABLE_PLAYER__RECORDER_NODE_HPP_
#define COMPOSABLE_PLAYER__RECORDER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace composable_player
{

class RecorderNode : public rclcpp::Node
{
public:
  explicit RecorderNode(const rclcpp::NodeOptions & options);
  ~RecorderNode() override;

private:
  void discover_and_subscribe();
  void subscribe_to_topic(const std::string & topic_name, const std::string & topic_type);
  void write_message(
    std::shared_ptr<rclcpp::SerializedMessage> msg,
    const std::string & topic_name);

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter> & parameters);

  // Parameters
  std::string output_uri_;
  std::string storage_id_;
  std::vector<std::string> topics_;
  bool all_topics_;
  std::vector<std::string> exclude_topics_;
  uint64_t max_bag_size_;
  int64_t max_bag_duration_;

  // Recording state
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::unordered_set<std::string> subscribed_topics_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  bool recording_;
  std::mutex mutex_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace composable_player

#endif  // COMPOSABLE_PLAYER__RECORDER_NODE_HPP_
