#include "composable_player/player_node.hpp"

#include <algorithm>
#include <cstring>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"

namespace composable_player
{

PlayerNode::PlayerNode(const rclcpp::NodeOptions & options)
: Node("player", options),
  paused_(false),
  finished_(false)
{
  declare_parameter<std::string>("bag_uri", "");
  declare_parameter<std::string>("storage_id", "mcap");
  declare_parameter<double>("rate", 1.0);
  declare_parameter<bool>("loop", false);
  declare_parameter<bool>("start_paused", false);
  declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});

  bag_uri_ = get_parameter("bag_uri").as_string();
  storage_id_ = get_parameter("storage_id").as_string();
  rate_ = get_parameter("rate").as_double();
  loop_ = get_parameter("loop").as_bool();
  start_paused_ = get_parameter("start_paused").as_bool();
  topic_filter_ = get_parameter("topics").as_string_array();

  if (bag_uri_.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'bag_uri' is required");
    return;
  }

  if (rate_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "Invalid rate %.2f, defaulting to 1.0", rate_);
    rate_ = 1.0;
  }

  pause_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/pause",
    std::bind(&PlayerNode::on_pause, this, std::placeholders::_1, std::placeholders::_2));

  resume_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/resume",
    std::bind(&PlayerNode::on_resume, this, std::placeholders::_1, std::placeholders::_2));

  open_bag();
  setup_publishers();

  paused_ = start_paused_;
  playback_wall_start_ = std::chrono::steady_clock::now();

  playback_timer_ = create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&PlayerNode::playback_callback, this));

  RCLCPP_INFO(
    get_logger(), "PlayerNode initialized: uri=%s storage=%s rate=%.2f loop=%s",
    bag_uri_.c_str(), storage_id_.c_str(), rate_, loop_ ? "true" : "false");
}

PlayerNode::~PlayerNode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  reader_.reset();
}

void PlayerNode::open_bag()
{
  storage_options_.uri = bag_uri_;
  storage_options_.storage_id = storage_id_;

  reader_ = std::make_unique<rosbag2_cpp::Reader>();
  reader_->open(storage_options_);

  auto metadata = reader_->get_metadata();
  auto start_time = metadata.starting_time;
  bag_start_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    start_time.time_since_epoch()).count();

  RCLCPP_INFO(
    get_logger(), "Opened bag: %zu topics, %zu messages",
    metadata.topics_with_message_count.size(), metadata.message_count);
}

void PlayerNode::setup_publishers()
{
  std::unordered_set<std::string> filter_set(topic_filter_.begin(), topic_filter_.end());
  bool use_filter = !filter_set.empty();

  auto topics = reader_->get_all_topics_and_types();
  for (const auto & topic_info : topics) {
    const auto & name = topic_info.topic_metadata.name;
    const auto & type = topic_info.topic_metadata.type;

    if (use_filter && filter_set.find(name) == filter_set.end()) {
      continue;
    }

    auto publisher = create_generic_publisher(name, type, rclcpp::QoS(10));
    publishers_[name] = publisher;
    RCLCPP_INFO(get_logger(), "Publishing: %s [%s]", name.c_str(), type.c_str());
  }
}

void PlayerNode::playback_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (paused_ || finished_) {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  auto wall_elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now - playback_wall_start_).count();
  int64_t playback_time = bag_start_time_ +
    static_cast<int64_t>(static_cast<double>(wall_elapsed_ns) * rate_);

  int messages_published = 0;
  const int max_batch = 100;

  while (messages_published < max_batch) {
    if (!next_msg_) {
      if (!reader_->has_next()) {
        break;
      }
      next_msg_ = reader_->read_next();
    }

    if (next_msg_->time_stamp > playback_time) {
      break;
    }

    auto it = publishers_.find(next_msg_->topic_name);
    if (it != publishers_.end()) {
      auto serialized_data = next_msg_->serialized_data;
      rclcpp::SerializedMessage serialized_msg(serialized_data->buffer_length);
      auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
      std::memcpy(rcl_msg.buffer, serialized_data->buffer, serialized_data->buffer_length);
      rcl_msg.buffer_length = serialized_data->buffer_length;
      it->second->publish(serialized_msg);
    }

    next_msg_.reset();
    ++messages_published;
  }

  if (!reader_->has_next() && !next_msg_) {
    if (loop_) {
      reset_playback();
      RCLCPP_INFO(get_logger(), "Looping playback");
    } else {
      finished_ = true;
      RCLCPP_INFO(get_logger(), "Playback finished");
    }
  }
}

void PlayerNode::reset_playback()
{
  reader_.reset();
  reader_ = std::make_unique<rosbag2_cpp::Reader>();
  reader_->open(storage_options_);

  auto metadata = reader_->get_metadata();
  bag_start_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    metadata.starting_time.time_since_epoch()).count();

  next_msg_.reset();
  playback_wall_start_ = std::chrono::steady_clock::now();
  finished_ = false;
}

void PlayerNode::on_pause(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!paused_) {
    paused_ = true;
    pause_wall_time_ = std::chrono::steady_clock::now();
    response->success = true;
    response->message = "Playback paused";
    RCLCPP_INFO(get_logger(), "Paused");
  } else {
    response->success = false;
    response->message = "Already paused";
  }
}

void PlayerNode::on_resume(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (paused_) {
    auto pause_duration = std::chrono::steady_clock::now() - pause_wall_time_;
    playback_wall_start_ += pause_duration;
    paused_ = false;
    response->success = true;
    response->message = "Playback resumed";
    RCLCPP_INFO(get_logger(), "Resumed");
  } else {
    response->success = false;
    response->message = "Not paused";
  }
}

}  // namespace composable_player

RCLCPP_COMPONENTS_REGISTER_NODE(composable_player::PlayerNode)
