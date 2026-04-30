// Hardware-oriented motion node (minimal): consumes /target_sequence and publishes PoseStamped
// commands using the blocks' real 3D positions. Keeps motion_node in src/ as a demo stub.

#include <deque>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <memory_game/Block.h>
#include <memory_game/BlockSequence.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace {

enum MotionState {
  IDLE,
  MOVING_TO_TARGET,
  AT_TARGET,
  RETURNING_HOME,
};

}  // namespace

class MotionHwNode {
 public:
  MotionHwNode() : pnh_("~") {
    pnh_.param("command_pose_topic", command_pose_topic_, std::string("/equilibrium_pose"));
    pnh_.param("pointing_offset_z", pointing_offset_z_, 0.0);
    pnh_.param("at_target_sec", at_target_sec_, 2.0);
    pnh_.param("hold_sec", hold_sec_, 1.0);
    pnh_.param("return_home", return_home_, true);
    pnh_.param("return_home_sec", return_home_sec_, 2.0);

    pnh_.param("home_x", home_pose_.pose.position.x, 0.3);
    pnh_.param("home_y", home_pose_.pose.position.y, 0.0);
    pnh_.param("home_z", home_pose_.pose.position.z, 0.4);
    pnh_.param("home_qx", home_pose_.pose.orientation.x, 0.0);
    pnh_.param("home_qy", home_pose_.pose.orientation.y, 0.0);
    pnh_.param("home_qz", home_pose_.pose.orientation.z, 0.0);
    pnh_.param("home_qw", home_pose_.pose.orientation.w, 1.0);

    home_pose_.header.frame_id = "panda_link0";
    state_ = IDLE;

    status_pub_ = nh_.advertise<std_msgs::String>("/motion_status", 10, true);
    cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(command_pose_topic_, 1, true);
    sequence_sub_ = nh_.subscribe("/target_sequence", 10, &MotionHwNode::sequenceCallback, this);

    publishStatus("IDLE");
    ROS_INFO("motion_hw_node ready: publishing PoseStamped to %s", command_pose_topic_.c_str());
  }

 private:
  void publishStatus(const std::string& state, int block_id = -1) {
    std_msgs::String msg;
    if (active_sequence_id_ >= 0) {
      msg.data = state + ":" + std::to_string(active_sequence_id_) + ":" + std::to_string(block_id);
    } else if (block_id >= 0) {
      msg.data = state + ":" + std::to_string(block_id);
    } else {
      msg.data = state;
    }
    status_pub_.publish(msg);
  }

  void sequenceCallback(const memory_game::BlockSequence::ConstPtr& msg) {
    if (!msg) {
      return;
    }

    target_queue_.clear();
    active_sequence_id_ = static_cast<int>(msg->sequence_id);
    for (const auto& block : msg->blocks) {
      target_queue_.push_back(block);
    }

    ROS_INFO("Queued sequence %d with %zu blocks", active_sequence_id_, msg->blocks.size());
    if (state_ == IDLE) {
      startNextTarget();
    }
  }

  void startNextTarget() {
    if (target_queue_.empty()) {
      state_ = IDLE;
      publishStatus("IDLE");
      ROS_INFO("Completed queued targets for sequence %d", active_sequence_id_);
      return;
    }

    const memory_game::Block next = target_queue_.front();
    target_queue_.pop_front();

    last_target_block_id_ = next.id;
    state_ = MOVING_TO_TARGET;
    publishStatus("MOVING_TO_TARGET", last_target_block_id_);

    geometry_msgs::PoseStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = !next.header.frame_id.empty() ? next.header.frame_id : "panda_link0";
    cmd.pose.position = next.position;
    cmd.pose.position.z += pointing_offset_z_;
    cmd.pose.orientation = home_pose_.pose.orientation;

    cmd_pub_.publish(cmd);

    at_target_timer_ = nh_.createTimer(
        ros::Duration(at_target_sec_), &MotionHwNode::atTargetTimerCb, this, true);
  }

  void atTargetTimerCb(const ros::TimerEvent&) {
    if (state_ != MOVING_TO_TARGET) {
      return;
    }
    state_ = AT_TARGET;
    publishStatus("AT_TARGET", last_target_block_id_);

    hold_timer_ =
        nh_.createTimer(ros::Duration(hold_sec_), &MotionHwNode::holdTimerCb, this, true);
  }

  void holdTimerCb(const ros::TimerEvent&) {
    if (state_ != AT_TARGET) {
      return;
    }

    if (!target_queue_.empty()) {
      startNextTarget();
      return;
    }

    if (!return_home_) {
      state_ = IDLE;
      publishStatus("IDLE");
      return;
    }

    state_ = RETURNING_HOME;
    publishStatus("RETURNING_HOME");

    home_pose_.header.stamp = ros::Time::now();
    cmd_pub_.publish(home_pose_);

    return_timer_ = nh_.createTimer(
        ros::Duration(return_home_sec_), &MotionHwNode::returnTimerCb, this, true);
  }

  void returnTimerCb(const ros::TimerEvent&) {
    if (state_ != RETURNING_HOME) {
      return;
    }
    state_ = IDLE;
    publishStatus("IDLE");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher status_pub_;
  ros::Publisher cmd_pub_;
  ros::Subscriber sequence_sub_;

  std::deque<memory_game::Block> target_queue_;
  MotionState state_;
  int last_target_block_id_ = -1;
  int active_sequence_id_ = -1;

  std::string command_pose_topic_;
  double pointing_offset_z_ = 0.0;
  double at_target_sec_ = 2.0;
  double hold_sec_ = 1.0;
  bool return_home_ = true;
  double return_home_sec_ = 2.0;

  geometry_msgs::PoseStamped home_pose_;

  ros::Timer at_target_timer_;
  ros::Timer hold_timer_;
  ros::Timer return_timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_hw_node");
  MotionHwNode node;
  ros::spin();
  return 0;
}
