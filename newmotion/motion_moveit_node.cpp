// MoveIt-backed motion executor for the memory game.
//
// Sequence contract stays unchanged:
//   - subscribes to /target_sequence
//   - publishes /motion_status with sequence_id and block_id
//
// Motion behavior:
//   1. plan + execute to hover pose
//   2. plan + execute to a lower indicate pose
//   3. plan + execute back to hover pose
//
// This intentionally avoids computeCartesianPath(), because the Panda lab setup
// was smoother and more reliable with standard MoveIt planned motions.

#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <memory_game/Block.h>
#include <memory_game/BlockSequence.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace {

void PublishStatus(ros::Publisher& pub,
                   const std::string& state,
                   int sequence_id = -1,
                   int block_id = -1) {
  std_msgs::String msg;
  std::ostringstream oss;
  oss << state;
  if (sequence_id >= 0) {
    oss << ":" << sequence_id << ":" << block_id;
  } else if (block_id >= 0) {
    oss << ":" << block_id;
  }
  msg.data = oss.str();
  pub.publish(msg);
}

}  // namespace

class MotionMoveItNode {
 public:
  MotionMoveItNode() : pnh_("~"), spinner_(2) {
    pnh_.param("planning_group", planning_group_, std::string("arm"));
    pnh_.param("pose_frame", pose_frame_, std::string("panda_link0"));

    pnh_.param("planning_time", planning_time_, 5.0);
    pnh_.param("max_plan_retries", max_plan_retries_, 3);

    pnh_.param("max_velocity_scaling", max_velocity_scaling_, 0.08);
    pnh_.param("max_acceleration_scaling", max_acceleration_scaling_, 0.05);
    pnh_.param("point_velocity_scaling", point_velocity_scaling_, 0.05);
    pnh_.param("point_acceleration_scaling", point_acceleration_scaling_, 0.03);

    pnh_.param("hover_target_z", hover_target_z_, 0.28);
    pnh_.param("hover_min_z", hover_min_z_, 0.25);
    pnh_.param("hover_max_z", hover_max_z_, 0.30);
    pnh_.param("point_dip_distance_z", point_dip_distance_z_, 0.10);
    pnh_.param("point_min_clearance_z", point_min_clearance_z_, 0.06);

    pnh_.param("indicate_hold_sec", indicate_hold_sec_, 0.20);
    pnh_.param("strict_pointing", strict_pointing_, false);

    pnh_.param("return_home", return_home_, true);
    pnh_.param("use_current_state_as_home", use_current_state_as_home_, true);
    pnh_.param("require_pose_frame_match", require_pose_frame_match_, true);
    pnh_.param("workspace_enable", workspace_enable_, false);
    pnh_.param("workspace_min_x", workspace_min_x_, -10.0);
    pnh_.param("workspace_max_x", workspace_max_x_, 10.0);
    pnh_.param("workspace_min_y", workspace_min_y_, -10.0);
    pnh_.param("workspace_max_y", workspace_max_y_, 10.0);
    pnh_.param("workspace_min_z", workspace_min_z_, -10.0);
    pnh_.param("workspace_max_z", workspace_max_z_, 10.0);
    pnh_.param("keep_current_orientation", keep_current_orientation_, false);

    status_pub_ = nh_.advertise<std_msgs::String>("/motion_status", 10, true);

    spinner_.start();
    PublishStatus(status_pub_, "INIT");

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(planning_group_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setPoseReferenceFrame(pose_frame_);
    move_group_->setGoalPositionTolerance(0.005);
    move_group_->setGoalOrientationTolerance(0.05);
    move_group_->setGoalJointTolerance(0.005);

    configureHomeTarget();
    cacheReferencePose();

    worker_thread_ = std::thread([this]() { workerLoop(); });
    sequence_sub_ = nh_.subscribe("/target_sequence", 10, &MotionMoveItNode::sequenceCb, this);

    PublishStatus(status_pub_, "IDLE");
    ROS_INFO("motion_moveit_node ready");
    ROS_INFO("planning_group=%s", planning_group_.c_str());
    ROS_INFO("pose_frame=%s", pose_frame_.c_str());
  }

  ~MotionMoveItNode() {
    {
      std::lock_guard<std::mutex> lk(mu_);
      shutting_down_ = true;
      pending_sequences_.clear();
    }
    queue_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

 private:
  struct SequenceJob {
    uint32_t sequence_id = 0;
    std::vector<memory_game::Block> blocks;
  };

  void sequenceCb(const memory_game::BlockSequence::ConstPtr& msg) {
    if (!msg) {
      return;
    }

    SequenceJob job;
    job.sequence_id = msg->sequence_id;
    job.blocks.assign(msg->blocks.begin(), msg->blocks.end());

    {
      std::lock_guard<std::mutex> lk(mu_);
      if (shutting_down_) {
        return;
      }
      pending_sequences_.push_back(job);
    }

    ROS_INFO("Queued sequence %u with %zu targets", job.sequence_id, job.blocks.size());
    queue_cv_.notify_one();
  }

  void workerLoop() {
    while (true) {
      SequenceJob job;
      {
        std::unique_lock<std::mutex> lk(mu_);
        queue_cv_.wait(lk, [this]() { return shutting_down_ || !pending_sequences_.empty(); });
        if (shutting_down_) {
          return;
        }
        job = pending_sequences_.front();
        pending_sequences_.pop_front();
      }

      if (!executeSequence(job)) {
        continue;
      }

      PublishStatus(status_pub_, "IDLE", static_cast<int>(job.sequence_id));
    }
  }

  void failAndStopQueue(const std::string& reason, uint32_t sequence_id, int block_id = -1) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      pending_sequences_.clear();
    }
    ROS_ERROR("%s", reason.c_str());
    PublishStatus(status_pub_, "MOVE_FAILED", static_cast<int>(sequence_id), block_id);
    PublishStatus(status_pub_, "IDLE", static_cast<int>(sequence_id));
  }

  bool executeSequence(const SequenceJob& job) {
    if (job.blocks.empty()) {
      ROS_WARN("Sequence %u is empty", job.sequence_id);
      return true;
    }

    for (const auto& block : job.blocks) {
      if (!executeTarget(job.sequence_id, block)) {
        return false;
      }
    }

    if (!return_home_) {
      return true;
    }

    PublishStatus(status_pub_, "RETURNING_HOME", static_cast<int>(job.sequence_id));
    if (!returnHome()) {
      failAndStopQueue("Return-home motion failed", job.sequence_id);
      return false;
    }

    return true;
  }

  bool executeTarget(uint32_t sequence_id, const memory_game::Block& block) {
    if (!move_group_) {
      failAndStopQueue("MoveGroupInterface not initialized", sequence_id, block.id);
      return false;
    }

    const std::string frame = !block.header.frame_id.empty() ? block.header.frame_id : pose_frame_;
    if (!validateTarget(block, frame)) {
      failAndStopQueue("Target validation failed", sequence_id, block.id);
      return false;
    }

    move_group_->setPoseReferenceFrame(frame);

    const geometry_msgs::Pose hover_pose = buildHoverPose(block);
    const geometry_msgs::Pose point_pose = buildPointPose(block, hover_pose);

    if (workspace_enable_ &&
        (!withinWorkspace(hover_pose.position) || !withinWorkspace(point_pose.position))) {
      ROS_ERROR("Rejecting block %d in sequence %u: hover/point pose outside workspace",
                block.id, sequence_id);
      failAndStopQueue("Target pose outside workspace", sequence_id, block.id);
      return false;
    }

    PublishStatus(status_pub_, "MOVING_TO_TARGET", static_cast<int>(sequence_id), block.id);
    if (!moveToPoseWithRetries(hover_pose,
                               max_velocity_scaling_,
                               max_acceleration_scaling_,
                               block.id,
                               "hover")) {
      failAndStopQueue("Hover move failed", sequence_id, block.id);
      return false;
    }

    bool point_succeeded = false;
    PublishStatus(status_pub_, "POINTING", static_cast<int>(sequence_id), block.id);
    if (moveToPoseWithRetries(point_pose,
                              point_velocity_scaling_,
                              point_acceleration_scaling_,
                              block.id,
                              "point")) {
      point_succeeded = true;
      if (indicate_hold_sec_ > 0.0) {
        ros::Duration(indicate_hold_sec_).sleep();
      }
    } else if (strict_pointing_) {
      failAndStopQueue("Point move failed", sequence_id, block.id);
      return false;
    } else {
      ROS_WARN("Point move failed for block %d; using hover-only indication for this target", block.id);
      if (indicate_hold_sec_ > 0.0) {
        ros::Duration(indicate_hold_sec_).sleep();
      }
    }

    PublishStatus(status_pub_, "AT_TARGET", static_cast<int>(sequence_id), block.id);

    if (point_succeeded) {
      if (!moveToPoseWithRetries(hover_pose,
                                 point_velocity_scaling_,
                                 point_acceleration_scaling_,
                                 block.id,
                                 "ascend")) {
        if (strict_pointing_) {
          failAndStopQueue("Ascend move failed", sequence_id, block.id);
          return false;
        }
        ROS_WARN("Ascend move failed for block %d; continuing from current state", block.id);
      }
    }

    return true;
  }

  bool validateTarget(const memory_game::Block& block, const std::string& frame) const {
    if (!std::isfinite(block.position.x) ||
        !std::isfinite(block.position.y) ||
        !std::isfinite(block.position.z)) {
      ROS_ERROR("Rejecting block %d: non-finite target position [%.3f, %.3f, %.3f]",
                block.id, block.position.x, block.position.y, block.position.z);
      return false;
    }

    if (require_pose_frame_match_ && frame != pose_frame_) {
      ROS_ERROR("Rejecting block %d: expected frame %s but got %s",
                block.id, pose_frame_.c_str(), frame.c_str());
      return false;
    }

    if (workspace_enable_ && frame != pose_frame_) {
      ROS_ERROR("Rejecting block %d: workspace checks require pose frame %s but target is in %s",
                block.id, pose_frame_.c_str(), frame.c_str());
      return false;
    }

    return true;
  }

  bool withinWorkspace(const geometry_msgs::Point& p) const {
    return p.x >= workspace_min_x_ && p.x <= workspace_max_x_ &&
           p.y >= workspace_min_y_ && p.y <= workspace_max_y_ &&
           p.z >= workspace_min_z_ && p.z <= workspace_max_z_;
  }

  geometry_msgs::Pose orientationReferencePose() {
    if (keep_current_orientation_) {
      return move_group_->getCurrentPose().pose;
    }
    return reference_pose_;
  }

  geometry_msgs::Pose buildHoverPose(const memory_game::Block& block) {
    geometry_msgs::Pose pose = orientationReferencePose();
    pose.position.x = block.position.x;
    pose.position.y = block.position.y;
    const double min_safe_hover_z = block.position.z + point_min_clearance_z_ + point_dip_distance_z_;
    const double desired_hover_z = std::max(hover_target_z_, min_safe_hover_z);
    pose.position.z = std::min(hover_max_z_, std::max(hover_min_z_, desired_hover_z));
    return pose;
  }

  geometry_msgs::Pose buildPointPose(const memory_game::Block& block,
                                     const geometry_msgs::Pose& hover_pose) {
    geometry_msgs::Pose pose = orientationReferencePose();
    pose.position.x = block.position.x;
    pose.position.y = block.position.y;

    const double min_safe_point_z = block.position.z + point_min_clearance_z_;
    const double desired_point_z = hover_pose.position.z - point_dip_distance_z_;
    pose.position.z = std::max(min_safe_point_z, desired_point_z);

    return pose;
  }

  bool moveToPoseWithRetries(const geometry_msgs::Pose& target_pose,
                             double velocity_scaling,
                             double acceleration_scaling,
                             int block_id,
                             const std::string& label) {
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);

    for (int attempt = 1; attempt <= max_plan_retries_; ++attempt) {
      move_group_->setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned =
          (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (!planned) {
        ROS_WARN("%s planning failed for block %d (attempt %d/%d)",
                 label.c_str(), block_id, attempt, max_plan_retries_);
        move_group_->clearPoseTargets();
        continue;
      }

      const bool executed =
          (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      move_group_->stop();
      move_group_->clearPoseTargets();

      if (executed) {
        return true;
      }

      ROS_WARN("%s execution failed for block %d (attempt %d/%d)",
               label.c_str(), block_id, attempt, max_plan_retries_);
      ros::Duration(0.15 * attempt).sleep();
    }

    return false;
  }

  bool returnHome() {
    if (!move_group_) {
      return false;
    }

    if (home_joint_values_.empty()) {
      ROS_WARN("Return-home requested but no home joint target is available");
      return false;
    }

    move_group_->setJointValueTarget(home_joint_values_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

    for (int attempt = 1; attempt <= max_plan_retries_; ++attempt) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned =
          (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (!planned) {
        ROS_WARN("Planning home failed (attempt %d/%d)", attempt, max_plan_retries_);
        continue;
      }

      const bool executed =
          (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      move_group_->stop();
      if (executed) {
        return true;
      }

      ROS_WARN("Executing home failed (attempt %d/%d)", attempt, max_plan_retries_);
      ros::Duration(0.15 * attempt).sleep();
    }

    return false;
  }

  void cacheReferencePose() {
    reference_pose_ = move_group_->getCurrentPose().pose;

    if (home_joint_values_.empty()) {
      ROS_WARN("Using current pose as orientation reference: no home joint target available");
      return;
    }

    auto robot_state = move_group_->getCurrentState();
    if (!robot_state) {
      ROS_WARN("Using current pose as orientation reference: no robot state available");
      return;
    }

    const moveit::core::JointModelGroup* joint_model_group =
        robot_state->getJointModelGroup(planning_group_);
    if (!joint_model_group) {
      ROS_WARN("Using current pose as orientation reference: joint model group %s not found",
               planning_group_.c_str());
      return;
    }

    std::string link_name = move_group_->getEndEffectorLink();
    if (link_name.empty()) {
      const std::vector<std::string>& link_names = move_group_->getLinkNames();
      if (!link_names.empty()) {
        link_name = link_names.back();
      }
    }

    if (link_name.empty()) {
      ROS_WARN("Using current pose as orientation reference: end-effector link is unknown");
      return;
    }

    robot_state->setJointGroupPositions(joint_model_group, home_joint_values_);
    robot_state->update();

    const Eigen::Isometry3d& link_tf = robot_state->getGlobalLinkTransform(link_name);
    const Eigen::Quaterniond q(link_tf.rotation());

    reference_pose_.position.x = link_tf.translation().x();
    reference_pose_.position.y = link_tf.translation().y();
    reference_pose_.position.z = link_tf.translation().z();
    reference_pose_.orientation.x = q.x();
    reference_pose_.orientation.y = q.y();
    reference_pose_.orientation.z = q.z();
    reference_pose_.orientation.w = q.w();

    ROS_INFO("Cached orientation reference from home target using link %s", link_name.c_str());
  }

  void configureHomeTarget() {
    std::vector<double> configured_home;
    const size_t joint_count = move_group_->getCurrentJointValues().size();
    if (pnh_.getParam("home_joint_values", configured_home) && !configured_home.empty()) {
      if (configured_home.size() == joint_count) {
        home_joint_values_ = configured_home;
        ROS_INFO("Using configured home_joint_values for return-home");
        return;
      }
      ROS_WARN("Ignoring home_joint_values: expected %zu joints, got %zu",
               joint_count, configured_home.size());
    }

    if (use_current_state_as_home_) {
      home_joint_values_ = move_group_->getCurrentJointValues();
      ROS_INFO("Using current robot state as home target");
    } else {
      return_home_ = false;
      ROS_ERROR("Return-home disabled: set ~home_joint_values or enable ~use_current_state_as_home");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher status_pub_;
  ros::Subscriber sequence_sub_;
  ros::AsyncSpinner spinner_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::vector<double> home_joint_values_;
  geometry_msgs::Pose reference_pose_;

  std::string planning_group_;
  std::string pose_frame_;

  double planning_time_ = 5.0;
  int max_plan_retries_ = 3;

  double max_velocity_scaling_ = 0.08;
  double max_acceleration_scaling_ = 0.05;
  double point_velocity_scaling_ = 0.05;
  double point_acceleration_scaling_ = 0.03;

  double hover_target_z_ = 0.28;
  double hover_min_z_ = 0.25;
  double hover_max_z_ = 0.30;
  double point_dip_distance_z_ = 0.10;
  double point_min_clearance_z_ = 0.06;

  double indicate_hold_sec_ = 0.20;
  bool strict_pointing_ = false;
  bool return_home_ = true;
  bool use_current_state_as_home_ = true;
  bool require_pose_frame_match_ = true;
  bool workspace_enable_ = false;
  double workspace_min_x_ = -10.0;
  double workspace_max_x_ = 10.0;
  double workspace_min_y_ = -10.0;
  double workspace_max_y_ = 10.0;
  double workspace_min_z_ = -10.0;
  double workspace_max_z_ = 10.0;
  bool keep_current_orientation_ = false;

  std::mutex mu_;
  std::condition_variable queue_cv_;
  std::deque<SequenceJob> pending_sequences_;
  std::thread worker_thread_;
  bool shutting_down_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_moveit_node");
  MotionMoveItNode node;
  ros::waitForShutdown();
  return 0;
}
