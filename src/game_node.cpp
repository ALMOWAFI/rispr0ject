#include <ros/ros.h>
#include <memory_game/BlockArray.h>
#include <memory_game/BlockSequence.h>
#include <memory_game/PlayerSelection.h>
#include <memory_game/Block.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

enum class GameState {
    IDLE,
    SHOWING_SEQUENCE,
    WAITING_PLAYER,
    CHECKING_INPUT,
    MOTION_FAILED,
    GAME_OVER
};

class GameNode {
private:
    struct MotionStatus {
        std::string state;
        int block_id = -1;
        int sequence_id = -1;
        bool has_block_id = false;
        bool has_sequence_id = false;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber blocks_sub_;
    ros::Subscriber selection_sub_;
    ros::Subscriber motion_sub_;

    ros::Publisher target_sequence_pub_;
    ros::Publisher state_pub_;
    ros::Publisher score_pub_;

    ros::Timer start_timer_;
    ros::Timer show_failsafe_timer_;
    ros::Timer round_timer_;
    ros::Timer player_timeout_timer_;
    ros::Timer target_sub_wait_timer_;
    ros::Timer motion_retry_timer_;

    GameState current_state_;

    std::vector<int> sequence_;
    std::vector<int> player_input_;
    std::map<int, memory_game::Block> known_blocks_;
    std::map<int, ros::Time> block_seen_times_;

    int score_;
    int level_;
    int player_index_;

    bool waiting_for_motion_;
    bool motion_started_for_sequence_;
    bool have_motion_state_;
    std::string last_motion_state_;

    std::mt19937 gen_;

    int num_blocks_;
    int base_length_;
    int length_per_level_;
    double round_pause_sec_;
    double player_timeout_sec_;
    double start_delay_sec_;
    bool no_immediate_repeat_;
    bool require_detected_blocks_;
    bool disable_red_;
    int min_detected_blocks_required_;
    double blocks_wait_sec_;
    double block_freshness_sec_;
    double target_subscriber_poll_sec_;
    double target_subscriber_timeout_sec_;
    double motion_timeout_per_target_sec_;
    double motion_timeout_padding_sec_;
    int motion_retry_limit_;
    double motion_retry_delay_sec_;

    geometry_msgs::Point default_target_;
    std::string target_frame_;
    std::vector<int> available_block_ids_;
    bool sequence_sent_to_motion_;
    int sequence_targets_completed_;
    ros::Time target_sub_wait_start_;
    uint32_t next_sequence_id_;
    uint32_t active_sequence_id_;
    memory_game::BlockSequence active_sequence_msg_;
    bool active_sequence_msg_valid_;
    int motion_retry_count_;

public:
    GameNode()
        : pnh_("~"),
          current_state_(GameState::IDLE),
          score_(0),
          level_(1),
          player_index_(0),
          waiting_for_motion_(false),
          motion_started_for_sequence_(false),
          have_motion_state_(false),
          gen_(std::random_device{}()),
          sequence_sent_to_motion_(false),
          sequence_targets_completed_(0),
          next_sequence_id_(1),
          active_sequence_id_(0),
          active_sequence_msg_valid_(false),
          motion_retry_count_(0) {
        loadParams();

        blocks_sub_ = nh_.subscribe("/detected_blocks", 10, &GameNode::blocksCallback, this);
        selection_sub_ = nh_.subscribe("/player_selection", 10, &GameNode::selectionCallback, this);
        motion_sub_ = nh_.subscribe("/motion_status", 10, &GameNode::motionCallback, this);

        target_sequence_pub_ = nh_.advertise<memory_game::BlockSequence>("/target_sequence", 10);
        state_pub_ = nh_.advertise<std_msgs::String>("/game_state", 10, true);
        score_pub_ = nh_.advertise<std_msgs::Int32>("/score", 10, true);

        publishState("IDLE");
        publishScore();

        start_timer_ = nh_.createTimer(
            ros::Duration(start_delay_sec_),
            &GameNode::startTimerCallback,
            this,
            true);

        ROS_INFO("Game node initialized");
    }

private:
    void loadParams() {
        pnh_.param("num_blocks", num_blocks_, 4);
        pnh_.param("base_length", base_length_, 3);
        pnh_.param("length_per_level", length_per_level_, 1);
        pnh_.param("round_pause_sec", round_pause_sec_, 1.0);
        if (!pnh_.getParam("player_timeout_sec", player_timeout_sec_)) {
            if (!pnh_.getParam("player_timeout", player_timeout_sec_)) {
                player_timeout_sec_ = 12.0;
            }
        }
        pnh_.param("start_delay_sec", start_delay_sec_, 0.8);
        pnh_.param("no_immediate_repeat", no_immediate_repeat_, true);
        pnh_.param("target_frame", target_frame_, std::string("panda_link0"));
        pnh_.param("require_detected_blocks", require_detected_blocks_, true);
        pnh_.param("disable_red", disable_red_, true);
        pnh_.param("min_detected_blocks_required", min_detected_blocks_required_, 3);
        pnh_.param("blocks_wait_sec", blocks_wait_sec_, 0.5);
        pnh_.param("block_freshness_sec", block_freshness_sec_, 0.20);
        pnh_.param("target_subscriber_poll_sec", target_subscriber_poll_sec_, 0.2);
        pnh_.param("target_subscriber_timeout_sec", target_subscriber_timeout_sec_, 5.0);
        pnh_.param("motion_timeout_per_target_sec", motion_timeout_per_target_sec_, 15.0);
        pnh_.param("motion_timeout_padding_sec", motion_timeout_padding_sec_, 2.0);
        pnh_.param("motion_retry_limit", motion_retry_limit_, 1);
        pnh_.param("motion_retry_delay_sec", motion_retry_delay_sec_, 0.75);

        pnh_.param("default_x", default_target_.x, 0.40);
        pnh_.param("default_y", default_target_.y, 0.00);
        pnh_.param("default_z", default_target_.z, 0.05);

        int seed = -1;
        pnh_.param("seed", seed, -1);
        if (seed >= 0) {
            gen_.seed(static_cast<unsigned int>(seed));
        }

        num_blocks_ = std::max(1, num_blocks_);
        base_length_ = std::max(1, base_length_);
        length_per_level_ = std::max(0, length_per_level_);
        if (player_timeout_sec_ < 0.0) {
            player_timeout_sec_ = 0.0;
        }
        blocks_wait_sec_ = std::max(0.05, blocks_wait_sec_);
        block_freshness_sec_ = std::max(0.0, block_freshness_sec_);
        min_detected_blocks_required_ = std::max(1, min_detected_blocks_required_);
        target_subscriber_poll_sec_ = std::max(0.05, target_subscriber_poll_sec_);
        target_subscriber_timeout_sec_ = std::max(0.0, target_subscriber_timeout_sec_);
        motion_timeout_per_target_sec_ = std::max(1.0, motion_timeout_per_target_sec_);
        motion_timeout_padding_sec_ = std::max(0.0, motion_timeout_padding_sec_);
        motion_retry_limit_ = std::max(0, motion_retry_limit_);
        motion_retry_delay_sec_ = std::max(0.0, motion_retry_delay_sec_);

        available_block_ids_.clear();
        for (int id = 0; id < num_blocks_; ++id) {
            if (disable_red_ && id == 0) {
                continue;
            }
            available_block_ids_.push_back(id);
        }

        if (available_block_ids_.empty()) {
            ROS_WARN("disable_red removed all available blocks; falling back to block 0");
            available_block_ids_.push_back(0);
        }
    }

    bool isBlockFresh(int id, const ros::Time& now) const {
        auto it = known_blocks_.find(id);
        if (it == known_blocks_.end()) {
            return false;
        }

        auto seen_it = block_seen_times_.find(id);
        if (seen_it == block_seen_times_.end()) {
            return false;
        }

        if (block_freshness_sec_ <= 0.0) {
            return true;
        }

        return (now - seen_it->second).toSec() <= block_freshness_sec_;
    }

    int freshBlockCount(const ros::Time& now) const {
        int count = 0;
        for (const auto& entry : known_blocks_) {
            if (isBlockFresh(entry.first, now)) {
                ++count;
            }
        }
        return count;
    }

    void pruneStaleBlocks(const ros::Time& now) {
        if (block_freshness_sec_ <= 0.0) {
            return;
        }

        for (auto it = block_seen_times_.begin(); it != block_seen_times_.end();) {
            if ((now - it->second).toSec() > block_freshness_sec_) {
                known_blocks_.erase(it->first);
                it = block_seen_times_.erase(it);
            } else {
                ++it;
            }
        }
    }

    std::vector<int> freshDetectedBlockIds() const {
        const ros::Time now = ros::Time::now();
        std::vector<int> ids;
        ids.reserve(known_blocks_.size());

        for (const auto& entry : known_blocks_) {
            const int id = entry.first;
            if (disable_red_ && id == 0) {
                continue;
            }
            if (id < 0 || id >= num_blocks_) {
                continue;
            }
            if (!isBlockFresh(id, now)) {
                continue;
            }
            ids.push_back(id);
        }

        std::sort(ids.begin(), ids.end());
        return ids;
    }

    bool refreshAvailableBlocksFromDetections() {
        if (!require_detected_blocks_) {
            return true;
        }

        std::vector<int> detected_ids = freshDetectedBlockIds();
        if (static_cast<int>(detected_ids.size()) < min_detected_blocks_required_) {
            return false;
        }

        available_block_ids_ = detected_ids;
        return true;
    }

    void blocksCallback(const memory_game::BlockArray::ConstPtr& msg) {
        const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        for (const auto& block : msg->blocks) {
            known_blocks_[block.id] = block;
            block_seen_times_[block.id] = stamp;
        }
        pruneStaleBlocks(stamp);
    }

    MotionStatus parseMotionStatus(const std::string& raw) const {
        MotionStatus out;
        out.state = raw;

        const std::size_t first_colon = raw.find(':');
        if (first_colon == std::string::npos) {
            return out;
        }

        out.state = raw.substr(0, first_colon);
        const std::string rest = raw.substr(first_colon + 1);
        const std::size_t second_colon = rest.find(':');

        if (second_colon == std::string::npos) {
            std::istringstream iss(rest);
            out.has_block_id = static_cast<bool>(iss >> out.block_id);
            return out;
        }

        std::istringstream seq_iss(rest.substr(0, second_colon));
        out.has_sequence_id = static_cast<bool>(seq_iss >> out.sequence_id);

        std::istringstream block_iss(rest.substr(second_colon + 1));
        out.has_block_id = static_cast<bool>(block_iss >> out.block_id);

        return out;
    }

    void motionCallback(const std_msgs::String::ConstPtr& msg) {
        last_motion_state_ = msg->data;
        have_motion_state_ = true;

        const MotionStatus status = parseMotionStatus(msg->data);

        if (current_state_ != GameState::SHOWING_SEQUENCE || !waiting_for_motion_) {
            return;
        }

        if (status.has_sequence_id && status.sequence_id != static_cast<int>(active_sequence_id_)) {
            ROS_WARN("Ignoring stale motion status for sequence %d while waiting for %u",
                     status.sequence_id, active_sequence_id_);
            return;
        }

        if (status.state == "MOVE_FAILED") {
            if (status.has_block_id) {
                scheduleMotionRetryOrAbort("Motion reported failure while executing block " +
                                           std::to_string(status.block_id));
            } else {
                scheduleMotionRetryOrAbort("Motion reported sequence failure");
            }
            return;
        }

        if (status.state == "MOVING_TO_TARGET" ||
            status.state == "POINTING" ||
            status.state == "AT_TARGET" ||
            status.state == "RETURNING_HOME") {
            motion_started_for_sequence_ = true;
        }

        if (status.state == "AT_TARGET") {
            const int expected_targets = static_cast<int>(sequence_.size());
            if (sequence_targets_completed_ >= expected_targets) {
                ROS_WARN("Received extra AT_TARGET after sequence completion (%d expected)",
                         expected_targets);
            } else {
                const int expected_block_id = sequence_[sequence_targets_completed_];
                if (status.has_block_id && status.block_id != expected_block_id) {
                    ROS_WARN("AT_TARGET: got block %d but expected block %d at step %d/%d",
                             status.block_id,
                             expected_block_id,
                             sequence_targets_completed_ + 1,
                             expected_targets);
                    abortDueToMotionFailure();
                    return;
                }

                ++sequence_targets_completed_;
                ROS_INFO("Motion completed target %d/%d",
                         sequence_targets_completed_,
                         expected_targets);
            }
        }

        if (status.state == "IDLE" && motion_started_for_sequence_) {
            show_failsafe_timer_.stop();
            waiting_for_motion_ = false;
            motion_started_for_sequence_ = false;

            if (sequence_targets_completed_ < static_cast<int>(sequence_.size())) {
                scheduleMotionRetryOrAbort("Motion returned to IDLE after only " +
                                           std::to_string(sequence_targets_completed_) + "/" +
                                           std::to_string(sequence_.size()) + " targets");
                return;
            }

            current_state_ = GameState::WAITING_PLAYER;
            publishState("WAITING_PLAYER");
            ROS_INFO("Sequence complete. Waiting for player input...");
            resetPlayerTimeout();
        }
    }

    void selectionCallback(const memory_game::PlayerSelection::ConstPtr& msg) {
        if (current_state_ != GameState::WAITING_PLAYER) {
            return;
        }

        player_input_.push_back(msg->block_id);
        player_index_++;

        ROS_INFO("Player selected block %d (%s)", msg->block_id, msg->color.c_str());

        resetPlayerTimeout();

        if (player_index_ >= static_cast<int>(sequence_.size())) {
            current_state_ = GameState::CHECKING_INPUT;
            publishState("CHECKING_INPUT");
            checkSequence();
        }
    }

    void startTimerCallback(const ros::TimerEvent&) {
        startGame();
    }

    void showFailsafeCallback(const ros::TimerEvent&) {
        if (current_state_ == GameState::SHOWING_SEQUENCE && waiting_for_motion_) {
            if (!motion_started_for_sequence_) {
                ROS_ERROR("Motion never started before sequence timeout");
                abortDueToMotionFailure();
                return;
            }
            ROS_ERROR("Motion sequence timed out before completion");
            abortDueToMotionFailure();
        }
    }

    void roundPauseCallback(const ros::TimerEvent&) {
        generateAndStartSequence();
    }

    void targetSubWaitCallback(const ros::TimerEvent&) {
        sendSequenceToMotion();
    }

    void motionRetryCallback(const ros::TimerEvent&) {
        sendSequenceToMotion();
    }

    void abortDueToMotionFailure() {
        waiting_for_motion_ = false;
        motion_started_for_sequence_ = false;
        sequence_targets_completed_ = 0;
        sequence_sent_to_motion_ = false;
        show_failsafe_timer_.stop();
        target_sub_wait_timer_.stop();
        motion_retry_timer_.stop();
        player_timeout_timer_.stop();
        round_timer_.stop();
        current_state_ = GameState::MOTION_FAILED;
        ROS_ERROR("Motion failed while showing the sequence. Aborting round.");
        publishState("MOTION_FAILED");
    }

    void scheduleMotionRetryOrAbort(const std::string& reason) {
        show_failsafe_timer_.stop();

        if (motion_retry_count_ >= motion_retry_limit_ || !active_sequence_msg_valid_) {
            ROS_ERROR("%s. Retry budget exhausted or no frozen sequence is available.", reason.c_str());
            abortDueToMotionFailure();
            return;
        }

        ++motion_retry_count_;
        waiting_for_motion_ = false;
        motion_started_for_sequence_ = false;
        sequence_targets_completed_ = 0;
        sequence_sent_to_motion_ = false;

        ROS_WARN("%s. Retrying frozen sequence %u (%d/%d).",
                 reason.c_str(),
                 active_sequence_id_,
                 motion_retry_count_,
                 motion_retry_limit_);
        publishState("WAITING_FOR_MOTION");
        motion_retry_timer_ = nh_.createTimer(
            ros::Duration(motion_retry_delay_sec_),
            &GameNode::motionRetryCallback,
            this,
            true);
    }

    void waitForBlocksAndRetry(const std::string& reason) {
        waiting_for_motion_ = false;
        motion_started_for_sequence_ = false;
        sequence_targets_completed_ = 0;
        sequence_sent_to_motion_ = false;
        publishState("WAITING_FOR_BLOCKS");
        ROS_WARN_THROTTLE(2.0, "%s Waiting for current /detected_blocks (%d/%zu cached)",
                          reason.c_str(), freshBlockCount(ros::Time::now()), available_block_ids_.size());
        target_sub_wait_timer_ = nh_.createTimer(
            ros::Duration(blocks_wait_sec_),
            &GameNode::targetSubWaitCallback,
            this,
            true);
    }

    bool haveSequenceTargets() const {
        const ros::Time now = ros::Time::now();
        for (const int block_id : sequence_) {
            if (!isBlockFresh(block_id, now)) {
                return false;
            }
        }
        return true;
    }

    bool motionReadyForSequence() const {
        if (motion_sub_.getNumPublishers() == 0 || !have_motion_state_) {
            return false;
        }

        const MotionStatus status = parseMotionStatus(last_motion_state_);
        return status.state == "IDLE";
    }

    void playerTimeoutCallback(const ros::TimerEvent&) {
        if (current_state_ != GameState::WAITING_PLAYER) {
            return;
        }

        ROS_WARN("Player timeout. Game over.");
        gameOver();
    }

    void startGame() {
        if (require_detected_blocks_ && !refreshAvailableBlocksFromDetections()) {
            publishState("WAITING_FOR_BLOCKS");
            ROS_WARN_THROTTLE(2.0, "Waiting for /detected_blocks (%d/%d usable blocks cached)",
                              freshBlockCount(ros::Time::now()), min_detected_blocks_required_);
            start_timer_ = nh_.createTimer(
                ros::Duration(blocks_wait_sec_),
                &GameNode::startTimerCallback,
                this,
                true);
            return;
        }

        ROS_INFO("Starting new game");
        score_ = 0;
        level_ = 1;
        publishScore();
        generateAndStartSequence();
    }

    void generateAndStartSequence() {
        sequence_.clear();
        player_input_.clear();
        player_index_ = 0;
        waiting_for_motion_ = false;
        motion_started_for_sequence_ = false;
        sequence_targets_completed_ = 0;
        sequence_sent_to_motion_ = false;
        target_sub_wait_start_ = ros::Time(0);
        active_sequence_msg_valid_ = false;
        motion_retry_count_ = 0;
        show_failsafe_timer_.stop();
        target_sub_wait_timer_.stop();
        motion_retry_timer_.stop();
        active_sequence_id_ = next_sequence_id_++;

        const int sequence_length = base_length_ + length_per_level_ * (level_ - 1);
        std::uniform_int_distribution<int> dis(0, static_cast<int>(available_block_ids_.size()) - 1);

        int prev = -1;
        for (int i = 0; i < sequence_length; i++) {
            int value = available_block_ids_[dis(gen_)];
            if (no_immediate_repeat_ && available_block_ids_.size() > 1) {
                while (value == prev) {
                    value = available_block_ids_[dis(gen_)];
                }
            }
            sequence_.push_back(value);
            prev = value;
        }

        current_state_ = GameState::SHOWING_SEQUENCE;
        publishState("SHOWING_SEQUENCE");

        ROS_INFO("Generated sequence %u of length %d", active_sequence_id_, sequence_length);
        for (size_t i = 0; i < sequence_.size(); i++) {
            ROS_INFO("  [%zu] Block %d", i, sequence_[i]);
        }

        sendSequenceToMotion();
    }

    void sendSequenceToMotion() {
        if (sequence_sent_to_motion_) {
            return;
        }

        if (!active_sequence_msg_valid_ && require_detected_blocks_ && !haveSequenceTargets()) {
            refreshAvailableBlocksFromDetections();
            waitForBlocksAndRetry("Sequence target missing.");
            return;
        }

        if (!motionReadyForSequence()) {
            const ros::Time now = ros::Time::now();
            if (target_sub_wait_start_.isZero()) {
                target_sub_wait_start_ = now;
            }

            const double waited_sec = (now - target_sub_wait_start_).toSec();
            if (waited_sec > target_subscriber_timeout_sec_) {
                ROS_ERROR("Motion node did not report ready after %.1fs", waited_sec);
                abortDueToMotionFailure();
                return;
            }

            const std::string last_status = have_motion_state_ ? last_motion_state_ : "none";
            ROS_WARN_THROTTLE(2.0,
                              "Waiting for motion node to report IDLE (%u publishers, last status: %s, %.1fs/%.1fs)",
                              motion_sub_.getNumPublishers(),
                              last_status.c_str(),
                              waited_sec,
                              target_subscriber_timeout_sec_);
            publishState("WAITING_FOR_MOTION");
            target_sub_wait_timer_ = nh_.createTimer(
                ros::Duration(target_subscriber_poll_sec_),
                &GameNode::targetSubWaitCallback,
                this,
                true);
            return;
        }

        if (target_sequence_pub_.getNumSubscribers() == 0) {
            const ros::Time now = ros::Time::now();
            if (target_sub_wait_start_.isZero()) {
                target_sub_wait_start_ = now;
            }

            const double waited_sec = (now - target_sub_wait_start_).toSec();
            if (waited_sec > target_subscriber_timeout_sec_) {
                ROS_ERROR("No /target_sequence subscribers after %.1fs", waited_sec);
                abortDueToMotionFailure();
                return;
            }

            ROS_WARN_THROTTLE(2.0, "Waiting for /target_sequence subscriber (%.1fs/%.1fs)",
                              waited_sec, target_subscriber_timeout_sec_);
            publishState("WAITING_FOR_MOTION");
            target_sub_wait_timer_ = nh_.createTimer(
                ros::Duration(target_subscriber_poll_sec_),
                &GameNode::targetSubWaitCallback,
                this,
                true);
            return;
        }

        if (!active_sequence_msg_valid_) {
            active_sequence_msg_ = memory_game::BlockSequence();
            active_sequence_msg_.header.frame_id = target_frame_;
            active_sequence_msg_.sequence_id = active_sequence_id_;
            active_sequence_msg_.blocks.reserve(sequence_.size());

            for (const int block_id : sequence_) {
                memory_game::Block target;
                if (!tryMakeTargetBlock(block_id, target)) {
                    waitForBlocksAndRetry("Sequence target missing while building sequence.");
                    return;
                }
                active_sequence_msg_.blocks.push_back(target);
            }

            active_sequence_msg_valid_ = true;
        }

        active_sequence_msg_.header.stamp = ros::Time::now();
        waiting_for_motion_ = true;
        motion_started_for_sequence_ = false;
        sequence_sent_to_motion_ = true;

        target_sequence_pub_.publish(active_sequence_msg_);
        ROS_INFO("Dispatched sequence %u with %zu frozen targets",
                 active_sequence_id_,
                 active_sequence_msg_.blocks.size());

        if (!have_motion_state_) {
            ROS_WARN("No /motion_status received yet. Is motion node running?");
        }

        const double per_target_budget_sec = motion_timeout_per_target_sec_;
        const double timeout_sec =
            std::max(5.0, per_target_budget_sec * static_cast<double>(sequence_.size()) + motion_timeout_padding_sec_);
        show_failsafe_timer_.stop();
        show_failsafe_timer_ = nh_.createTimer(
            ros::Duration(timeout_sec),
            &GameNode::showFailsafeCallback,
            this,
            true);
    }

    void checkSequence() {
        bool correct = true;

        if (player_input_.size() != sequence_.size()) {
            correct = false;
        }

        for (size_t i = 0; correct && i < sequence_.size(); i++) {
            if (sequence_[i] != player_input_[i]) {
                correct = false;
            }
        }

        if (correct) {
            score_ += 10 * level_;
            level_++;
            publishScore();

            ROS_INFO("Correct! Score: %d, Level: %d", score_, level_);

            current_state_ = GameState::IDLE;
            publishState("ROUND_PAUSE");

            round_timer_ = nh_.createTimer(
                ros::Duration(round_pause_sec_),
                &GameNode::roundPauseCallback,
                this,
                true);
        } else {
            ROS_INFO("Incorrect sequence. Game over!");
            ROS_INFO("Expected:");
            for (size_t i = 0; i < sequence_.size(); i++) {
                ROS_INFO("  [%zu] Block %d", i, sequence_[i]);
            }
            ROS_INFO("Got:");
            for (size_t i = 0; i < player_input_.size(); i++) {
                ROS_INFO("  [%zu] Block %d", i, player_input_[i]);
            }

            gameOver();
        }
    }

    void gameOver() {
        current_state_ = GameState::GAME_OVER;
        publishState("GAME_OVER");

        player_timeout_timer_.stop();
        show_failsafe_timer_.stop();
        round_timer_.stop();
        target_sub_wait_timer_.stop();
        motion_retry_timer_.stop();
    }

    void resetPlayerTimeout() {
        player_timeout_timer_.stop();
        if (player_timeout_sec_ <= 0.0) {
            ROS_INFO_THROTTLE(2.0, "Player timeout disabled (player_timeout_sec <= 0)");
            return;
        }
        player_timeout_timer_ = nh_.createTimer(
            ros::Duration(player_timeout_sec_),
            &GameNode::playerTimeoutCallback,
            this,
            true);
    }

    bool tryMakeTargetBlock(int block_id, memory_game::Block& target) const {
        const ros::Time now = ros::Time::now();
        auto it = known_blocks_.find(block_id);
        if (it != known_blocks_.end() && isBlockFresh(block_id, now)) {
            target = it->second;
            target.header.stamp = now;
            target.id = block_id;
            return true;
        }

        if (require_detected_blocks_) {
            return false;
        }

        target = memory_game::Block();
        target.header.stamp = ros::Time::now();
        target.header.frame_id = target_frame_;
        target.id = block_id;
        target.color = "unknown";
        target.position = default_target_;
        target.orientation.w = 1.0;
        target.confidence = 0.0;
        target.is_selected = false;
        ROS_WARN_THROTTLE(2.0, "Using fallback target for block %d because detections are not required", block_id);
        return true;
    }

    void publishState(const std::string& state) {
        std_msgs::String msg;
        msg.data = state;
        state_pub_.publish(msg);
        ROS_INFO("Game state: %s", state.c_str());
    }

    void publishScore() {
        std_msgs::Int32 msg;
        msg.data = score_;
        score_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "game_node");
    GameNode game_node;
    ros::spin();
    return 0;
}
