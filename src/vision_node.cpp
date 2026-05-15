/**
 * @file vision_node.cpp
 * @brief Real camera-based vision node for colored block detection.
 *
 * This node detects colored blocks from RGB + depth streams and publishes:
 *   - /detected_blocks (memory_game/BlockArray)
 *   - debug image topics for mask/overlay inspection
 * Player selection is handled separately by scripts/player_selection.py.
 *
 * Core pipeline:
 *   1) RGB BGR -> HSV conversion
 *   2) Per-color segmentation with configurable HSV ranges
 *   3) Contour filtering by area and centroid extraction
 *   4) Robust depth (median over a local pixel window)
 *   5) Pixel+depth -> 3D camera point via camera intrinsics
 *   6) Camera frame -> robot base frame transform via TF
 *   7) Optional EMA smoothing for stable positions
 */

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <memory_game/Block.h>
#include <memory_game/BlockArray.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace {

struct HsvRange {
    int h_min;
    int h_max;
    int s_min;
    int s_max;
    int v_min;
    int v_max;
};

struct ColorConfig {
    int id;
    std::string name;
    std::vector<HsvRange> ranges;
};

struct DetectionResult {
    cv::Mat mask;
    std::vector<cv::Point> centroids;
    std::vector<double> areas;
};

struct ExclusionZone {
    double x;
    double y;
    double z;
    double radius;
};

}  // namespace


class VisionNode {
public:
VisionNode()
    : nh_(),
      pnh_("~"),
      it_(nh_),
      tf_buffer_(),
      tf_listener_(tf_buffer_),
      has_depth_(false),
      has_camera_model_(false) {
    loadParams();

    blocks_pub_ = nh_.advertise<memory_game::BlockArray>("/detected_blocks", 10);
    if (enable_debug_images_) {
        debug_mask_pub_ = it_.advertise("/vision/debug_mask", 1);
        debug_overlay_pub_ = it_.advertise("/vision/debug_overlay", 1);
    }

    color_sub_ = nh_.subscribe(color_topic_, 5, &VisionNode::colorCallback, this);
    depth_sub_ = nh_.subscribe(depth_topic_, 10, &VisionNode::depthCallback, this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic_, 5, &VisionNode::cameraInfoCallback, this);

    ROS_INFO("vision_node ready (block detection only)");
    ROS_INFO("color_topic=%s", color_topic_.c_str());
    ROS_INFO("depth_topic=%s", depth_topic_.c_str());
    ROS_INFO("camera_info_topic=%s", camera_info_topic_.c_str());
    ROS_INFO("target_frame=%s", target_frame_.c_str());
}

private:
void loadParams() {
    pnh_.param("color_topic", color_topic_, std::string("/realsense/color/image_raw"));
    pnh_.param("depth_topic", depth_topic_, std::string("/realsense/aligned_depth_to_color/image_raw"));
    pnh_.param("camera_info_topic", camera_info_topic_, std::string("/realsense/color/camera_info"));
    pnh_.param("target_frame", target_frame_, std::string("panda_link0"));
    pnh_.param("disable_red", disable_red_, true);

    pnh_.param("blur_kernel_size", blur_kernel_size_, 3);
    if (blur_kernel_size_ > 0 && blur_kernel_size_ % 2 == 0) {
        blur_kernel_size_++;  // OpenCV requires odd kernel size
    }

    pnh_.param("min_block_area", min_block_area_, 500.0);
    pnh_.param("max_block_area", max_block_area_, 60000.0);
    pnh_.param("mask_open_iterations", mask_open_iterations_, 2);
    pnh_.param("mask_close_iterations", mask_close_iterations_, 2);

    pnh_.param("depth_window_radius", depth_window_radius_, 2);
    pnh_.param("min_depth_m", min_depth_m_, 0.10);
    pnh_.param("max_depth_m", max_depth_m_, 2.50);
    pnh_.param("max_depth_age_sec", max_depth_age_sec_, 1.0);
    pnh_.param("depth_buffer_size", depth_buffer_size_, 10);

    pnh_.param("enable_debug_images", enable_debug_images_, true);
    pnh_.param("smoothing_alpha", smoothing_alpha_, 0.35);
    pnh_.param("image_track_timeout_sec", image_track_timeout_sec_, 0.5);
    pnh_.param("max_candidate_jump_px", max_candidate_jump_px_, 120.0);
    pnh_.param("position_reset_timeout_sec", position_reset_timeout_sec_, 0.5);
    pnh_.param("max_position_jump_m", max_position_jump_m_, 0.10);
    pnh_.param("base_exclusion_radius_m", base_exclusion_radius_m_, 0.15);
    pnh_.param("workspace_enable", workspace_enable_, false);
    pnh_.param("workspace_min_x", workspace_min_x_, -10.0);
    pnh_.param("workspace_max_x", workspace_max_x_, 10.0);
    pnh_.param("workspace_min_y", workspace_min_y_, -10.0);
    pnh_.param("workspace_max_y", workspace_max_y_, 10.0);
    pnh_.param("workspace_min_z", workspace_min_z_, -10.0);
    pnh_.param("workspace_max_z", workspace_max_z_, 10.0);

    if (mask_open_iterations_ < 0) mask_open_iterations_ = 0;
    if (mask_close_iterations_ < 0) mask_close_iterations_ = 0;
    if (depth_window_radius_ < 0) depth_window_radius_ = 0;
    depth_buffer_size_ = std::max(2, depth_buffer_size_);
    smoothing_alpha_ = std::max(0.0, std::min(1.0, smoothing_alpha_));
    image_track_timeout_sec_ = std::max(0.0, image_track_timeout_sec_);
    max_candidate_jump_px_ = std::max(1.0, max_candidate_jump_px_);
    position_reset_timeout_sec_ = std::max(0.0, position_reset_timeout_sec_);
    max_position_jump_m_ = std::max(0.0, max_position_jump_m_);

    initDefaultColors();
    applyColorFilters();
    loadHsvRangesFromParams();
    loadExtraExclusionsFromParams();
}

void initDefaultColors() {
    color_configs_.clear();

    color_configs_.push_back(ColorConfig{
        0,
        "red",
        {
            {0, 15, 90, 255, 50, 255},    // raised S_min 60->90 to reject low-sat table surfaces
            {165, 180, 90, 255, 50, 255}, // same for upper red wrap-around
        },
    });

    color_configs_.push_back(ColorConfig{1, "green", {{40, 85, 70, 255, 50, 255}}});
    color_configs_.push_back(ColorConfig{2, "blue", {{95, 135, 70, 255, 50, 255}}});
    color_configs_.push_back(ColorConfig{3, "yellow", {{18, 40, 100, 255, 80, 255}}}); // widened H 20-35 -> 18-40
}

void applyColorFilters() {
    if (!disable_red_) {
        return;
    }

    color_configs_.erase(
        std::remove_if(color_configs_.begin(),
                       color_configs_.end(),
                       [](const ColorConfig& cfg) { return cfg.id == 0 || cfg.name == "red"; }),
        color_configs_.end());
}

void loadHsvRangesFromParams() {
    XmlRpc::XmlRpcValue hsv_cfg;
    if (!pnh_.getParam("hsv_ranges", hsv_cfg)) {
        ROS_WARN("Using built-in HSV defaults (param '~hsv_ranges' not found)");
        return;
    }

    if (hsv_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_WARN("Ignoring '~hsv_ranges': expected dictionary");
        return;
    }

    for (ColorConfig& cfg : color_configs_) {
        if (!hsv_cfg.hasMember(cfg.name)) {
            continue;
        }

        XmlRpc::XmlRpcValue ranges = hsv_cfg[cfg.name];
        if (ranges.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("Ignoring hsv_ranges.%s: expected list", cfg.name.c_str());
            continue;
        }

        std::vector<HsvRange> parsed;
        for (int i = 0; i < ranges.size(); ++i) {
            if (ranges[i].getType() != XmlRpc::XmlRpcValue::TypeArray || ranges[i].size() != 6) {
                ROS_WARN("Ignoring hsv_ranges.%s[%d]: expected [hmin,hmax,smin,smax,vmin,vmax]",
                         cfg.name.c_str(), i);
                continue;
            }

            bool ok = true;
            int vals[6] = {0};
            for (int j = 0; j < 6; ++j) {
                if (ranges[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                    ok = false;
                    break;
                }
                vals[j] = static_cast<int>(ranges[i][j]);
            }
            if (!ok) {
                ROS_WARN("Ignoring hsv_ranges.%s[%d]: all values must be integers", cfg.name.c_str(), i);
                continue;
            }

            parsed.push_back(HsvRange{vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]});
        }

        if (!parsed.empty()) {
            cfg.ranges = parsed;
        }
    }
}

void loadExtraExclusionsFromParams() {
    extra_exclusions_.clear();
    XmlRpc::XmlRpcValue zones;
    if (!pnh_.getParam("extra_exclusions", zones)) return;
    for (int i = 0; i < zones.size(); ++i) {
        XmlRpc::XmlRpcValue& z = zones[i];
        extra_exclusions_.push_back({
            static_cast<double>(z["x"]),
            static_cast<double>(z["y"]),
            static_cast<double>(z["z"]),
            static_cast<double>(z["radius"])});
    }
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_model_.fromCameraInfo(msg);
    has_camera_model_ = true;
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    DepthFrame frame;
    frame.stamp = msg->header.stamp;

    try {
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || msg->encoding == "16UC1") {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            frame.image = cv_ptr->image.clone();
            frame.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 || msg->encoding == "32FC1") {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            frame.image = cv_ptr->image.clone();
            frame.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        } else {
            ROS_WARN_THROTTLE(2.0, "Unsupported depth encoding: %s", msg->encoding.c_str());
            return;
        }
    } catch (const cv_bridge::Exception& e) {
        ROS_WARN_THROTTLE(2.0, "Depth cv_bridge error: %s", e.what());
        return;
    }

    depth_buffer_.push_back(frame);
    while (static_cast<int>(depth_buffer_.size()) > depth_buffer_size_) {
        depth_buffer_.pop_front();
    }
    has_depth_ = !depth_buffer_.empty();
}

void colorCallback(const sensor_msgs::ImageConstPtr& color_msg) {
    // Decode first so we can always publish a debug image regardless of what fails below.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        ROS_WARN_THROTTLE(2.0, "Color cv_bridge error: %s", e.what());
        return;
    }

    if (!has_camera_model_) {
        ROS_WARN_THROTTLE(2.0, "Waiting for camera_info (%s)", camera_info_topic_.c_str());
        if (enable_debug_images_ && debug_overlay_pub_.getNumSubscribers() > 0) {
            cv::Mat overlay = cv_ptr->image.clone();
            cv::putText(overlay, "NO CAMERA INFO", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            cv_bridge::CvImage out;
            out.header = color_msg->header;
            out.encoding = sensor_msgs::image_encodings::BGR8;
            out.image = overlay;
            debug_overlay_pub_.publish(out.toImageMsg());
        }
        return;
    }

    // Normalise zero stamps to wall-clock now so depth sync math doesn't blow up.
    const ros::Time color_stamp =
        color_msg->header.stamp.isZero() ? ros::Time::now() : color_msg->header.stamp;

    if (!selectClosestDepthFrame(color_stamp)) {
        // Log the best available delta so the user can tune max_depth_age_sec or
        // diagnose timestamp mismatches between the depth and RGB streams.
        double best_delta = std::numeric_limits<double>::infinity();
        for (const DepthFrame& f : depth_buffer_) {
            const ros::Time ds = f.stamp.isZero() ? ros::Time::now() : f.stamp;
            const double d = std::fabs((color_stamp - ds).toSec());
            if (d < best_delta) best_delta = d;
        }
        if (depth_buffer_.empty()) {
            ROS_WARN_THROTTLE(2.0, "No depth frame received yet — check that '%s' is publishing",
                              depth_topic_.c_str());
        } else {
            ROS_WARN_THROTTLE(2.0,
                              "Depth/RGB sync failed: best delta=%.3fs > max_depth_age_sec=%.3f. "
                              "Increase max_depth_age_sec or check camera timestamp alignment.",
                              best_delta, max_depth_age_sec_);
        }

        // Still publish a debug overlay so rqt_image_view shows what the camera sees.
        if (enable_debug_images_ && debug_overlay_pub_.getNumSubscribers() > 0) {
            cv::Mat overlay = cv_ptr->image.clone();
            cv::putText(overlay, "NO DEPTH SYNC", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            cv_bridge::CvImage out;
            out.header = color_msg->header;
            out.encoding = sensor_msgs::image_encodings::BGR8;
            out.image = overlay;
            debug_overlay_pub_.publish(out.toImageMsg());
        }
        return;
    }

    // A small blur before HSV conversion reduces pixel-level noise from the camera sensor.
    // Without it, edge pixels on block boundaries can have wildly wrong hues and speckle the masks.
    cv::Mat bgr_smoothed;
    if (blur_kernel_size_ > 1) {
        cv::GaussianBlur(cv_ptr->image, bgr_smoothed,
                         cv::Size(blur_kernel_size_, blur_kernel_size_), 0);
    } else {
        bgr_smoothed = cv_ptr->image;
    }
    cv::Mat hsv;
    cv::cvtColor(bgr_smoothed, hsv, cv::COLOR_BGR2HSV);

    std::vector<memory_game::Block> blocks;
    blocks.reserve(color_configs_.size());

    const ros::Time frame_stamp = color_stamp;
    pruneTrackingState(frame_stamp);

    const bool publish_debug_images =
        enable_debug_images_ &&
        ((debug_mask_pub_.getNumSubscribers() > 0) ||
         (debug_overlay_pub_.getNumSubscribers() > 0));

    cv::Mat debug_mask_accum;
    cv::Mat debug_overlay;
    if (publish_debug_images) {
        debug_mask_accum = cv::Mat::zeros(hsv.size(), CV_8UC1);
        debug_overlay = cv_ptr->image.clone();
    }

    for (const ColorConfig& cfg : color_configs_) {
        // Per-color 2D detection in image space.
        const DetectionResult det = detectColorBlob(hsv, cfg);
        if (publish_debug_images && !det.mask.empty()) {
            cv::bitwise_or(debug_mask_accum, det.mask, debug_mask_accum);
        }
        if (det.centroids.empty()) {
            continue;
        }

        bool dropped_far_candidates = false;
        const std::vector<int> ranked_indices =
            rankCandidateIndices(cfg.id, det, frame_stamp, &dropped_far_candidates);
        if (ranked_indices.empty()) {
            if (dropped_far_candidates) {
                ROS_WARN_THROTTLE(2.0,
                                  "Dropped %s candidates: no contour stayed near the recent track",
                                  cfg.name.c_str());
            }
            continue;
        }

        geometry_msgs::Point p_base;
        cv::Point chosen_centroid;
        double chosen_area = 0.0;
        bool accepted_candidate = false;

        // Try multiple valid contours for this color. If the largest blob has bad depth,
        // we still want to fall back to the next candidate.
        for (const int idx : ranked_indices) {
            const cv::Point candidate = det.centroids[static_cast<size_t>(idx)];

            double depth_m = 0.0;
            if (!readDepthMedianMeters(candidate.x, candidate.y, depth_m)) {
                continue;
            }

            geometry_msgs::Point candidate_base;
            if (!projectAndTransformToBase(candidate.x, candidate.y, depth_m, color_msg->header, candidate_base)) {
                continue;
            }

            // Reject non-finite positions — a bad TF or depth value can produce NaN/inf
            // which would send the robot to an undefined location.
            if (!std::isfinite(candidate_base.x) ||
                !std::isfinite(candidate_base.y) ||
                !std::isfinite(candidate_base.z)) {
                ROS_WARN_THROTTLE(2.0, "Rejected %s block: non-finite 3D position (%.3f, %.3f, %.3f)",
                                  cfg.name.c_str(),
                                  candidate_base.x, candidate_base.y, candidate_base.z);
                continue;
            }

            double position_jump_m = 0.0;
            if (violatesRecentPositionTrack(cfg.id, candidate_base, frame_stamp, position_jump_m)) {
                ROS_WARN_THROTTLE(2.0,
                                  "Rejected %s block at (%.3f, %.3f, %.3f): jumped %.3fm from recent track",
                                  cfg.name.c_str(),
                                  candidate_base.x, candidate_base.y, candidate_base.z,
                                  position_jump_m);
                continue;
            }

            if (workspace_enable_ && !withinWorkspace(candidate_base)) {
                ROS_WARN_THROTTLE(2.0,
                                  "Rejected %s block at (%.3f, %.3f, %.3f): outside workspace",
                                  cfg.name.c_str(),
                                  candidate_base.x, candidate_base.y, candidate_base.z);
                continue;
            }

            // Reject detections too close to the robot base — the Panda status LED
            // can be mistaken for a blue block.
            if (base_exclusion_radius_m_ > 0.0) {
                const double dist = std::sqrt(
                    candidate_base.x * candidate_base.x +
                    candidate_base.y * candidate_base.y +
                    candidate_base.z * candidate_base.z);
                if (dist < base_exclusion_radius_m_) {
                    ROS_WARN_THROTTLE(2.0, "Rejected %s block at (%.3f, %.3f, %.3f): within base exclusion zone (%.2fm)",
                                      cfg.name.c_str(),
                                      candidate_base.x, candidate_base.y, candidate_base.z,
                                      base_exclusion_radius_m_);
                    continue;
                }
            }

            bool in_extra = false;
            for (const ExclusionZone& zone : extra_exclusions_) {
                const double dx = candidate_base.x - zone.x;
                const double dy = candidate_base.y - zone.y;
                const double dz = candidate_base.z - zone.z;
                if (dx * dx + dy * dy + dz * dz < zone.radius * zone.radius) {
                    ROS_WARN_THROTTLE(2.0, "Rejected %s block: inside extra exclusion zone", cfg.name.c_str());
                    in_extra = true;
                    break;
                }
            }
            if (in_extra) continue;

            // If we already have a fresh tracked position for this color, reject
            // candidates that jumped too far from it — treat them as false positives
            // and let the next-best candidate be tried. If none survive, no detection
            // is published this frame and the previous position naturally expires
            // after position_reset_timeout_sec, allowing a true re-lock afterwards.
            auto fp_it = filtered_positions_.find(cfg.id);
            if (fp_it != filtered_positions_.end() && max_position_jump_m_ > 0.0) {
                const double dx = candidate_base.x - fp_it->second.x;
                const double dy = candidate_base.y - fp_it->second.y;
                const double dz = candidate_base.z - fp_it->second.z;
                const double jump = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (jump > max_position_jump_m_) {
                    ROS_WARN_THROTTLE(2.0,
                                      "Rejected %s candidate: %.3fm jump from previous tracked position",
                                      cfg.name.c_str(), jump);
                    continue;
                }
            }

            p_base = applyEmaSmoothing(cfg.id, candidate_base, frame_stamp);
            chosen_centroid = candidate;
            chosen_area = det.areas[static_cast<size_t>(idx)];
            accepted_candidate = true;
            rememberImageCentroid(cfg.id, candidate, frame_stamp);
            break;
        }

        if (!accepted_candidate) {
            continue;
        }

        memory_game::Block block;
        block.header.stamp = color_msg->header.stamp;
        block.header.frame_id = target_frame_;
        block.id = cfg.id;
        block.color = cfg.name;
        block.position = p_base;
        block.orientation.w = 1.0;
        block.confidence = static_cast<float>(
            std::min(1.0, chosen_area / std::max(1.0, min_block_area_ * 8.0)));
        block.is_selected = false;

        blocks.push_back(block);

        if (publish_debug_images) {
            cv::circle(debug_overlay, chosen_centroid, 6, cv::Scalar(255, 255, 255), 2);
            cv::putText(debug_overlay,
                        cfg.name,
                        cv::Point(chosen_centroid.x + 8, chosen_centroid.y - 8),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(255, 255, 255),
                        1,
                        cv::LINE_AA);
        }
    }

    std::sort(blocks.begin(), blocks.end(), [](const memory_game::Block& a, const memory_game::Block& b) {
        return a.id < b.id;
    });

    publishBlocks(blocks, color_msg->header.stamp);

    if (publish_debug_images) {
        publishDebugImages(debug_mask_accum, debug_overlay, color_msg->header);
    }

    ROS_DEBUG_THROTTLE(1.0, "Published %zu detected blocks", blocks.size());
}

bool withinWorkspace(const geometry_msgs::Point& point) const {
    return point.x >= workspace_min_x_ && point.x <= workspace_max_x_ &&
           point.y >= workspace_min_y_ && point.y <= workspace_max_y_ &&
           point.z >= workspace_min_z_ && point.z <= workspace_max_z_;
}


DetectionResult detectColorBlob(const cv::Mat& hsv,
                                const ColorConfig& cfg) const {
    DetectionResult out;
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);

    for (const HsvRange& r : cfg.ranges) {
        cv::Mat partial;
        cv::inRange(hsv,
                    cv::Scalar(r.h_min, r.s_min, r.v_min),
                    cv::Scalar(r.h_max, r.s_max, r.v_max),
                    partial);
        cv::bitwise_or(mask, partial, mask);
    }
    // CLOSE first: fills glare spots, shadows, and occlusion holes inside the block.
    //    Doing open first would erode a block with a hole into two fragments, then fail area filter.
    if (mask_close_iterations_ > 0) {
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), mask_close_iterations_);
    }
    // OPEN second: removes small isolated noise blobs that survived after closing.
    if (mask_open_iterations_ > 0) {
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), mask_open_iterations_);
    }
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int best_idx = -1;
    double best_area = 0.0;
    for (size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area < min_block_area_ || area > max_block_area_) {
            continue;
        }

        const cv::Moments m = cv::moments(contours[i]);
        if (m.m00 <= 1e-6) {
            continue;
        }

        out.centroids.push_back(cv::Point(static_cast<int>(m.m10 / m.m00),
                                          static_cast<int>(m.m01 / m.m00)));
        out.areas.push_back(area);
        if (area > best_area) {
            best_area = area;
            best_idx = static_cast<int>(out.areas.size()) - 1;
        }
    }

    out.mask = mask;
    if (best_idx > 0) {
        std::swap(out.centroids[0], out.centroids[static_cast<size_t>(best_idx)]);
        std::swap(out.areas[0], out.areas[static_cast<size_t>(best_idx)]);
    }
    return out;
}

bool selectClosestDepthFrame(const ros::Time& color_stamp) {
    if (depth_buffer_.empty()) {
        has_depth_ = false;
        return false;
    }

    double best_delta = std::numeric_limits<double>::infinity();
    const DepthFrame* best_frame = nullptr;
    const DepthFrame* newest_frame = nullptr;
    double newest_time = -1.0;

    for (const DepthFrame& frame : depth_buffer_) {
        const ros::Time ds = frame.stamp.isZero() ? ros::Time::now() : frame.stamp;
        const double delta = std::fabs((color_stamp - ds).toSec());
        if (delta < best_delta) {
            best_delta = delta;
            best_frame = &frame;
        }
        // Track newest frame as fallback.
        const double t = ds.toSec();
        if (t > newest_time) {
            newest_time = t;
            newest_frame = &frame;
        }
    }

    // Use best match if within tolerance; otherwise fall back to the newest
    // depth frame so the pipeline keeps running despite timestamp drift.
    const DepthFrame* chosen = nullptr;
    if (best_delta <= max_depth_age_sec_) {
        chosen = best_frame;
    } else {
        ROS_WARN_THROTTLE(2.0,
                          "Depth/RGB timestamp drift: best delta=%.3fs (max=%.3fs). "
                          "Using newest depth frame as fallback.",
                          best_delta, max_depth_age_sec_);
        chosen = newest_frame;
    }

    if (chosen == nullptr) {
        has_depth_ = false;
        return false;
    }

    latest_depth_ = chosen->image;
    latest_depth_encoding_ = chosen->encoding;
    latest_depth_stamp_ = chosen->stamp;
    has_depth_ = true;
    return true;
}

bool readDepthMedianMeters(int u, int v, double& depth_m) const {
    if (latest_depth_.empty()) {
        return false;
    }

    std::vector<double> depth_values;
    const int window = depth_window_radius_;
    depth_values.reserve(static_cast<size_t>((2 * window + 1) * (2 * window + 1)));

    for (int dv = -window; dv <= window; ++dv) {
        for (int du = -window; du <= window; ++du) {
            const int uu = u + du;
            const int vv = v + dv;
            if (uu < 0 || vv < 0 || uu >= latest_depth_.cols || vv >= latest_depth_.rows) {
                continue;
            }

            double z = 0.0;
            if (latest_depth_encoding_ == sensor_msgs::image_encodings::TYPE_16UC1) {
                const uint16_t raw = latest_depth_.at<uint16_t>(vv, uu);
                if (raw == 0) {
                    continue;
                }
                z = static_cast<double>(raw) * 0.001;
            } else if (latest_depth_encoding_ == sensor_msgs::image_encodings::TYPE_32FC1) {
                const float raw = latest_depth_.at<float>(vv, uu);
                if (!std::isfinite(raw) || raw <= 0.0f) {
                    continue;
                }
                z = static_cast<double>(raw);
            } else {
                return false;
            }

            if (z >= min_depth_m_ && z <= max_depth_m_) {
                depth_values.push_back(z);
            }
        }
    }

    if (depth_values.empty()) {
        return false;
    }

    // Median depth is more robust than a single-pixel sample.
    std::nth_element(depth_values.begin(),
                     depth_values.begin() + depth_values.size() / 2,
                     depth_values.end());
    depth_m = depth_values[depth_values.size() / 2];
    return true;
}

bool projectAndTransformToBase(int u,
                               int v,
                               double depth_m,
                               const std_msgs::Header& image_header,
                               geometry_msgs::Point& out_base_point) {
    // Intrinsics projection: pixel -> normalized ray -> metric camera point.
    const cv::Point2d pixel(static_cast<double>(u), static_cast<double>(v));
    const cv::Point3d ray = camera_model_.projectPixelTo3dRay(pixel);

    geometry_msgs::PointStamped p_cam;
    p_cam.header = image_header;
    p_cam.point.x = ray.x * depth_m;
    p_cam.point.y = ray.y * depth_m;
    p_cam.point.z = ray.z * depth_m;

    geometry_msgs::PointStamped p_base;
    try {
        p_base = tf_buffer_.transform(p_cam, target_frame_, ros::Duration(0.03));
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "TF transform failed (%s -> %s): %s",
                          p_cam.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
        return false;
    }

    out_base_point = p_base.point;
    return true;
}

std::vector<int> rankCandidateIndices(int block_id,
                                      const DetectionResult& det,
                                      const ros::Time& stamp,
                                      bool* dropped_far_candidates) const {
    if (dropped_far_candidates) {
        *dropped_far_candidates = false;
    }

    std::vector<int> indices(det.centroids.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        indices[i] = static_cast<int>(i);
    }

    auto centroid_it = last_image_centroids_.find(block_id);
    auto time_it = last_image_centroid_times_.find(block_id);
    if (centroid_it == last_image_centroids_.end() || time_it == last_image_centroid_times_.end()) {
        return indices;
    }

    if (image_track_timeout_sec_ <= 0.0 || (stamp - time_it->second).toSec() > image_track_timeout_sec_) {
        return indices;
    }

    const cv::Point2f previous = centroid_it->second;
    bool have_nearby_candidate = false;
    for (const int idx : indices) {
        const double dx = static_cast<double>(det.centroids[static_cast<size_t>(idx)].x) - previous.x;
        const double dy = static_cast<double>(det.centroids[static_cast<size_t>(idx)].y) - previous.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= max_candidate_jump_px_) {
            have_nearby_candidate = true;
            break;
        }
    }

    if (!have_nearby_candidate) {
        if (dropped_far_candidates) {
            *dropped_far_candidates = true;
        }
        return {};
    }

    std::stable_sort(indices.begin(), indices.end(),
                     [&](int a, int b) {
                         const cv::Point& pa = det.centroids[static_cast<size_t>(a)];
                         const cv::Point& pb = det.centroids[static_cast<size_t>(b)];
                         const double dxa = static_cast<double>(pa.x) - previous.x;
                         const double dya = static_cast<double>(pa.y) - previous.y;
                         const double dxb = static_cast<double>(pb.x) - previous.x;
                         const double dyb = static_cast<double>(pb.y) - previous.y;
                         const double dist_a = std::sqrt(dxa * dxa + dya * dya);
                         const double dist_b = std::sqrt(dxb * dxb + dyb * dyb);
                         const bool near_a = dist_a <= max_candidate_jump_px_;
                         const bool near_b = dist_b <= max_candidate_jump_px_;
                         if (near_a != near_b) {
                             return near_a;
                         }
                         if (std::fabs(dist_a - dist_b) > 1e-6) {
                             return dist_a < dist_b;
                         }
                         return det.areas[static_cast<size_t>(a)] > det.areas[static_cast<size_t>(b)];
                     });
    return indices;
}

void rememberImageCentroid(int block_id, const cv::Point& centroid, const ros::Time& stamp) {
    last_image_centroids_[block_id] = centroid;
    last_image_centroid_times_[block_id] = stamp;
}

void pruneTrackingState(const ros::Time& now) {
    if (image_track_timeout_sec_ > 0.0) {
        for (auto it = last_image_centroid_times_.begin(); it != last_image_centroid_times_.end();) {
            if ((now - it->second).toSec() > image_track_timeout_sec_) {
                last_image_centroids_.erase(it->first);
                it = last_image_centroid_times_.erase(it);
            } else {
                ++it;
            }
        }
    }

    if (position_reset_timeout_sec_ > 0.0) {
        for (auto it = filtered_position_times_.begin(); it != filtered_position_times_.end();) {
            if ((now - it->second).toSec() > position_reset_timeout_sec_) {
                filtered_positions_.erase(it->first);
                it = filtered_position_times_.erase(it);
            } else {
                ++it;
            }
        }
    }
}

bool violatesRecentPositionTrack(int block_id,
                               const geometry_msgs::Point& current,
                               const ros::Time& stamp,
                               double& jump_m) const {
    jump_m = 0.0;
    if (max_position_jump_m_ <= 0.0 || position_reset_timeout_sec_ <= 0.0) {
        return false;
    }

    auto it = filtered_positions_.find(block_id);
    auto time_it = filtered_position_times_.find(block_id);
    if (it == filtered_positions_.end() || time_it == filtered_position_times_.end()) {
        return false;
    }

    const double age_sec = (stamp - time_it->second).toSec();
    if (age_sec > position_reset_timeout_sec_) {
        return false;
    }

    const double dx = current.x - it->second.x;
    const double dy = current.y - it->second.y;
    const double dz = current.z - it->second.z;
    jump_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    return jump_m > max_position_jump_m_;
}

geometry_msgs::Point applyEmaSmoothing(int block_id,
                                       const geometry_msgs::Point& current,
                                       const ros::Time& stamp) {
    auto it = filtered_positions_.find(block_id);
    auto time_it = filtered_position_times_.find(block_id);
    if (it == filtered_positions_.end() || time_it == filtered_position_times_.end()) {
        filtered_positions_[block_id] = current;
        filtered_position_times_[block_id] = stamp;
        return current;
    }

    const double age_sec = (stamp - time_it->second).toSec();
    const double dx = current.x - it->second.x;
    const double dy = current.y - it->second.y;
    const double dz = current.z - it->second.z;
    const double jump_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    if ((position_reset_timeout_sec_ > 0.0 && age_sec > position_reset_timeout_sec_) ||
        (max_position_jump_m_ > 0.0 && jump_m > max_position_jump_m_)) {
        filtered_positions_[block_id] = current;
        filtered_position_times_[block_id] = stamp;
        return current;
    }

    geometry_msgs::Point& prev = it->second;
    prev.x = smoothing_alpha_ * current.x + (1.0 - smoothing_alpha_) * prev.x;
    prev.y = smoothing_alpha_ * current.y + (1.0 - smoothing_alpha_) * prev.y;
    prev.z = smoothing_alpha_ * current.z + (1.0 - smoothing_alpha_) * prev.z;
    filtered_position_times_[block_id] = stamp;
    return prev;
}

void publishBlocks(const std::vector<memory_game::Block>& blocks, const ros::Time& stamp) {
    memory_game::BlockArray out;
    out.header.stamp = stamp;
    out.header.frame_id = target_frame_;
    out.blocks = blocks;
    blocks_pub_.publish(out);
}

void publishDebugImages(const cv::Mat& mask,
                        const cv::Mat& overlay,
                        const std_msgs::Header& header) {
    if (!enable_debug_images_) {
        return;
    }

    if (!mask.empty() && debug_mask_pub_) {
        cv_bridge::CvImage mask_img(header, sensor_msgs::image_encodings::MONO8, mask);
        debug_mask_pub_.publish(mask_img.toImageMsg());
    }

    if (!overlay.empty() && debug_overlay_pub_) {
        cv_bridge::CvImage overlay_img(header, sensor_msgs::image_encodings::BGR8, overlay);
        debug_overlay_pub_.publish(overlay_img.toImageMsg());
    }
}

private:
ros::NodeHandle nh_;
ros::NodeHandle pnh_;

image_transport::ImageTransport it_;

struct DepthFrame {
    cv::Mat image;
    std::string encoding;
    ros::Time stamp;
};

ros::Subscriber color_sub_;
ros::Subscriber depth_sub_;
ros::Subscriber camera_info_sub_;

ros::Publisher blocks_pub_;

image_transport::Publisher debug_mask_pub_;
image_transport::Publisher debug_overlay_pub_;

std::string color_topic_;
std::string depth_topic_;
std::string camera_info_topic_;
std::string target_frame_;
bool disable_red_ = false;

int blur_kernel_size_ = 3;
double min_block_area_;
double max_block_area_;
int mask_open_iterations_;
int mask_close_iterations_;

int depth_window_radius_;
double min_depth_m_;
double max_depth_m_;
double max_depth_age_sec_;
int depth_buffer_size_;

bool enable_debug_images_;
double smoothing_alpha_;
double image_track_timeout_sec_ = 0.5;
double max_candidate_jump_px_ = 120.0;
double position_reset_timeout_sec_ = 0.5;
double max_position_jump_m_ = 0.10;
double base_exclusion_radius_m_ = 0.0;
std::vector<ExclusionZone> extra_exclusions_;
bool workspace_enable_ = false;
double workspace_min_x_ = -10.0;
double workspace_max_x_ = 10.0;
double workspace_min_y_ = -10.0;
double workspace_max_y_ = 10.0;
double workspace_min_z_ = -10.0;
double workspace_max_z_ = 10.0;

std::vector<ColorConfig> color_configs_;

cv::Mat latest_depth_;
std::string latest_depth_encoding_;
ros::Time latest_depth_stamp_;
bool has_depth_;
std::deque<DepthFrame> depth_buffer_;

image_geometry::PinholeCameraModel camera_model_;
bool has_camera_model_;

std::map<int, geometry_msgs::Point> filtered_positions_;
std::map<int, ros::Time> filtered_position_times_;
std::map<int, cv::Point2f> last_image_centroids_;
std::map<int, ros::Time> last_image_centroid_times_;

tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node");
    VisionNode node;
    ros::spin();
    return 0;
}
