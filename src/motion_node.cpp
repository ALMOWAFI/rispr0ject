#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory_game/Block.h>
#include <memory_game/BlockSequence.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <deque>
#include <string>

// ---------------- STATE MACHINE ----------------
enum MotionState {
    IDLE,
    MOVING_TO_TARGET,
    AT_TARGET,
    RETURNING_HOME
};

MotionState current_state = IDLE;

// ---------------- GLOBALS ----------------
ros::Publisher marker_pub;
ros::Publisher status_pub;
std::string marker_frame = "panda_link0";

geometry_msgs::Point current_pos;
geometry_msgs::Point target_pos;
geometry_msgs::Point home_pos;
std::deque<memory_game::Block> target_queue;

double speed = 0.02;
ros::Time state_start_time;

int active_block_id = -1;

void publishStatus(const std::string& state, int block_id = -1);

void startNextTarget()
{
    if (target_queue.empty()) {
        return;
    }

    const memory_game::Block next = target_queue.front();
    target_queue.pop_front();

    active_block_id = next.id;
    ROS_INFO("Moving to block %d", next.id);

    target_pos.x = 0.4;
    target_pos.y = 0.1 * next.id;
    target_pos.z = 0.2;

    current_state = MOVING_TO_TARGET;
    publishStatus("MOVING_TO_TARGET", active_block_id);
}


// ---------------- STATUS PUBLISH ----------------
void publishStatus(const std::string& state, int block_id)
{
    std_msgs::String msg;
    if (block_id >= 0) {
        msg.data = state + ":" + std::to_string(block_id);
    } else {
        msg.data = state;
    }
    status_pub.publish(msg);
}


// ---------------- COLOR BY STATE ----------------
void setColor(visualization_msgs::Marker& marker)
{
    marker.color.a = 1.0;

    switch(current_state)
    {
        case MOVING_TO_TARGET:
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0; // Yellow
            break;

        case AT_TARGET:
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0; // Green
            break;

        case RETURNING_HOME:
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0; // Blue
            break;

        default:
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0; // White
    }
}


// ---------------- PUBLISH MARKER ----------------
void publishMarker()
{
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = marker_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "motion";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = current_pos;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    setColor(marker);

    array.markers.push_back(marker);
    marker_pub.publish(array);
}


// ---------------- DISTANCE FUNCTION ----------------
double distance3D(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;

    return sqrt(dx*dx + dy*dy + dz*dz);
}


// ---------------- MOVE STEP ----------------
void moveTowards(const geometry_msgs::Point& goal)
{
    double dx = goal.x - current_pos.x;
    double dy = goal.y - current_pos.y;
    double dz = goal.z - current_pos.z;

    double dist = sqrt(dx*dx + dy*dy + dz*dz);

    if (dist < 0.01)
        return;

    current_pos.x += speed * dx;
    current_pos.y += speed * dy;
    current_pos.z += speed * dz;
}


// ---------------- TARGET CALLBACK ----------------
void enqueueTarget(const memory_game::Block& block)
{
    target_queue.push_back(block);
    ROS_INFO("Queued block %d (pending=%zu)", block.id, target_queue.size());

    if (current_state == IDLE) {
        startNextTarget();
    }
}

void targetCallback(const memory_game::Block::ConstPtr& msg)
{
    enqueueTarget(*msg);
}

void sequenceCallback(const memory_game::BlockSequence::ConstPtr& msg)
{
    if (!msg) {
        return;
    }

    for (const auto& block : msg->blocks) {
        enqueueTarget(block);
    }
}


// ---------------- MAIN ----------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("marker_frame", marker_frame, std::string("panda_link0"));

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
    status_pub = nh.advertise<std_msgs::String>("/motion_status", 10);

    ros::Subscriber sub = nh.subscribe("/target_block", 10, targetCallback);
    ros::Subscriber seq_sub = nh.subscribe("/target_sequence", 10, sequenceCallback);

    // Home position
    home_pos.x = 0.3;
    home_pos.y = 0.0;
    home_pos.z = 0.2;

    current_pos = home_pos;
    publishStatus("IDLE");

    ROS_WARN("motion_node is a marker/demo node. It ignores /target_block.position and uses synthetic targets.");

    ros::Rate rate(60);

    while (ros::ok())
    {
        switch(current_state)
        {

            case MOVING_TO_TARGET:

                moveTowards(target_pos);

                if (distance3D(current_pos, target_pos) < 0.01)
                {
                    current_state = AT_TARGET;
                    state_start_time = ros::Time::now();
                    publishStatus("AT_TARGET", active_block_id);
                }

                break;


            case AT_TARGET:

                if ((ros::Time::now() - state_start_time).toSec() > 1.0)
                {
                    current_state = RETURNING_HOME;
                    publishStatus("RETURNING_HOME", active_block_id);
                }

                break;


            case RETURNING_HOME:

                moveTowards(home_pos);

                if (distance3D(current_pos, home_pos) < 0.01)
                {
                    if (!target_queue.empty()) {
                        startNextTarget();
                    } else {
                        current_state = IDLE;
                        publishStatus("IDLE");
                    }
                }

                break;


            default:
                break;
        }

        publishMarker();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
