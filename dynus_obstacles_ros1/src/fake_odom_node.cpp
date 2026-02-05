#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class FakeOdomNode {
public:
    FakeOdomNode(ros::NodeHandle& nh) : nh_(nh), trajectory_index_(0) {
        loadParameters();
        loadTrajectory();

        // Publishers
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/CERLAB/quadcopter/odom", 10);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/CERLAB/quadcopter/pose", 10);

        // Timer
        double period = 1.0 / publish_rate_;
        publish_timer_ = nh_.createTimer(ros::Duration(period), &FakeOdomNode::publishCallback, this);

        start_time_ = ros::Time::now();

        ROS_INFO("Fake Odometry Node initialized:");
        ROS_INFO("  - Trajectory file: %s", trajectory_file_.c_str());
        ROS_INFO("  - Waypoints loaded: %zu", trajectory_.size());
        ROS_INFO("  - Publish rate: %.1f Hz", publish_rate_);
    }

    void spin() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Timer publish_timer_;

    std::string trajectory_file_;
    double publish_rate_;
    double velocity_;  // Desired velocity along trajectory
    ros::Time start_time_;

    struct Waypoint {
        double time;
        Eigen::Vector3d position;
    };

    std::vector<Waypoint> trajectory_;
    size_t trajectory_index_;

    void loadParameters() {
        nh_.param<std::string>("trajectory_file", trajectory_file_,
                               "/cfg/mpc_navigation/ref_trajectory_straight_line.txt");
        nh_.param<double>("publish_rate", publish_rate_, 100.0);
        nh_.param<double>("velocity", velocity_, 1.5);
    }

    void loadTrajectory() {
        // Construct full path to trajectory file
        // trajectory_file_ should be like "/cfg/mpc_navigation/ref_trajectory_straight_line.txt"
        // We need to prepend the package path
        std::string pkg_path = ros::package::getPath("autonomous_flight");
        if (pkg_path.empty()) {
            ROS_ERROR("Could not find autonomous_flight package!");
            return;
        }

        std::string full_path = pkg_path + trajectory_file_;

        std::ifstream file(full_path);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open trajectory file: %s", full_path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Waypoint wp;
            iss >> wp.time >> wp.position(0) >> wp.position(1) >> wp.position(2);
            trajectory_.push_back(wp);
        }

        file.close();
        ROS_INFO("Loaded %zu waypoints from %s", trajectory_.size(), full_path.c_str());
    }

    void publishCallback(const ros::TimerEvent& event) {
        if (trajectory_.empty()) {
            ROS_WARN_THROTTLE(5.0, "No trajectory loaded!");
            return;
        }

        double t = (ros::Time::now() - start_time_).toSec();

        // Find current position along trajectory based on time
        Eigen::Vector3d position, velocity;
        getCurrentState(t, position, velocity);

        // Publish Odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = position(0);
        odom_msg.pose.pose.position.y = position(1);
        odom_msg.pose.pose.position.z = position(2);
        odom_msg.pose.pose.orientation.w = 1.0;  // No rotation

        odom_msg.twist.twist.linear.x = velocity(0);
        odom_msg.twist.twist.linear.y = velocity(1);
        odom_msg.twist.twist.linear.z = velocity(2);

        odom_pub_.publish(odom_msg);

        // Publish Pose
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose.pose;

        pose_pub_.publish(pose_msg);
    }

    void getCurrentState(double t, Eigen::Vector3d& position, Eigen::Vector3d& velocity) {
        // Find the two waypoints that bracket the current time
        size_t idx = 0;
        for (size_t i = 0; i < trajectory_.size() - 1; ++i) {
            if (t >= trajectory_[i].time && t < trajectory_[i + 1].time) {
                idx = i;
                break;
            }
        }

        // Handle end of trajectory
        if (t >= trajectory_.back().time) {
            position = trajectory_.back().position;
            velocity.setZero();
            return;
        }

        // Linear interpolation between waypoints
        double t0 = trajectory_[idx].time;
        double t1 = trajectory_[idx + 1].time;
        double alpha = (t - t0) / (t1 - t0);

        position = (1.0 - alpha) * trajectory_[idx].position +
                   alpha * trajectory_[idx + 1].position;

        // Velocity = direction * speed
        Eigen::Vector3d direction = trajectory_[idx + 1].position - trajectory_[idx].position;
        double distance = direction.norm();
        if (distance > 1e-6) {
            velocity = direction / distance * velocity_;
        } else {
            velocity.setZero();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_odom_node");
    ros::NodeHandle nh("~");

    FakeOdomNode node(nh);
    node.spin();

    return 0;
}
