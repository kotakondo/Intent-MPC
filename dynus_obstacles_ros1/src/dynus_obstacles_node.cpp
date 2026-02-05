#include <dynus_obstacles_ros1/dynus_obstacles_node.h>

namespace dynus_obstacles {

void ObstacleSpec::evaluate(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) const {
    if (is_static) {
        // Static obstacle - constant position, zero velocity
        pos << x0, y0, z0;
        vel << 0.0, 0.0, 0.0;
        return;
    }

    // Dynamic obstacle - trefoil knot trajectory
    double tt = t / slower + offset;

    // Position
    pos(0) = (sx / 6.0) * (sin(tt) + 2.0 * sin(2.0 * tt)) + x0;
    pos(1) = (sy / 5.0) * (cos(tt) - 2.0 * cos(2.0 * tt)) + y0;
    pos(2) = (sz / 2.0) * (-sin(3.0 * tt)) + z0;

    // Velocity (derivatives)
    double inv_slow = 1.0 / slower;
    vel(0) = (sx / 6.0) * inv_slow * (cos(tt) + 4.0 * cos(2.0 * tt));
    vel(1) = (sy / 5.0) * inv_slow * (-sin(tt) + 4.0 * sin(2.0 * tt));
    vel(2) = -(3.0 * sz / 2.0) * inv_slow * cos(3.0 * tt);
}

DynusObstaclesNode::DynusObstaclesNode(ros::NodeHandle& nh) : nh_(nh) {
    loadParameters();
    generateObstacles();

    // Publishers (use separate topic to avoid conflict with Gazebo's ModelStates)
    model_states_pub_ = nh_.advertise<gazebo_msgs::ModelStates>("/dynus/model_states", 10);

    if (publish_markers_) {
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynus_obstacles/markers", 10);
    }

    // Timer for publishing
    double period = 1.0 / publish_rate_hz_;
    publish_timer_ = nh_.createTimer(ros::Duration(period), &DynusObstaclesNode::publishCallback, this);

    start_time_ = ros::Time::now();

    ROS_INFO("DYNUS Obstacles Node initialized:");
    ROS_INFO("  - Total obstacles: %d", num_obstacles_);
    ROS_INFO("  - Dynamic obstacles: %d", static_cast<int>(num_obstacles_ * dynamic_ratio_));
    ROS_INFO("  - Static obstacles: %d", num_obstacles_ - static_cast<int>(num_obstacles_ * dynamic_ratio_));
    ROS_INFO("  - Publish rate: %.1f Hz", publish_rate_hz_);
    ROS_INFO("  - Seed: %d", seed_);
}

void DynusObstaclesNode::loadParameters() {
    nh_.param<int>("num_obstacles", num_obstacles_, 10);
    nh_.param<double>("dynamic_ratio", dynamic_ratio_, 0.5);
    nh_.param<double>("publish_rate_hz", publish_rate_hz_, 50.0);
    nh_.param<int>("seed", seed_, 0);
    nh_.param<bool>("publish_markers", publish_markers_, true);

    // Spatial ranges (DYNUS defaults)
    nh_.param<double>("x_min", x_min_, 5.0);
    nh_.param<double>("x_max", x_max_, 105.0);
    nh_.param<double>("y_min", y_min_, -15.0);
    nh_.param<double>("y_max", y_max_, 15.0);
    nh_.param<double>("z_min", z_min_, 0.0);
    nh_.param<double>("z_max", z_max_, 7.0);

    // Trajectory parameters
    nh_.param<double>("slower_min", slower_min_, 4.0);
    nh_.param<double>("slower_max", slower_max_, 6.0);
}

void DynusObstaclesNode::generateObstacles() {
    std::mt19937 rng(seed_);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    int num_dynamic = static_cast<int>(num_obstacles_ * dynamic_ratio_);
    int num_static = num_obstacles_ - num_dynamic;

    // Obstacle sizes (from DYNUS)
    Eigen::Vector3d bbox_dynamic(0.8, 0.8, 0.8);              // Small cubes
    Eigen::Vector3d bbox_static_vert(0.4, 0.4, 4.0);          // Vertical pillars
    Eigen::Vector3d bbox_static_horiz(0.4, 4.0, 0.4);         // Horizontal walls
    double percentage_vert = 0.35;  // 35% of static are vertical

    // Scale ranges for dynamic obstacles
    double scale_min = 2.0, scale_max = 4.0;
    double offset_min = 0.0, offset_max = 3.0;

    obstacles_.reserve(num_obstacles_);

    for (int i = 0; i < num_obstacles_; ++i) {
        ObstacleSpec obs;
        // Name will be set after bbox is determined (needs size encoding)
        obs.is_static = (i >= num_dynamic);

        // Random position
        double x = x_min_ + (x_max_ - x_min_) * uniform(rng);
        double y = y_min_ + (y_max_ - y_min_) * uniform(rng);
        double z = z_min_ + (z_max_ - z_min_) * uniform(rng);

        if (obs.is_static) {
            // Static obstacle
            int static_idx = i - num_dynamic;
            bool is_vertical = static_idx < (num_static * percentage_vert);

            if (is_vertical) {
                // Vertical pillar - place at ground level
                z = bbox_static_vert(2) / 2.0;
                obs.bbox = bbox_static_vert;
            } else {
                // Horizontal wall
                obs.bbox = bbox_static_horiz;
            }

            obs.x0 = x;
            obs.y0 = y;
            obs.z0 = z;
            obs.sx = obs.sy = obs.sz = 0.0;
            obs.offset = 0.0;
            obs.slower = 0.0;

        } else {
            // Dynamic obstacle - trefoil trajectory
            obs.bbox = bbox_dynamic;
            obs.x0 = x;
            obs.y0 = y;
            obs.z0 = z;
            obs.sx = scale_min + (scale_max - scale_min) * uniform(rng);
            obs.sy = scale_min + (scale_max - scale_min) * uniform(rng);
            obs.sz = scale_min + (scale_max - scale_min) * uniform(rng);
            obs.offset = offset_min + (offset_max - offset_min) * uniform(rng);
            obs.slower = slower_min_ + (slower_max_ - slower_min_) * uniform(rng);
        }

        // Generate name with size encoding in format: obstacle_dyn[XXX][YYY][ZZZ]
        // fake_detector expects EXACTLY 21 characters: 12-char prefix + 9 numeric digits
        // This matches the format used by Gazebo models like "dynamic_box_080080170"
        int xsize_cm = static_cast<int>(obs.bbox(0) * 100);  // Convert m to cm
        int ysize_cm = static_cast<int>(obs.bbox(1) * 100);
        int zsize_cm = static_cast<int>(obs.bbox(2) * 100);

        char name_buffer[50];
        snprintf(name_buffer, sizeof(name_buffer), "obstacle_d%03d_%03d_%03d",
                 xsize_cm, ysize_cm, zsize_cm);
        obs.name = std::string(name_buffer);

        obstacles_.push_back(obs);
    }

    ROS_INFO("Generated %zu obstacles", obstacles_.size());
}

void DynusObstaclesNode::publishCallback(const ros::TimerEvent& event) {
    double t = (ros::Time::now() - start_time_).toSec();
    publishModelStates(t);

    if (publish_markers_) {
        publishMarkers(t);
    }
}

void DynusObstaclesNode::publishModelStates(double t) {
    gazebo_msgs::ModelStates msg;

    for (const auto& obs : obstacles_) {
        Eigen::Vector3d pos, vel;
        obs.evaluate(t, pos, vel);

        // Name
        msg.name.push_back(obs.name);

        // Pose
        geometry_msgs::Pose pose;
        pose.position.x = pos(0);
        pose.position.y = pos(1);
        pose.position.z = pos(2);
        pose.orientation.w = 1.0;  // No rotation
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        msg.pose.push_back(pose);

        // Twist
        geometry_msgs::Twist twist;
        twist.linear.x = vel(0);
        twist.linear.y = vel(1);
        twist.linear.z = vel(2);
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        msg.twist.push_back(twist);
    }

    model_states_pub_.publish(msg);
}

void DynusObstaclesNode::publishMarkers(double t) {
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < obstacles_.size(); ++i) {
        const auto& obs = obstacles_[i];
        Eigen::Vector3d pos, vel;
        obs.evaluate(t, pos, vel);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "dynus_obstacles";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pos(0);
        marker.pose.position.y = pos(1);
        marker.pose.position.z = pos(2);
        marker.pose.orientation.w = 1.0;

        marker.scale.x = obs.bbox(0);
        marker.scale.y = obs.bbox(1);
        marker.scale.z = obs.bbox(2);

        // Color: red for dynamic, blue for static
        if (obs.is_static) {
            marker.color.r = 0.2;
            marker.color.g = 0.2;
            marker.color.b = 0.8;
        } else {
            marker.color.r = 0.8;
            marker.color.g = 0.2;
            marker.color.b = 0.2;
        }
        marker.color.a = 0.8;

        marker.lifetime = ros::Duration(0.2);  // Persistent

        marker_array.markers.push_back(marker);
    }

    markers_pub_.publish(marker_array);
}

void DynusObstaclesNode::spin() {
    ros::spin();
}

} // namespace dynus_obstacles

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynus_obstacles_node");
    ros::NodeHandle nh("~");

    dynus_obstacles::DynusObstaclesNode node(nh);
    node.spin();

    return 0;
}
