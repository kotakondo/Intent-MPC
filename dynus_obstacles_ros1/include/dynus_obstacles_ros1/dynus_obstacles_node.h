#ifndef DYNUS_OBSTACLES_NODE_H
#define DYNUS_OBSTACLES_NODE_H

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <random>

namespace dynus_obstacles {

struct ObstacleSpec {
    std::string name;
    bool is_static;

    // Position parameters
    double x0, y0, z0;           // Center position
    double sx, sy, sz;           // Scale factors
    double offset;               // Phase offset
    double slower;               // Time scaling

    // Bounding box
    Eigen::Vector3d bbox;        // [hx, hy, hz]

    // Evaluate position and velocity at time t
    void evaluate(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) const;
};

class DynusObstaclesNode {
public:
    DynusObstaclesNode(ros::NodeHandle& nh);
    ~DynusObstaclesNode() = default;

    void spin();

private:
    ros::NodeHandle nh_;
    ros::Publisher model_states_pub_;
    ros::Publisher markers_pub_;
    ros::Timer publish_timer_;

    // Parameters
    int num_obstacles_;
    double dynamic_ratio_;
    double publish_rate_hz_;
    int seed_;
    bool publish_markers_;

    // Spatial ranges
    double x_min_, x_max_;
    double y_min_, y_max_;
    double z_min_, z_max_;
    double slower_min_, slower_max_;

    // Obstacle specifications
    std::vector<ObstacleSpec> obstacles_;

    // Start time
    ros::Time start_time_;

    // Methods
    void loadParameters();
    void generateObstacles();
    void publishCallback(const ros::TimerEvent& event);
    void publishModelStates(double t);
    void publishMarkers(double t);
};

} // namespace dynus_obstacles

#endif // DYNUS_OBSTACLES_NODE_H
