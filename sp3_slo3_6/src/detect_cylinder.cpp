#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>
#include <vector>

/**
 * @file detect_cylinder.cpp
 * @brief This node identifies a cylindrical object using laser scans and publishes its position.
 */

/**
 * @class CylinderExclusionNode
 * @brief A ROS 2 node that identifies a cylindrical object from laser scans and excludes it from mapping.
 */
class CylinderExclusionNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the CylinderExclusionNode class.
     * It declares parameters, subscribes to the `/scan` topic, and publishes markers for the cylindrical object.
     */
    CylinderExclusionNode() : Node("cylinder_exclusion_node")
    {
        // Declare parameters for the cylinder's position and radius
        this->declare_parameter<float>("cylinder_radius", 0.15);  // 15 cm radius for a 30 cm diameter
        this->declare_parameter<float>("cylinder_x", 3.0);  // Default x position
        this->declare_parameter<float>("cylinder_y", 0.0);  // Default y position
        this->declare_parameter<float>("cylinder_z", 0.0);  // Default z position

        // Get parameters
        this->get_parameter("cylinder_radius", cylinder_radius_);
        this->get_parameter("cylinder_x", cylinder_position_.x);
        this->get_parameter("cylinder_y", cylinder_position_.y);
        this->get_parameter("cylinder_z", cylinder_position_.z);

        // Subscribe to laser scan topic
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderExclusionNode::laserCallback, this, std::placeholders::_1));

        // Publisher for visualization markers
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_marker", 10);

        // Publish the marker to represent the cylinder in the environment
        publishCylinderMarker();
    }

private:
    /**
     * @brief Callback function to process laser scan data and detect a cylindrical object.
     * @param scan The laser scan data containing distance readings around the robot.
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    /**
     * @brief Publishes a marker representing the cylindrical object in the simulation environment.
     */
    void publishCylinderMarker();

    /// Subscription to the `/scan` topic to receive laser scan data.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    
    /// Publisher for the `/cylinder_marker` topic to visualize the detected cylinder.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    
    /// Radius of the cylindrical object being detected.
    float cylinder_radius_;
    
    /// Position of the cylindrical object in the environment.
    geometry_msgs::msg::Point cylinder_position_;
};

/**
 * @brief Main function to initialize and run the CylinderExclusionNode.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Integer representing success or failure.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderExclusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
