#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>
#include <vector>

class CylinderExclusionNode : public rclcpp::Node
{
public:
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
     
     void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    bool cylinder_detected = false;  // Flag to indicate if a cylinder is detected
    size_t window_size = 12;  // Use size_t for consistency

    float expected_radius = 0.15;  // 15 cm radius for the 30 cm diameter cylinder
    float angle_increment = scan->angle_increment;
    float angle_min = scan->angle_min;

    std::vector<float> ranges = scan->ranges;
    float detected_x = 0.0, detected_y = 0.0;  // Variables to store the detected cylinder's position

    // Loop through laser scan ranges and analyze groups of points
    for (size_t i = 0; i < ranges.size() - window_size; i++)
    {
        std::vector<float> x_points, y_points;

        // Collect a small window of points to analyze the surface curvature
        for (size_t j = 0; j < window_size; j++)
        {
            float angle = angle_min + (i + j) * angle_increment;
            float x = ranges.at(i + j) * std::cos(angle);
            float y = ranges.at(i + j) * std::sin(angle);
            x_points.push_back(x);
            y_points.push_back(y);
        }

        // Calculate the distances between consecutive points
        std::vector<float> distances;
        for (size_t k = 1; k < window_size; k++)
        {
            float distance = std::hypot(x_points[k] - x_points[k - 1], y_points[k] - y_points[k - 1]);
            distances.push_back(distance);
        }

        // Check if the distances between points approximate an arc (cylinder curvature)
        float curvature = 0.0;
        for (const auto& dist : distances)
        {
            curvature += dist;
        }
        curvature /= distances.size();  // Average distance between points

        // Compare the average curvature to the expected radius of the cylinder
        if (std::abs(curvature - expected_radius) < 0.05)  // Tolerance for detecting the cylinder
        {
            // If the cylinder is detected, calculate its approximate position
            float sum_x = 0.0, sum_y = 0.0;
            for (size_t k = 0; k < x_points.size(); k++)
            {
                sum_x += x_points[k];
                sum_y += y_points[k];
            }
            detected_x = sum_x / x_points.size();  // Average X position
            detected_y = sum_y / y_points.size();  // Average Y position

            cylinder_detected = true;
            break;  // No need to check further if the cylinder is detected
        }
    }

    // Log to the console whether the cylinder was detected and its approximate position
    if (cylinder_detected)
    {
        RCLCPP_INFO(this->get_logger(), "Cylinder detected at approximate position: x = %f, y = %f",
                    detected_x, detected_y);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No cylinder detected.");
    }
}

    // Function to publish the marker representing the cylindrical object
    void publishCylinderMarker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set marker pose to the user-specified position
        marker.pose.position = cylinder_position_;
        marker.pose.orientation.w = 1.0;  // No rotation

        // Set scale (30cm diameter cylinder)
        marker.scale.x = 0.30;
        marker.scale.y = 0.30;
        marker.scale.z = 1.0;  // Cylinder height

        // Set color (blue cylinder)
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent marker

        // Publish the marker
        marker_publisher_->publish(marker);

        RCLCPP_INFO(this->get_logger(), "Cylinder marker published at x = %f, y = %f, z = %f",
                    cylinder_position_.x, cylinder_position_.y, cylinder_position_.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    float cylinder_radius_;
    geometry_msgs::msg::Point cylinder_position_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderExclusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

