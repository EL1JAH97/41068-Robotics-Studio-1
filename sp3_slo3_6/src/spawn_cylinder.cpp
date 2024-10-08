#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include <memory>
#include <fstream>
#include <string>

class SpawnCylinderNode : public rclcpp::Node
{
public:
    SpawnCylinderNode() : Node("spawn_cylinder_node")
    {
        // Create a client to call the spawn entity service
        client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the /spawn_entity service...");
        }

        // Load the SDF model from a file
        std::ifstream file("/home/student/ros2_ws/src/sp3_slo3_6/src/cylinder_model.sdf");
        std::string sdf_content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        
        if (sdf_content.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load cylinder SDF file");
            return;
        }

        // Set the position where the cylinder should be spawned
        float cylinder_x = 3.0;  // X-coordinate
        float cylinder_y = 2.0;  // Y-coordinate
        float cylinder_z = 0.0;  // Z-coordinate (height)

        // Create the request to spawn the cylinder
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "cylinder";
        request->xml = sdf_content;  // SDF file content
        request->robot_namespace = "cylinder";
        request->initial_pose.position.x = cylinder_x;
        request->initial_pose.position.y = cylinder_y;
        request->initial_pose.position.z = cylinder_z;
        request->initial_pose.orientation.w = 0;  // No rotation

        // Call the service to spawn the cylinder
        auto future = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Cylinder spawned successfully.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn the cylinder.");
        }
    }

private:
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpawnCylinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
