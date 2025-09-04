#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

// MoveIt 2
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_visual_tools/moveit_visual_tools.h"

// Shapes
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

// Messages
#include "shape_msgs/msg/mesh.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "geometry_msgs/msg/pose.hpp"

class BuffingPlannerNode : public rclcpp::Node
{
public:
    BuffingPlannerNode()
        : Node("buffing_planner_node")
    {
        RCLCPP_INFO(get_logger(), "Starting Buffing Planner Node...");
    }

    void initialize()
    {
        auto node_ptr = shared_from_this(); // now safe

        // MoveIt 2 interfaces
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "dobot_arm");
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_ptr, "base_link");

        visual_tools_->deleteAllMarkers();

        // Load STL mesh
        std::string stl_file = "file:///home/bonnybk/Projects/dobot_ros/ros_ws/src/buffing_planner/meshes/tunning_fork.stl";
        shapes::Mesh *mesh_shape = shapes::createMeshFromResource(stl_file);
        if (!mesh_shape)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load STL mesh: %s", stl_file.c_str());
            return;
        }

        shapes::ShapeMsg mesh_msg;
        if (!shapes::constructMsgFromShape(mesh_shape, mesh_msg))
        {
            RCLCPP_ERROR(get_logger(), "Failed to convert mesh to ShapeMsg");
            return;
        }

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "buffing_part";
        collision_object.header.frame_id = move_group_->getPlanningFrame();
        collision_object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));

        geometry_msgs::msg::Pose mesh_pose;
        mesh_pose.orientation.w = 1.0;
        mesh_pose.position.x = 0.6;
        mesh_pose.position.y = 0.2;
        mesh_pose.position.z = 0.1;
        collision_object.mesh_poses.push_back(mesh_pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_->applyCollisionObject(collision_object);
        RCLCPP_INFO(get_logger(), "Mesh added to planning scene.");

        planBuffingPath();
    }

private:
    void planBuffingPath()
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = 0.3;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.1;

        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(get_logger(), "Plan successful, visualizing trajectory...");
            visual_tools_->publishTrajectoryLine(plan.trajectory_, move_group_->getCurrentState()->getJointModelGroup("dobot_arm"));
            visual_tools_->trigger();
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Planning failed!");
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BuffingPlannerNode>();

    // Initialize MoveIt objects safely
    node->initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
