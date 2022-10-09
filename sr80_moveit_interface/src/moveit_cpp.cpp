#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_cpp", "", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor](){executor.spin();}).detach();

    const std::string planning_group = "sr80_arm";

    rclcpp::sleep_for(std::chrono::seconds(1));

    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(planning_group, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto robot_start_state = planning_components->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(planning_group);

    planning_components->setStartStateToCurrentState();

    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = "base_link";
    target_pose1.pose.orientation.w = 0;
    target_pose1.pose.orientation.x = -0.7071;
    target_pose1.pose.orientation.y = 0.7071;
    target_pose1.pose.orientation.z = 0;
    target_pose1.pose.position.x = 1.682;
    target_pose1.pose.position.y = 0;
    target_pose1.pose.position.z = 0.49;

    planning_components->setGoal(target_pose1, "tool0");

    auto plan_solution1 = planning_components->plan();

    if(plan_solution1)
    {
        planning_components->execute();
    }

    executor.remove_node(node);
    rclcpp::shutdown();

} 