#include "../include/sr80_moveit_interface/sr80_moveit_pose_action_client.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <algorithm>

PoseGoalClient::PoseGoalClient(const rclcpp::NodeOptions& options)
    : Node("sr80_pose_goal_action_client", options)
{
    using namespace std::placeholders;

    this->m_PoseGoalClientPtr = rclcpp_action::create_client<sr80_custom_interfaces::action::PoseGoal>(
        this,
        "sr80_pose_goal"
    );
    
    this->m_PoseGoalTimer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PoseGoalClient::send_pose_goal, this)
    );

}

void PoseGoalClient::send_pose_goal()
{   

    RCLCPP_INFO(this->get_logger(), "Sending pose goal");
    using namespace std::placeholders;

    if(!this->m_PoseGoalTimer->is_canceled())
        this->m_PoseGoalTimer->cancel();

    if(!this->m_PoseGoalClientPtr->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action Server for Pose Goal is not available after waiting");
        rclcpp::shutdown();
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x= 1.682;
    pose.position.y = 0;
    pose.position.z = 0.49;
    pose.orientation.x = 0.7071;
    pose.orientation.y = 0.7071;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    auto goal_msg = sr80_custom_interfaces::action::PoseGoal::Goal();
    goal_msg.request.move_group="sr80_arm";
    goal_msg.request.requested_pose = pose;

    RCLCPP_INFO(this->get_logger(), "Sending Pose Goal");

    auto send_goal_options = rclcpp_action::Client<sr80_custom_interfaces::action::PoseGoal>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(
        &PoseGoalClient::pose_goal_response_callback,
        this,
        _1
    );
    send_goal_options.feedback_callback = std::bind(
        &PoseGoalClient::pose_goal_feedback_callback,
        this,
        _1,
        _2
    );
    send_goal_options.result_callback = std::bind(
        &PoseGoalClient::pose_goal_result_callback,
        this,
        _1
    );

    this->m_PoseGoalClientPtr->async_send_goal(goal_msg, send_goal_options);

}

void PoseGoalClient::pose_goal_response_callback(rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::SharedPtr goalHandle)
{

    if(!goalHandle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal was accepted by the server, waiting for result");
    }
}

void PoseGoalClient::pose_goal_feedback_callback(
        rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::SharedPtr goalHandle,
        const std::shared_ptr<const sr80_custom_interfaces::action::PoseGoal::Feedback> feedback)
{
    
}

void PoseGoalClient::pose_goal_result_callback(const rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::WrappedResult& result)
{

    switch(result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }

}

void PoseGoalClient::pose_goal_topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
{   
    
    for(std::size_t i = 0; i < msg->data.size(); i++)
    {
        m_PoseGoalValuesContainer->at(i) = msg->data.at(i);
    }

    *m_PoseGoalArrived ^= true;

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor multi_thread_executor;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<PoseGoalClient>(node_options);
    
    multi_thread_executor.add_node(node);
    multi_thread_executor.spin();

    rclcpp::shutdown();

    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(PoseGoalClient)

