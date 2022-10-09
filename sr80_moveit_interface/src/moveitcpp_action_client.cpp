#include "../include/sr80_moveit_interface/moveitcpp_action_client.hpp"

#include <algorithm>

#include <geometry_msgs/msg/pose_stamped.hpp>

MoveitcppActionClient::MoveitcppActionClient(const rclcpp::NodeOptions& options)
    : Node("moveitcpp_action_client_node", options)
{
    using namespace std::placeholders;

    this->m_PoseActionClientPtr = rclcpp_action::create_client<sr80_custom_interfaces::action::PoseGoal>(this, "sr80_pose_action");

    this->m_JointActionClientPtr = rclcpp_action::create_client<sr80_custom_interfaces::action::JointSpaceGoal>(this, "sr80_joint_action");

    this->m_PoseMsgSubscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "pose_goal_data",
        10,
        std::bind(&MoveitcppActionClient::callback_pose_data, this, _1)
    );  

    this->m_JointMsgSubscription = this->create_subscription<sr80_custom_interfaces::msg::DoubleArray>(
        "joint_goal_data",
        10,
        std::bind(&MoveitcppActionClient::callback_joint_data, this, _1)
    );

    m_StampedPoseMsg = new geometry_msgs::msg::PoseStamped();
    m_JointMsg = new sr80_custom_interfaces::msg::DoubleArray();
    //this->m_JointMsgSubscription = this->create_subscription<sr80_custom_interfaces::msg::DoubleArray>(
    //    "",
    //    10,
    //    std::bind(&MoveitcppActionClient::callback_joint_data, this, _1)
    //);

    //this->m_PoseGoalTimer = this->create_wall_timer(
    //    std::chrono::milliseconds(500), 
    //    std::bind(&MoveitcppActionClient::send_goal_pose, 
    //    this));

}

// ------------------- Pose Client -------------------

void MoveitcppActionClient::response_callback_pose(rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::SharedPtr goal_handle)
{
    if(!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal was accepted by the server, waiting for result...");
    }
}

void MoveitcppActionClient::feedback_callback_pose( rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::SharedPtr goal_handle, const std::shared_ptr<const sr80_custom_interfaces::action::PoseGoal::Feedback> feedback)
{

}

void MoveitcppActionClient::result_callback_pose(const rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::WrappedResult& result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Successfully executed the pose goal.");
        // if(this->m_JointGoalTimer->is_canceled())
        // {
        //     this->m_JointGoalTimer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MoveitcppActionClient::send_goal_pose, this));
        // }
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "Goal was aborted.");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
}

void MoveitcppActionClient::send_goal_pose()
{

    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Sending pose goal to action server.");

    if(!this->m_PoseGoalTimer->is_canceled())
    {
        RCLCPP_INFO(this->get_logger(), "canceling timer");    
        this->m_PoseGoalTimer->cancel();
    }

    if(!this->m_PoseActionClientPtr->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action Server for Pose Goal is not available after waiting");
        rclcpp::shutdown();
    }
    
    RCLCPP_INFO(this->get_logger(), "Copying message data");

    auto goalMsg = sr80_custom_interfaces::action::PoseGoal::Goal();
    geometry_msgs::msg::PoseStamped stampedPose;
    stampedPose.header = m_StampedPoseMsg->header;
    stampedPose.pose = m_StampedPoseMsg->pose;
    //stampedPose.header.frame_id = "base_link";
    //stampedPose.pose.orientation.w = 0;
    //stampedPose.pose.orientation.x = -0.7071;
    //stampedPose.pose.orientation.y = -0.7071;
    //stampedPose.pose.orientation.z = 0;
    //stampedPose.pose.position.x = 1.682;
    //stampedPose.pose.position.y = 0.7;
    //stampedPose.pose.position.z = 0.49;

    goalMsg.request.requested_pose = stampedPose;

    RCLCPP_INFO(this->get_logger(), "Copied message data");

    auto sendGoalOptions = rclcpp_action::Client<sr80_custom_interfaces::action::PoseGoal>::SendGoalOptions();

    sendGoalOptions.goal_response_callback = std::bind(&MoveitcppActionClient::response_callback_pose, this, _1);
    sendGoalOptions.feedback_callback = std::bind(&MoveitcppActionClient::feedback_callback_pose, this, _1, _2);
    sendGoalOptions.result_callback = std::bind(&MoveitcppActionClient::result_callback_pose, this, _1);

    this->m_PoseActionClientPtr->async_send_goal(goalMsg, sendGoalOptions);

}

void MoveitcppActionClient::callback_pose_data(const geometry_msgs::msg::PoseStamped& pose_msg)
{   
    
    RCLCPP_INFO(this->get_logger(), "Received Pose Data");

    m_StampedPoseMsg->header = pose_msg.header;
    m_StampedPoseMsg->pose = pose_msg.pose;

    RCLCPP_INFO(this->get_logger(), "Copied Pose Data");

    this->m_PoseGoalTimer = this->create_wall_timer(
           std::chrono::milliseconds(500), 
           std::bind(&MoveitcppActionClient::send_goal_pose, this)
       );

    

    RCLCPP_INFO(this->get_logger(), "Created timer");

}


// ------------------- Joint Client -------------------

void MoveitcppActionClient::response_callback_joint(rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr goal_handle)
{
    if(!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal was accepted by the server, waiting for result...");
    }
}

void MoveitcppActionClient::feedback_callback_joint(rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr goal_handle, const std::shared_ptr<const sr80_custom_interfaces::action::JointSpaceGoal::Feedback> feedback)
{

}

void MoveitcppActionClient::result_callback_joint(const rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::WrappedResult& result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Successfully executed the joint goal.");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "Goal was aborted.");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
}

void MoveitcppActionClient::send_goal_joint()
{
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Sending joint goal to action server.");

    if(!this->m_JointGoalTimer->is_canceled())
    {
        RCLCPP_INFO(this->get_logger(), "canceling timer");    
        this->m_JointGoalTimer->cancel();
    }

    if(!this->m_JointActionClientPtr->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action Server for Joint Goal is not available after waiting");
        rclcpp::shutdown();
    }
    
    RCLCPP_INFO(this->get_logger(), "Copying message data");

    auto goalMsg = sr80_custom_interfaces::action::JointSpaceGoal::Goal();
    sr80_custom_interfaces::msg::DoubleArray jointValues;
    jointValues.double_array = m_JointMsg->double_array;

    goalMsg.request.joint_positions.double_array = jointValues.double_array;

    //std::copy(
    //    m_JointMsg->double_array.begin(),
    //    m_JointMsg->double_array.end(),
    //    std::back_inserter(goalMsg.request.joint_positions.double_array)
    //);

    RCLCPP_INFO(this->get_logger(), "Copied message data");

    auto sendGoalOptions = rclcpp_action::Client<sr80_custom_interfaces::action::JointSpaceGoal>::SendGoalOptions();

    sendGoalOptions.goal_response_callback = std::bind(&MoveitcppActionClient::response_callback_joint, this, _1);
    sendGoalOptions.feedback_callback = std::bind(&MoveitcppActionClient::feedback_callback_joint, this, _1, _2);
    sendGoalOptions.result_callback = std::bind(&MoveitcppActionClient::result_callback_joint, this, _1);

    this->m_JointActionClientPtr->async_send_goal(goalMsg, sendGoalOptions);
}

void MoveitcppActionClient::callback_joint_data(const sr80_custom_interfaces::msg::DoubleArray& joint_msg)
{
    RCLCPP_INFO(this->get_logger(), "Received Joint Data");

    m_JointMsg->double_array = joint_msg.double_array;

    RCLCPP_INFO(this->get_logger(), "Copied Joint Data");

    this->m_JointGoalTimer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MoveitcppActionClient::send_goal_joint, this)
    );

    RCLCPP_INFO(this->get_logger(), "Created timer");

}


int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto moveitcpp_action_client_node = std::make_shared<MoveitcppActionClient>(node_options);

    rclcpp::executors::MultiThreadedExecutor mt_executor;
    mt_executor.add_node(moveitcpp_action_client_node);

    while(rclcpp::ok())
    {
        mt_executor.spin();
    }

    mt_executor.remove_node(moveitcpp_action_client_node);
    rclcpp::shutdown();

}