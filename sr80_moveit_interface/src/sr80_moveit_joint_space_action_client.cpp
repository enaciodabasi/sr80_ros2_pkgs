#include "../include/sr80_moveit_interface/sr80_moveit_joint_space_action_client.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <algorithm>

JointSpaceClient::JointSpaceClient(const rclcpp::NodeOptions& options)
    : Node("sr80_joint_space_action_client", options)
{
    using namespace std::placeholders;
    
    this->m_JointSpaceGoalValuesContainer = new std::vector<float>();

    this->m_JointSpaceGoalSubscriber = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/joint_goal_data",
        10,
        std::bind(&JointSpaceClient::joint_space_goal_topic_callback, this, _1)
    );

    this->m_JointSpaceClientPtr = rclcpp_action::create_client<sr80_custom_interfaces::action::JointSpaceGoal>(
        this,
        "sr80_joint_space_goal"
     );

    this->m_JointSpaceGoalTimer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&JointSpaceClient::send_joint_space_goal, this)
        );
}

// ---------------------- Joint Space Goal Client Functions ----------------------

void JointSpaceClient::send_joint_space_goal()
{   

    //std::lock_guard<std::recursive_mutex> lock(m_UpdateMutex);

    if(!m_JointSpaceGoalValuesContainer->empty())
    {
    using namespace std::placeholders;

    if(!this->m_JointSpaceGoalTimer->is_canceled())
    {
        this->m_JointSpaceGoalTimer->cancel();
    }

    if(!this->m_JointSpaceClientPtr->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(), "Action Server for Joint Space Goal is not available.");
        //rclcpp::shutdown();
    }

    auto joint_space_goal_msg = sr80_custom_interfaces::action::JointSpaceGoal::Goal();
    //joint_space_goal_msg.request.joint_positions = *this->m_JointSpaceGoalValuesContainer;
    //std::vector<float> vec = {-1.0, 1.5, 1.0, -1.0, 0.0, 0.0};
//
    //std::copy(
    //    vec.begin(),
    //    vec.end(),
    //    std::back_inserter(joint_space_goal_msg.request.joint_positions)
    //);

    std::copy(
        m_JointSpaceGoalValuesContainer->begin(),
        m_JointSpaceGoalValuesContainer->end(),
        std::back_inserter(joint_space_goal_msg.request.joint_positions)
    );

    RCLCPP_INFO(this->get_logger(), "Sending joint space goal to action server.");

    auto send_joint_space_goal_options = rclcpp_action::Client<sr80_custom_interfaces::action::JointSpaceGoal>::SendGoalOptions();

    send_joint_space_goal_options.goal_response_callback = std::bind(
        &JointSpaceClient::joint_space_goal_response_callback,
        this,
        _1
    );

    send_joint_space_goal_options.feedback_callback = std::bind(
        &JointSpaceClient::joint_space_feedback_callback,
        this,
        _1,
        _2
    );

    send_joint_space_goal_options.result_callback = std::bind(
        &JointSpaceClient::joint_space_result_callback,
        this,
        _1
    );

    RCLCPP_INFO(this->get_logger(), "Sending goal... insallah");

    this->m_JointSpaceClientPtr->async_send_goal(
        joint_space_goal_msg,
        send_joint_space_goal_options
    );

    m_JointSpaceGoalValuesContainer->clear();
    
    }
}

void JointSpaceClient::joint_space_goal_response_callback(
    rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr joint_space_goal_handle
)   
{
    //std::lock_guard<std::recursive_mutex> lock(m_UpdateMutex);

    if(!joint_space_goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Joint Space Goal was rejected by the server.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Joint Space Goal was accepted by the server, waiting for result");
    }
}

void JointSpaceClient::joint_space_feedback_callback(
    rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr joint_space_goal_handle,
    const std::shared_ptr<const sr80_custom_interfaces::action::JointSpaceGoal::Feedback> joint_space_feedback
)
{
    //std::lock_guard<std::recursive_mutex> lock(m_UpdateMutex);
}

void JointSpaceClient::joint_space_result_callback(
        const rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::WrappedResult& joint_space_result
)
{

    //std::lock_guard<std::recursive_mutex> lock(m_UpdateMutex);

    switch (joint_space_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Joint space goal is successfull, waiting for the next goal...");
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

void JointSpaceClient::joint_space_goal_topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
{

    //std::lock_guard<std::recursive_mutex> lock(m_UpdateMutex);

    RCLCPP_INFO(this->get_logger(), "Joint goal information arrived from the interface:");

    std::copy(
        msg->data.begin(),
        msg->data.end(),
        std::back_inserter(*m_JointSpaceGoalValuesContainer)
    );
    
    RCLCPP_INFO(
        this->get_logger(),
        "L1: %.f L2: %.f L3: %.f L4: %.f 51: %.f L6: %.f",
        msg->data[0],
        msg->data[1],
        msg->data[2],
        msg->data[3],
        msg->data[4],
        msg->data[5]
    );

}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor multi_thread_executor;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<JointSpaceClient>(node_options);
    
    multi_thread_executor.add_node(node);
    multi_thread_executor.spin();

    rclcpp::shutdown();

    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(JointSpaceClient)

