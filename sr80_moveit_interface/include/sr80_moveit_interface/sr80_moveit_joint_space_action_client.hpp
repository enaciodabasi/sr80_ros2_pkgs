#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <mutex>
#include <string>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <sr80_custom_interfaces/action/joint_space_goal.hpp>

class JointSpaceClient : public rclcpp::Node
{
    public:

    explicit JointSpaceClient(const rclcpp::NodeOptions& options);

    private:

    rclcpp_action::Client<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr m_JointSpaceClientPtr;

    rclcpp::TimerBase::SharedPtr m_JointSpaceGoalTimer;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_JointSpaceGoalSubscriber;

    std::vector<float>* m_JointSpaceGoalValuesContainer;

    bool* m_JointSpaceGoalArrived;

    mutable std::recursive_mutex m_UpdateMutex;
    

    // ------------------------ Joint Space Goal Functions ------------------------

    void send_joint_space_goal();

    void joint_space_goal_response_callback(
        rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr joint_space_goal_handle
    );

    void joint_space_feedback_callback(
        rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr joint_space_goal_handle,
        const std::shared_ptr<const sr80_custom_interfaces::action::JointSpaceGoal::Feedback> joint_space_feedback
    );

    void joint_space_result_callback(
        const rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::WrappedResult& joint_space_result
    );

    void joint_space_goal_topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const;


    
};