#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <sr80_custom_interfaces/action/joint_space_goal.hpp>
#include <sr80_custom_interfaces/action/pose_goal.hpp>
#include <sr80_custom_interfaces/msg/double_array.hpp>

class MoveitcppActionClient : public rclcpp::Node
{
    public:

    MoveitcppActionClient(const rclcpp::NodeOptions& options);

    private:

    // Clients and Client Callbacks

    rclcpp_action::Client<sr80_custom_interfaces::action::PoseGoal>::SharedPtr m_PoseActionClientPtr;
    rclcpp::TimerBase::SharedPtr m_PoseGoalTimer;
    void send_goal_pose();
    void response_callback_pose(rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::SharedPtr goal_handle);
    void feedback_callback_pose(rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::SharedPtr goal_handle, const std::shared_ptr<const sr80_custom_interfaces::action::PoseGoal::Feedback> feedback);
    void result_callback_pose(const rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::WrappedResult& result);

    rclcpp_action::Client<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr m_JointActionClientPtr;
    rclcpp::TimerBase::SharedPtr m_JointGoalTimer;
    void send_goal_joint();
    void response_callback_joint(rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr goal_handle);
    void feedback_callback_joint(rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::SharedPtr goal_handle, const std::shared_ptr<const sr80_custom_interfaces::action::JointSpaceGoal::Feedback> feedback);
    void result_callback_joint(const rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::JointSpaceGoal>::WrappedResult& result);

    // 

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_PoseMsgSubscription;
    void callback_pose_data(const geometry_msgs::msg::PoseStamped& pose_msg);
    geometry_msgs::msg::PoseStamped* m_StampedPoseMsg = nullptr;

    rclcpp::CallbackGroup::SharedPtr m_PoseCallbackGroup;

    rclcpp::Subscription<sr80_custom_interfaces::msg::DoubleArray>::SharedPtr m_JointMsgSubscription;
    void callback_joint_data(const sr80_custom_interfaces::msg::DoubleArray& joint_msg);
    sr80_custom_interfaces::msg::DoubleArray* m_JointMsg;

};