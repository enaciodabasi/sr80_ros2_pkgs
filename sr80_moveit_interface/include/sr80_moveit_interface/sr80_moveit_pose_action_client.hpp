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

#include <sr80_custom_interfaces/action/pose_goal.hpp>
#include <sr80_custom_interfaces/action/joint_space_goal.hpp>

class PoseGoalClient : public rclcpp::Node
{
    public:

    explicit PoseGoalClient(const rclcpp::NodeOptions& options);

    private:

    rclcpp_action::Client<sr80_custom_interfaces::action::PoseGoal>::SharedPtr m_PoseGoalClientPtr;

    rclcpp::TimerBase::SharedPtr m_PoseGoalTimer;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr poseGoalSubscriber;

    std::vector<float>* m_PoseGoalValuesContainer;

    bool* m_PoseGoalArrived;



    void send_pose_goal();

    void pose_goal_response_callback(
        rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::SharedPtr goalHandle
    );

    void pose_goal_feedback_callback(
        rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::SharedPtr goalHandle,
        const std::shared_ptr<const sr80_custom_interfaces::action::PoseGoal::Feedback> feedback);

    void pose_goal_result_callback(
        const rclcpp_action::ClientGoalHandle<sr80_custom_interfaces::action::PoseGoal>::WrappedResult& result);

    void pose_goal_topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const;
    
};