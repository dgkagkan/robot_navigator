#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/action/turtle_controller.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "my_robot_interfaces/msg/speed_angle.hpp"

namespace turtle_controller_cpp
{
using namespace std::placeholders;
using TurtleController = my_robot_interfaces::action::TurtleController;
using TurtleControllerGoalHandle = rclcpp_action::ServerGoalHandle<TurtleController>;
using LifecycleCallbackReturn = 
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TurtleControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    TurtleControllerNode(const rclcpp::NodeOptions &options);

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);
private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TurtleController::Goal> goal);

    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<TurtleControllerGoalHandle>goal_handle);

    void handle_accepted_goal(const std::shared_ptr<TurtleControllerGoalHandle>goal_handle);

    void execute_goal(const std::shared_ptr<TurtleControllerGoalHandle>goal_handle);

    void pose_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> msg);

    void controller_callback(const std::shared_ptr<const my_robot_interfaces::msg::SpeedAngle> msg);

    double current_x_;
    double current_y_;
    double current_theta_;
    bool active_state_= false; 
    bool initialize_pose_ = false;
    double nav2_x_;
    double nav2_y_;
    std::shared_ptr<TurtleControllerGoalHandle> current_goal_handle_;
    rclcpp_action::Server<TurtleController>::SharedPtr controller_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
    rclcpp::Subscription<my_robot_interfaces::msg::SpeedAngle>::SharedPtr controller_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cancel_pub_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
};
}