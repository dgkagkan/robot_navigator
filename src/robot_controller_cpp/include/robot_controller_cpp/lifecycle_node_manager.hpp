#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <thread>
#include <chrono>
#include <vector>
#include <future>

namespace lifecycle_node_manager_cpp
{
class LifecycleNodeManager : public rclcpp::Node
{
public:
    LifecycleNodeManager(const rclcpp::NodeOptions &options);

    bool change_state(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
                      uint8_t transition_id, std::string label);

    void initialization_sequence();

private:
    std::vector<rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> clients_;
};    
}