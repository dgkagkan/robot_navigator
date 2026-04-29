#include "rclcpp/rclcpp.hpp"
#include "robot_controller_cpp/lifecycle_node_manager.hpp"

namespace lifecycle_node_manager_cpp
{
    LifecycleNodeManager::LifecycleNodeManager(const rclcpp::NodeOptions &options) : Node("lifecycle_manager", options)
    {
        this->declare_parameter("managed_node_names", std::vector<std::string>());
        std::vector<std::string> node_names = this->get_parameter("managed_node_names").as_string_array();
        for(const auto &node_name : node_names)
        {
            std::string service_name = "/" + node_name + "/change_state";
            this->clients_.push_back(
                this->create_client<lifecycle_msgs::srv::ChangeState>(service_name)
            );
        }
        std::thread{std::bind(&LifecycleNodeManager::initialization_sequence, this)}.detach();
    }

    bool LifecycleNodeManager::change_state(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
                      uint8_t transition_id, std::string label)
    {
        client->wait_for_service();
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;
        request->transition.label = label;

        std::promise<bool> promise;
        auto future_result = promise.get_future();

        client->async_send_request(request,
            [&promise, this, label](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
                bool success = future.get()->success;
                if(success)
                    RCLCPP_INFO(this->get_logger(), "Transition '%s' succeeded", label.c_str());
                else
                    RCLCPP_WARN(this->get_logger(), "Transition '%s' failed", label.c_str());
                promise.set_value(success);
            }
        );

        return future_result.get();
    }

    void LifecycleNodeManager::initialization_sequence()
    {   
        bool all_configured = true;
        RCLCPP_INFO(this->get_logger(), "Trying to switch to configuring");
        for (const auto &client : clients_)
        {
            if (!change_state(client, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configure"))
            {
                all_configured = false;
            }
        }

        if(!all_configured){
            RCLCPP_WARN(this->get_logger(), "Some nodes failed to configure");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "All nodes configured");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "Trying to switch to activating");
        for(const auto &client : clients_)
        {
            change_state(client, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate");
        }
        RCLCPP_INFO(this->get_logger(), "All nodes are now active");
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_node_manager_cpp::LifecycleNodeManager)