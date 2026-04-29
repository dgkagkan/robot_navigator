#include "rclcpp/rclcpp.hpp"
#include "robot_controller_cpp/turtle_controller.hpp"

namespace turtle_controller_cpp
{
using namespace std::placeholders;
using TurtleController = my_robot_interfaces::action::TurtleController;
using TurtleControllerGoalHandle = rclcpp_action::ServerGoalHandle<TurtleController>;
using LifecycleCallbackReturn = 
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


    TurtleControllerNode::TurtleControllerNode(const rclcpp::NodeOptions &options) : LifecycleNode("turtle_controller",options)
    {
        RCLCPP_INFO(this->get_logger(),"In contructor");
    }
    LifecycleCallbackReturn TurtleControllerNode::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(),"In on_configure");
        cancel_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav2_cancel",10);
        controller_sub_ = this->create_subscription<my_robot_interfaces::msg::SpeedAngle>(
            "controller",10,
            std::bind(&TurtleControllerNode::controller_callback,this,_1));
        goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav2_goal", 10);
        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav2_initial_pose",10);
        pose_subscriber_=this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&TurtleControllerNode::pose_callback,this,_1));
        cmd_vel_publisher_= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        controller_server_=rclcpp_action::create_server<TurtleController>(
            this,
            "controller",
            std::bind(&TurtleControllerNode::goal_callback,this,_1,_2),
            std::bind(&TurtleControllerNode::cancel_callback,this,_1),
            std::bind(&TurtleControllerNode::handle_accepted_goal,this,_1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn TurtleControllerNode::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(),"In on_cleanup");
        controller_server_.reset();
        cmd_vel_publisher_.reset();
        pose_subscriber_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn TurtleControllerNode::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(),"In on_active");
        cmd_vel_publisher_->on_activate();
        this->active_state_ = true;
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn TurtleControllerNode::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(),"In on_deactive");
        cmd_vel_publisher_->on_deactivate();
        this->active_state_= false;
        this->initialize_pose_ = false;
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn TurtleControllerNode::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(),"In on_shutdown");
        controller_server_.reset();
        cmd_vel_publisher_.reset();
        pose_subscriber_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

    rclcpp_action::GoalResponse TurtleControllerNode::goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TurtleController::Goal> goal)
        {
            (void)uuid;
            RCLCPP_INFO(this->get_logger(),"Recieved a goal");
            if(goal->velocity<=0 || goal->x<-20 || goal->x>=20 || goal->y<-20 || goal->y>=20 ){
                RCLCPP_INFO(this->get_logger(),"Rejectring the goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
            else if (!this->active_state_)
            {
                RCLCPP_INFO(this->get_logger(),"The goal is rejected due deactivation");
                return rclcpp_action::GoalResponse::REJECT;
            }
            else if(current_goal_handle_!=nullptr && current_goal_handle_->is_active()){
                RCLCPP_INFO(this->get_logger(),"Snother goal is running");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::CancelResponse TurtleControllerNode::cancel_callback(const std::shared_ptr<TurtleControllerGoalHandle>goal_handle)
    {
        RCLCPP_INFO(this->get_logger(),"Received a canel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void TurtleControllerNode::handle_accepted_goal(const std::shared_ptr<TurtleControllerGoalHandle>goal_handle)
    {
        std::thread{std::bind(&TurtleControllerNode::execute_goal, this, _1), goal_handle}.detach();
    }

    void TurtleControllerNode::execute_goal(const std::shared_ptr<TurtleControllerGoalHandle>goal_handle)
    {
        auto result = std::make_shared<TurtleController::Result>();
        auto feedback = std::make_shared<TurtleController::Feedback>();
        auto goal = goal_handle->get_goal();
        double target_x = goal->x;
        double target_y = goal->y;
        double velocity = goal->velocity;
        current_goal_handle_ = goal_handle;

        if(goal->use_nav2){

                //initialize the position on nav2
                if(!this->initialize_pose_){
                    auto init_msg = geometry_msgs::msg::PoseStamped();
                init_msg.header.frame_id = "map";
                init_msg.header.stamp = this->now();
                init_msg.pose.position.x = 0.0;
                init_msg.pose.position.y = -3.0;
                init_msg.pose.position.z = 0.0;
                init_msg.pose.orientation.z = 0.0;
                init_msg.pose.orientation.w = 1.0;
                initial_pose_publisher_->publish(init_msg);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                this->initialize_pose_ = true;
                }
                //send the goal after initialization
                auto pose_msg = geometry_msgs::msg::PoseStamped();
                pose_msg.header.frame_id = "map";
                pose_msg.header.stamp = this->now();
                pose_msg.pose.position.x = target_x;
                pose_msg.pose.position.y = target_y;
                pose_msg.pose.position.z = 0.0;
                pose_msg.pose.orientation.w = 1.0;
                goal_pose_publisher_->publish(pose_msg);

                result->msg = "Goal sent to Nav2!";
                goal_handle->succeed(result);
                current_goal_handle_ = nullptr;
                return; 
            }
         rclcpp::Rate rate(10);
        while(rclcpp::ok())
        {
            if(goal_handle->is_canceling()){
            result->msg = "The goal is cancelled from the client";
            goal_handle->canceled(result);
            return;
            }
            double dx = - current_x_ + target_x;
            double dy = - current_y_ + target_y;
            double distance = std::sqrt(std::pow(dx,2) + std::pow(dy,2));
            double angle_to_target = std::atan2(dy ,dx);
            double angle_error= angle_to_target - current_theta_;
            auto msg = geometry_msgs::msg::Twist();
            while (angle_error > M_PI) angle_error -= 2 * M_PI;
            while (angle_error < -M_PI) angle_error += 2 * M_PI;
            if (distance<0.5){
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                cmd_vel_publisher_->publish(msg);
                break;
            }
            else if(std::abs(angle_error)>M_PI/4){
                msg.linear.x = 0.0;
                msg.angular.z = std::clamp(2.0 * angle_error, -1.0, 1.0);
            }
            else {
                msg.linear.x = std::clamp(distance * velocity, 0.5, 1.0);
                msg.angular.z = std::clamp(2.0 * angle_error, -2.0, 2.0);
            }
            

            cmd_vel_publisher_->publish(msg);
            rate.sleep();
                
        }
        current_goal_handle_ = nullptr;
        result->msg = "Turtle reached the destination!";
        goal_handle->succeed(result);
    }

    void TurtleControllerNode::pose_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        auto q = msg->pose.pose.orientation;
        current_theta_ = std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    }

    void TurtleControllerNode::controller_callback(const std::shared_ptr<const my_robot_interfaces::msg::SpeedAngle> msg)
    {
        if(!this->initialize_pose_){
            auto init_msg = geometry_msgs::msg::PoseStamped();
            init_msg.header.frame_id = "map";
            init_msg.header.stamp = this->now();
            init_msg.pose.position.x = 0.0;
            init_msg.pose.position.y = -3.0;
            init_msg.pose.orientation.w = 1.0;
            initial_pose_publisher_->publish(init_msg);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            this->initialize_pose_ = true;
        }

        int vel = static_cast<int>(msg->velocity);
        int ang = static_cast<int>(msg->angle);
        if(vel == 0 && ang == 0){
            auto cancel_msg = geometry_msgs::msg::PoseStamped();
            cancel_pub_->publish(cancel_msg);
            return;
        }

        nav2_x_ += vel;
        nav2_y_ += ang;

        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->now();
        pose_msg.pose.position.x = nav2_x_;
        pose_msg.pose.position.y = nav2_y_ - 3;
        pose_msg.pose.orientation.w = 1.0;
        goal_pose_publisher_->publish(pose_msg);
    }


}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(turtle_controller_cpp::TurtleControllerNode)