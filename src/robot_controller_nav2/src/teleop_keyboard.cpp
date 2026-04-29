    #include "rclcpp/rclcpp.hpp"
    #include "my_robot_interfaces/msg/speed_angle.hpp"
    #include <thread>
    #include <iostream>
    #include <termios.h>
    #include <unistd.h>
     
    class TeleopKeyboardNode : public rclcpp::Node
    {
    public:
        TeleopKeyboardNode() : Node("teleop_keyboard") 
        {
            controller_pub_ = this->create_publisher<my_robot_interfaces::msg::SpeedAngle>("controller",10);
            input_thread_ = std::thread(&TeleopKeyboardNode::readInput, this);
            RCLCPP_INFO(this->get_logger(), "Teleop started. W/A/S/D to move, Q to quit.");
        }
     
        
        ~TeleopKeyboardNode()
        {
            if (input_thread_.joinable())
                input_thread_.join();
        }

        
    private:
        char getKey()
        {
            struct termios oldt, newt;
            char ch;
            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            ch = getchar();
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            return ch;
        }


        void publishcontroller()
        {
            auto msg = my_robot_interfaces::msg::SpeedAngle();
            msg.velocity = velocity_;
            msg.angle = angle_;
            controller_pub_->publish(msg);
        }

        void readInput()
        {
            while(rclcpp::ok())
            {
                char key = getKey();
                switch(key)
                {
                    case 'w': velocity_ =1.0; angle_ = 0.0; break;
                    case 'd': velocity_ =0.0; angle_ = -1.0; break;
                    case 'a': velocity_ =0.0; angle_ = 1.0; break;
                    case 's': velocity_ =-1.0; angle_ = 0.0; break;
                    case ' ': velocity_ = 0.0; angle_ = 0.0; break;
                    case 'q':
                        RCLCPP_INFO(this->get_logger(),"Quiting...");
                        restoreTerminal();
                        rclcpp::shutdown();
                        return;
                    default:
                        velocity_ = 0.0; angle_ = 0.0;
                }
                RCLCPP_INFO(this->get_logger(), "Velocity: %.2f |\
                                                 Angle: %.2f", velocity_, angle_);
                publishcontroller();
            }
        }

        void restoreTerminal()
        {
            struct termios oldt;
            tcgetattr(STDIN_FILENO, &oldt);
            oldt.c_lflag |= (ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        }



        double velocity_ = 0;
        double angle_ = 0;
        std::thread input_thread_;
        rclcpp::Publisher<my_robot_interfaces::msg::SpeedAngle>::SharedPtr controller_pub_;
    };
     
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<TeleopKeyboardNode>(); 
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }