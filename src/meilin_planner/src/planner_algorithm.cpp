#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class BasicNode : public rclcpp::Node
{
public:
    BasicNode() : Node("basic_node")
    {
        // 创建订阅者
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "meilin_planner_input",
            10,
            std::bind(&BasicNode::callback, this, std::placeholders::_1)
        );
        // 创建发布者
        pub_ = this->create_publisher<std_msgs::msg::String>(
            "meilin_planner_output",
            10
        );
    }
private:

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "receive: %s", msg->data.c_str());
        
        // 这里调用你的算法
        std::string result = "processed: " + msg->data;

        std_msgs::msg::String out;
        out.data = result;

        pub_->publish(out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BasicNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}