//#include <functional>
//#include <memory>
//#include "rclcpp/rclcpp.hpp"
//#include "interfaces/msg/configuration_data_message.hpp"
//
//class Colossus_subscriber : public rclcpp::Node
//{
//public:
//    Colossus_subscriber() : Node{ "colossus_subscriber" }
//    {
//        using std::placeholders::_1;
//
//        Configuration_data_subscriber = Node::create_subscription<std_msgs::msg::String>(
//            "configuration_data", 5, std::bind(&Colossus_subscriber::configuration_data_callback, this, _1));
//        Fft_data_subscriber = Node::create_subscription<std_msgs::msg::String>(
//            "fft_data", 400, std::bind(&Colossus_subscriber::fft_data_callback, this, _1));
//    }
//
//private:
//    void configuration_data_callback(const std_msgs::msg::String::SharedPtr msg) const
//    {
//        RCLCPP_INFO(Node::get_logger(), "I heard: '%s'", msg->data.c_str());
//
//        //char configDataBytes[10];
//        //std::memcpy(configDataBytes, msg->data, sizeof(configDataBytes));
//
//        //RCLCPP_INFO(node->get_logger(), "Configuration Data recieved");
//        //RCLCPP_INFO(node->get_logger(), "Expected Rotation Rate: %i", data->ExpectedRotationRate);
//        //RCLCPP_INFO(node->get_logger(), "Range In Bins: : %i", data->RangeInBins);
//        //RCLCPP_INFO(node->get_logger(), "Bin Size: %f", data->BinSize / 10000.0);
//        //RCLCPP_INFO(node->get_logger(), "Range In Metres: %f", data->BinSize / 10000.0 * data->RangeInBins);
//        //RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", azimuthSamples);
//    }
//    void fft_data_callback(const std_msgs::msg::String::SharedPtr msg) const
//    {
//        RCLCPP_INFO(Node::get_logger(), "I heard: '%s'", msg->data.c_str());
//    }
//    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Configuration_data_subscriber;
//    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Fft_data_subscriber;
//};
//
int main(int argc, char* argv[])
{
//    rclcpp::init(argc, argv);
//    auto node = std::make_shared<Colossus_subscriber>();
//    rclcpp::spin(node);
//    rclcpp::shutdown();
}
