#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;

auto config_data_received = false;
VideoWriter video_writer;
int encoder_size;
int azimuth_samples;
int video_width;
int video_height;
int bearing_count = 0;

class Colossus_subscriber_to_video : public rclcpp::Node
{
public:
    Colossus_subscriber_to_video() : Node{ "colossus_subscriber_to_video" }
    {
        using std::placeholders::_1;

        configuration_data_subscriber = Node::create_subscription<interfaces::msg::ConfigurationDataMessage>(
            "radar_data/configuration_data", 5, std::bind(&Colossus_subscriber_to_video::configuration_data_callback, this, _1));
        fft_data_subscriber = Node::create_subscription<interfaces::msg::FftDataMessage>(
            "radar_data/fft_data", 1600, std::bind(&Colossus_subscriber_to_video::fft_data_callback, this, _1));
    }

private:
    void configuration_data_callback(const interfaces::msg::ConfigurationDataMessage::SharedPtr msg) const
    {
        RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");
        RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", msg->azimuth_samples);
        azimuth_samples = msg->azimuth_samples;
        RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", msg->encoder_size);
        encoder_size = msg->encoder_size;
        RCLCPP_INFO(Node::get_logger(), "Bin Size: %i", msg->bin_size);
        RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", msg->range_in_bins);
        RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", msg->expected_rotation_rate);

        video_width = msg->range_in_bins;
        video_height = msg->azimuth_samples;
        video_writer.open("output_videos/radar_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), msg->expected_rotation_rate, Size(video_width, video_height), true);
        config_data_received = true;
    }
    void fft_data_callback(const interfaces::msg::FftDataMessage::SharedPtr msg) const
    {
        if (!config_data_received)
        {
            RCLCPP_INFO(Node::get_logger(), "No Configuration Data Received");
            return;
        }

        int bearing = ((double)msg->azimuth / (double)encoder_size) * (double)azimuth_samples;

        bearing_count++;

        if (bearing_count >= 400) {
            Mat radar_image(Size(video_width, video_height), CV_8UC3, Scalar(0, 0, 0));
            auto image_data = new byte[video_width * video_height];
            video_writer.write(radar_image);
            bearing_count = 0;
        }

        //RCLCPP_INFO(Node::get_logger(), "FFT Data Received");
        //RCLCPP_INFO(Node::get_logger(), "Angle: %f", msg->angle);
        //RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", encoder_size);
        //RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", azimuth_samples);
        //RCLCPP_INFO(Node::get_logger(), "Azimuth: %i", msg->azimuth);
        //RCLCPP_INFO(Node::get_logger(), "Bearing: %i", bearing);
        //RCLCPP_INFO(Node::get_logger(), "Sweep Counter: %i", msg->sweep_counter);
        //RCLCPP_INFO(Node::get_logger(), "NTP Seconds: : %i", msg->ntp_seconds);
        //RCLCPP_INFO(Node::get_logger(), "NTP Split Seconds: %i", msg->ntp_split_seconds);

        //radar_image = Mat::zeros(video_width, video_height, CV_8UC3);
        //Mat radar_image(Size(video_width, video_height), CV_8UC3, Scalar(0, 0, 0));
    }

    rclcpp::Subscription<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<interfaces::msg::FftDataMessage>::SharedPtr fft_data_subscriber;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Colossus_subscriber_to_video>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
