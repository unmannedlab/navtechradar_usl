#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "radar_client.h"

class Colossus_and_dual_camera_publisher : public ::rclcpp::Node {
public:
    Colossus_and_dual_camera_publisher();

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);

    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);

    void camera_image_handler(cv::Mat image_left, cv::Mat image_right, int fps_left, int fps_right);

    void cleanup_and_shutdown();
};

extern std::shared_ptr<Colossus_and_dual_camera_publisher> node;
extern std::shared_ptr<Navtech::Radar_client> radar_client;
extern std::string radar_ip;
extern uint16_t radar_port;
extern std::string camera_left_url;
extern rclcpp::Publisher<interfaces::msg::CameraImageMessage>::SharedPtr camera_left_image_publisher;
extern std::string camera_right_url;
extern rclcpp::Publisher<interfaces::msg::CameraImageMessage>::SharedPtr camera_right_image_publisher;