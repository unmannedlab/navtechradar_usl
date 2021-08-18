#include <opencv2/opencv.hpp>

class Video_capture_manager {
public:
    cv::VideoCapture capture;
    cv::Mat latest_image{ };
	
    Video_capture_manager();

    int connect_to_camera(std::string camera_url);

    cv::Mat get_latest_frame();

    void test_framerate(int num_captures);

    void disconnect_from_camera();
};

extern std::shared_ptr<Video_capture_manager> vid_cap_manager;