#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>

#include "video_capture_manager.h"

using namespace std;
using namespace cv;

std::shared_ptr<Video_capture_manager> vid_cap_manager{};
std::thread video_capture_thread;

Video_capture_manager::Video_capture_manager(){
	VideoCapture capture;
}

int Video_capture_manager::connect_to_camera(std::string camera_url){
	cout << "Connecting to camera..." << endl;

	capture = VideoCapture{camera_url};
	if (!capture.isOpened()) {
		cout << "Unable to connect to camera" << endl;
		return 0;
	}
	else {
		cout << "Camera connected" << endl;
		cout << "Width: " << capture.get(CAP_PROP_FRAME_WIDTH) << endl;
		cout << "Height: " << capture.get(CAP_PROP_FRAME_HEIGHT) << endl;
		cout << "Fps: " << capture.get(CAP_PROP_FPS) << endl;
		return 1;
	}
	
}

Mat Video_capture_manager::get_latest_frame() {
	Mat latest_image{ };
	auto start = std::chrono::high_resolution_clock::now();
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	while (elapsed.count() < 1 / (capture.get(CAP_PROP_FPS) + 1)) {
		start = std::chrono::high_resolution_clock::now();
		capture >> latest_image;
		finish = std::chrono::high_resolution_clock::now();
		elapsed = finish - start;
	}
	return latest_image;
}

void Video_capture_manager::test_framerate(int num_captures){
	
	// Flush the capture buffer
	auto start = std::chrono::high_resolution_clock::now();
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	while (elapsed.count() < 1 / (capture.get(CAP_PROP_FPS) + 1)){
		start = std::chrono::high_resolution_clock::now();
		capture.grab();
		finish = std::chrono::high_resolution_clock::now();
		elapsed = finish - start;
		//std::cout << "Buffer not empty" << endl;
	}
	std::cout << "Buffer cleared" << endl;

	cout << "Capturing " << num_captures << " frames..." << endl;
	
	start = std::chrono::high_resolution_clock::now();

	Mat captured_image{ };
	for (auto f = 0; f < num_captures; f++){
		capture >> captured_image;
	}
	
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << "Elapsed time: " << elapsed.count() << endl;
	auto fps = num_captures / elapsed.count();
	std::cout << "FPS: " << fps << endl;
}

void Video_capture_manager::disconnect_from_camera(){
	cout << "Disconnecting from camera..." << endl;
	capture.release();
}