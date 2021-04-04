#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

int main(int argc, char *argv[]) try {
  std::cout << "Waiting for device..." << std::endl;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg;
  // Enable fisheye and pose streams
  // cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 1);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 2);
  // Start pipeline with chosen configuration
  rs2::pipeline_profile pipe_profile = pipe.start(cfg);

  // T265 has two fisheye sensors, we can choose any of them (index 1 or 2)
  const int fisheye_sensor_idx0 = 1;
  const int fisheye_sensor_idx1 = 2;

  std::cout << "Device got. Streaming data" << std::endl;
  // Declare depth colorizer for pretty visualization of depth data
  rs2::colorizer color_map;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  using namespace cv;
  const auto window_name = "Display Image";
  namedWindow(window_name, WINDOW_AUTOSIZE);
  int count = 0;

  while (true) {
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    // Get a frame from the fisheye stream
    rs2::video_frame fisheye_frame0 =
        frames.get_fisheye_frame(fisheye_sensor_idx0);
    rs2::video_frame fisheye_frame1 =
        frames.get_fisheye_frame(fisheye_sensor_idx1);


    // Query frame size (width and height)
    const int w = fisheye_frame0.get_width();
    const int h = fisheye_frame0.get_height();

    // Create OpenCV matrix of size (w,h) from the colorized depth data
    Mat image0(Size(w, h), CV_8UC1, (void *)fisheye_frame0.get_data(),
              Mat::AUTO_STEP);
    Mat image1(Size(w, h), CV_8UC1, (void *)fisheye_frame1.get_data(),
              Mat::AUTO_STEP);
    Mat img_show;
    cv::hconcat(image0, image1, img_show);
    // Update the window with new data
    imshow(window_name, img_show);
    char key = (char) cv::waitKey(10);   // explicit cast
    if (key == 27) break;                // break if `esc' key was pressed. 
    if (key == ' '){
      if(!std::filesystem::exists("imgs")){
        std::filesystem::create_directories("imgs/cam0");
        std::filesystem::create_directories("imgs/cam1");
      }
      std::stringstream ss;
      std::string s;
      ss << std::setfill('0') << std::setw(4) << count++;
      ss >> s;
      std::cout << s << std::endl;
      imwrite("imgs/cam0/"+s+".png", image0);
      imwrite("imgs/cam1/"+s+".png", image1);
    }
  }

  return EXIT_SUCCESS;
} catch (const rs2::error &e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}