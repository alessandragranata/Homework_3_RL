#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    // Sottoscrizione al topic "/camera"
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera", rclcpp::QoS(10),
        std::bind(&MinimalImagePublisher::image_callback, this, std::placeholders::_1));

    // Pubblicazione delle immagini processate sul topic "/random_image"
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/random_image", 10);

    // Configurazione del SimpleBlobDetector
    configure_blob_detector();
  }

private:
  void configure_blob_detector() {
    // Configura i parametri di SimpleBlobDetector
    params_.minThreshold = 50;
    params_.maxThreshold = 200;

    params_.filterByArea = false;
    params_.minArea = 1500;

    params_.filterByCircularity = true;
    params_.minCircularity = 0.8;

    params_.filterByConvexity = false;
    params_.minConvexity = 0.87;

    params_.filterByInertia = false;
    params_.minInertiaRatio = 0.01;

    detector_ = cv::SimpleBlobDetector::create(params_);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Converte il messaggio ROS in un'immagine OpenCV
    cv::Mat input_image;
    try {
      input_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Trova i blob nell'immagine
    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(input_image, keypoints);

    // Disegna i blob trovati
    cv::Mat output_image;
    cv::drawKeypoints(input_image, keypoints, output_image, cv::Scalar(0, 0, 255),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Converte l'immagine processata in un messaggio ROS
    auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", output_image).toImageMsg();

    // Pubblica l'immagine processata
    publisher_->publish(*output_msg);
    RCLCPP_INFO(this->get_logger(), "Processed image published with %ld blobs detected.", keypoints.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::SimpleBlobDetector::Params params_;
  cv::Ptr<cv::SimpleBlobDetector> detector_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
