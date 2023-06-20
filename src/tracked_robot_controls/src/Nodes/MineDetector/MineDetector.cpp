#include "Nodes/MineDetector/MineDetector.h"

#include <zbar.h>

MineDetector::MineDetector()
    : Node("MineDetector")
{
    mSubscription = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&MineDetector::onImageReceived, this, std::placeholders::_1));
}


void MineDetector::onImageReceived(const sensor_msgs::msg::Image::SharedPtr image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        std::cout << e.what() << std::endl;
        return;
    }

    cv::Mat mimage = cv_ptr->image;
    cv::Mat gray;
    cv::cvtColor(mimage, gray, cv::COLOR_BGR2GRAY);

    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    zbar::Image zbar_image(mimage.cols, mimage.rows, "Y800", gray.data, mimage.cols * mimage.rows);

    scanner.scan(zbar_image);
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
    {
        RCLCPP_INFO(this->get_logger(),"Mine detected");
    }

   
}
