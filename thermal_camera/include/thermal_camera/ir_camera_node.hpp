#ifndef THERMAL_CAMERA_IR_CAMERA_NODE_HPP
#define THERMAL_CAMERA_IR_CAMERA_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

extern "C" {
    #include <guideusb2livestream.h>
}

class IRCameraNode : public rclcpp::Node
{
public:
    IRCameraNode();
    ~IRCameraNode();

private:
    void process_frame(guide_usb_frame_data_t *pVideoData);
    static int frameCallbackStatic(guide_usb_frame_data_t *pVideoData);
    static int serialCallback(guide_usb_serial_data_t *pSerialData);
    static int connectStatusCallback(guide_usb_device_status_e deviceStatus);
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    guide_measure_debugparam_t *mDebugParam;
};

#endif // THERMAL_CAMERA_IR_CAMERA_NODE_HPP
