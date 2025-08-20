#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

extern "C" {
    #include "thermal_camera/guideusb2livestream.h"
    #include <stdio.h>
    #include <sys/types.h>
    #include <unistd.h>
    #include <malloc.h>
    #include <string.h>
    #include "sys/time.h"
    #include "time.h"
    #include <libmemcached/memcached.h>
}

// **全域變數**
class IRCameraNode;
IRCameraNode *g_node_instance = nullptr;

// **靜態回調函數**
int frameCallbackStatic(guide_usb_frame_data_t *pVideoData);

class IRCameraNode : public rclcpp::Node
{
public:
    IRCameraNode() : Node("ir_camera")
    {
        g_node_instance = this; 

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/ir_camera/image", 10);
        guide_usb_setloglevel(LOG_INFO);

        int ret = guide_usb_initial();
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed: %d", ret);
            rclcpp::shutdown();
            return;
        }

        ret = guide_usb_opencommandcontrol(serailCallback);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open command control: %d", ret);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "USB command control opened.");

        guide_measure_loadcurve();

        mDebugParam = (guide_measure_debugparam_t *)malloc(sizeof(guide_measure_debugparam_t));
        if (!mDebugParam) {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for mDebugParam.");
            rclcpp::shutdown();
            return;
        }

        guide_usb_device_info_t *deviceInfo = (guide_usb_device_info_t *)malloc(sizeof(guide_usb_device_info_t));
        if (!deviceInfo) {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for deviceInfo.");
            free(mDebugParam);
            rclcpp::shutdown();
            return;
        }

        deviceInfo->width = 256;
        deviceInfo->height = 192;
        deviceInfo->video_mode = Y16_PARAM_YUV;

        ret = guide_usb_openstream(deviceInfo, frameCallbackStatic, connectStatusCallback);
        startTime_ = tick();
        RCLCPP_INFO(this->get_logger(), "Stream started with return code: %d", ret);

        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open stream: %d", ret);
            free(mDebugParam);
            free(deviceInfo);
            rclcpp::shutdown();
            return;
        }

        free(deviceInfo);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&IRCameraNode::timerCallback, this));
    }

    ~IRCameraNode()
    {
        guide_usb_closestream();
        guide_measure_deloadcurve();
        guide_usb_closecommandcontrol();
        guide_usb_exit();

        if (mDebugParam) {
            free(mDebugParam);
            mDebugParam = nullptr;
        }
    }

    int frameCallback(guide_usb_frame_data_t *pVideoData)
    {
        if (!pVideoData || !pVideoData->paramLine)
            return 0;

        int width = pVideoData->frame_width;
        int height = pVideoData->frame_height;

        // **歸一化處理以提升影像清晰度**
        cv::Mat img16(height, width, CV_16UC1, pVideoData->frame_src_data);
        cv::Mat img8;
        cv::normalize(img16, img8, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        // **應用偽彩色**
        cv::Mat color_img;
        cv::applyColorMap(img8, color_img, cv::COLORMAP_INFERNO); 

        // **提升影像解析度**
        cv::Mat resized_img;
        cv::resize(color_img, resized_img, cv::Size(width * 2, height * 2), 0, 0, cv::INTER_CUBIC);

        // **轉換為 ROS2 訊息**
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_img).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        publisher_->publish(*msg);

        return 1;
    }

    static int serailCallback(guide_usb_serial_data_t *pSerialData)
    {
        if (!pSerialData)
            return 0;
        return 1;
    }

    static int connectStatusCallback(guide_usb_device_status_e deviceStatus)
    {
        if (deviceStatus == DEVICE_CONNECT_OK)
            printf("Stream start OK\n");
        else
            printf("Stream end\n");
        return 1;
    }

    void timerCallback()
    {
        usleep(10);
    }

private:
    double tick(void)
    {
        struct timeval t;
        gettimeofday(&t, 0);
        return t.tv_sec + 1E-6 * t.tv_usec;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double startTime_;
    guide_measure_debugparam_t *mDebugParam;
};

int frameCallbackStatic(guide_usb_frame_data_t *pVideoData)
{
    if (g_node_instance) {
        return g_node_instance->frameCallback(pVideoData);
    }
    return 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IRCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
