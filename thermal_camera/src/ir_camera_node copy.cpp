#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <algorithm>

static constexpr int FIRE_THRESH_8U = 230;
static constexpr double AREA_MIN_PX = 10.0;
static constexpr int CIRCLE_THICK = 1;

// 🔥 參數：判斷火源需連續出現幾幀，且明顯高於全圖平均
static constexpr float TEMP_DIFF_THRESH = 15.0;     // 高於平均溫度幾度才算火源
static constexpr int   FIRE_HOLD_FRAMES = 5;         // 持續幾幀以上才觸發發佈

static void drawFireCircle(const cv::Mat &gray8, cv::Mat &canvas) {
    cv::Mat mask;
    cv::threshold(gray8, mask, FIRE_THRESH_8U, 255, cv::THRESH_BINARY);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5}), {-1,-1}, 2);

    std::vector<std::vector<cv::Point>> cs;
    cv::findContours(mask, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (cs.empty()) return;

    auto it = std::max_element(cs.begin(), cs.end(),
              [](auto &a, auto &b){ return cv::contourArea(a) < cv::contourArea(b); });
    if (cv::contourArea(*it) < AREA_MIN_PX) return;

    cv::Rect box = cv::boundingRect(*it);
    cv::rectangle(canvas, box, {0,255,0}, CIRCLE_THICK);
}

extern "C" {
    #include <guideusb2livestream.h>
    #include <stdio.h>
    #include <sys/types.h>
    #include <unistd.h>
    #include <malloc.h>
    #include <string.h>
    #include "sys/time.h"
    #include "time.h"
    #include <libmemcached/memcached.h>
}

class IRCameraNode;
IRCameraNode *g_node_instance = nullptr;
int frameCallbackStatic(guide_usb_frame_data_t *pVideoData);

class IRCameraNode : public rclcpp::Node {
public:
    IRCameraNode() : Node("ir_camera"), fire_candidate_count_(0) {
        g_node_instance = this;

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ir_camera/image", 10);
        max_temp_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thermal_max_temp", 10);

        guide_usb_setloglevel(LOG_INFO);
        int ret = guide_usb_initial();
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Initialization failed: %d", ret);
            rclcpp::shutdown(); return;
        }

        ret = guide_usb_opencommandcontrol(serialCallback);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open command control: %d", ret);
            rclcpp::shutdown(); return;
        }

        guide_measure_loadcurve();
        mDebugParam = (guide_measure_debugparam_t *)malloc(sizeof(guide_measure_debugparam_t));
        if (!mDebugParam) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to allocate mDebugParam");
            rclcpp::shutdown(); return;
        }

        guide_usb_device_info_t deviceInfo;
        deviceInfo.width = 256;
        deviceInfo.height = 192;
        deviceInfo.video_mode = Y16_PARAM_YUV;

        ret = guide_usb_openstream(&deviceInfo, frameCallbackStatic, connectStatusCallback);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open stream: %d", ret);
            rclcpp::shutdown(); return;
        }

        RCLCPP_INFO(this->get_logger(), "📸 熱像串流已啟動");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&IRCameraNode::keepAlive, this));
    }

    ~IRCameraNode() {
        guide_usb_closestream();
        guide_measure_deloadcurve();
        guide_usb_closecommandcontrol();
        guide_usb_exit();
        if (mDebugParam) free(mDebugParam);
    }

    int frameCallback(guide_usb_frame_data_t *pVideoData) {
        if (!pVideoData || !pVideoData->frame_src_data || !mDebugParam)
            return 0;

        int width = pVideoData->frame_width;
        int height = pVideoData->frame_height;

        mDebugParam->exkf = 100; mDebugParam->exb = 0;
        mDebugParam->emiss = 98;
        mDebugParam->transs = 0;
        mDebugParam->reflectTemp = 23.0f;
        mDebugParam->distance = 30.0f;
        mDebugParam->fEnvironmentIncrement = 2500;

        cv::Mat img16(height, width, CV_16UC1, pVideoData->frame_src_data);
        cv::Mat img8;
        img16.convertTo(img8, CV_8UC1, 0.055);
        cv::Mat color_img;
        cv::applyColorMap(img8, color_img, cv::COLORMAP_JET);

        drawFireCircle(img8, color_img);

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_img).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        image_pub_->publish(*msg);

        int max_index = 0;
        uint16_t max_gray_value = 0;
        uint64_t sum = 0;
        for (int i = 0; i < width * height; i++) {
            uint16_t val = pVideoData->frame_src_data[i];
            sum += val;
            if (val > max_gray_value) {
                max_gray_value = val;
                max_index = i;
            }
        }

        int max_x = max_index % width;
        int max_y = max_index / width;
        float max_temp = guide_measure_convertsinglegray2temper(max_gray_value, pVideoData->paramLine, mDebugParam, 1);

        float avg_temp = guide_measure_convertsinglegray2temper(sum / (width * height), pVideoData->paramLine, mDebugParam, 1);

        if ((max_temp - avg_temp) > TEMP_DIFF_THRESH) {
            fire_candidate_count_++;
        } else {
            fire_candidate_count_ = 0;
        }

        if (fire_candidate_count_ >= FIRE_HOLD_FRAMES) {
            std_msgs::msg::Float32MultiArray temp_msg;
            temp_msg.data = {static_cast<float>(max_x), static_cast<float>(max_y), max_temp};
            max_temp_pub_->publish(temp_msg);
            RCLCPP_INFO(this->get_logger(), "📡 發佈🔥 %.2f°C (平均: %.2f°C) 位置: (%d, %d)", max_temp, avg_temp, max_x, max_y);
        } else {
            RCLCPP_INFO(this->get_logger(), "⚠️ 未達火源門檻 %.2f°C (平均: %.2f°C)", max_temp, avg_temp);
        }

        return 1;
    }

    static int serialCallback(guide_usb_serial_data_t *pSerialData) {
        RCLCPP_INFO(rclcpp::get_logger("ir_camera"), "📡 收到序列數據，長度: %d", pSerialData->serial_recv_data_length);
        return 1;
    }

    static int connectStatusCallback(guide_usb_device_status_e deviceStatus) {
        if (deviceStatus == DEVICE_CONNECT_OK) {
            RCLCPP_INFO(rclcpp::get_logger("ir_camera"), "✅ 設備已連接！");
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ir_camera"), "❌ 設備已斷開！");
        }
        return 1;
    }

    void keepAlive() {}

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr max_temp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    guide_measure_debugparam_t *mDebugParam;
    int fire_candidate_count_;
};

int frameCallbackStatic(guide_usb_frame_data_t *pVideoData) {
    return g_node_instance ? g_node_instance->frameCallback(pVideoData) : 0;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRCameraNode>());
    rclcpp::shutdown();
    return 0;
}
