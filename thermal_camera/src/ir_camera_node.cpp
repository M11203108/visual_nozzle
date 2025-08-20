#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <algorithm>

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

// 🔧 可調參數
// static constexpr int FIRE_THRESH_8U = 230;
static constexpr int FIRE_THRESH_8U = 210;
static constexpr double AREA_MIN_PX = 15.0;
static constexpr int CIRCLE_THICK = 1;
// static constexpr float TEMP_DIFF_THRESH = 15.0;
static constexpr float TEMP_DIFF_THRESH = 5.0;
static constexpr int FIRE_HOLD_FRAMES = 5;

int frameCallbackStatic(guide_usb_frame_data_t *pVideoData);

class IRCameraNode;
IRCameraNode *g_node_instance = nullptr;

static bool drawFireSquare(const cv::Mat &gray8, cv::Mat &canvas, cv::Rect &output_box)
{
    cv::Mat mask;
    cv::threshold(gray8, mask, FIRE_THRESH_8U, 255, cv::THRESH_BINARY);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5}), {-1,-1}, 2);

    std::vector<std::vector<cv::Point>> cs;
    cv::findContours(mask, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (cs.empty()) return false;

    auto it = std::max_element(cs.begin(), cs.end(),
              [](auto &a, auto &b){ return cv::contourArea(a) < cv::contourArea(b); });

    output_box = cv::boundingRect(*it);
    double area = cv::contourArea(*it);

    if (area < AREA_MIN_PX) {
        // ⚠️ 輪廓太小 → 使用固定大小的框
        const int FIXED_W = 5;
        const int FIXED_H = 5;
        int cx = output_box.x + output_box.width / 2;
        int cy = output_box.y + output_box.height / 2;
        output_box = cv::Rect(cx - FIXED_W / 2, cy - FIXED_H / 2, FIXED_W, FIXED_H);
    }

    cv::rectangle(canvas, output_box, {0,255,0}, CIRCLE_THICK);
    return true;
}


// 🌡️ 熱像儀節點
class IRCameraNode : public rclcpp::Node {
public:
    IRCameraNode() : Node("ir_camera"), fire_candidate_count_(0)
    {
        g_node_instance = this;
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ir_camera/image", 10);
        max_temp_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thermal_max_temp", 10);

        guide_usb_setloglevel(LOG_INFO);

        if (guide_usb_initial() < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Initialization failed. Try: sudo chmod -R 777 /dev/bus/usb");
            rclcpp::shutdown(); return;
        }

        if (guide_usb_opencommandcontrol(serialCallback) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open command control");
            rclcpp::shutdown(); return;
        }

        guide_measure_loadcurve();
        mDebugParam = (guide_measure_debugparam_t *)malloc(sizeof(guide_measure_debugparam_t));
        if (!mDebugParam) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to allocate mDebugParam");
            rclcpp::shutdown(); return;
        }

        guide_usb_device_info_t deviceInfo{256, 192, Y16_PARAM_YUV};
        if (guide_usb_openstream(&deviceInfo, frameCallbackStatic, connectStatusCallback) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open stream");
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

    int frameCallback(guide_usb_frame_data_t *pVideoData)
    {
        if (!pVideoData || !pVideoData->frame_src_data || !mDebugParam) return 0;

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

        // 🔲 畫出最大火源框（若有）
        cv::Rect fire_box;
        bool has_box = drawFireSquare(img8, color_img, fire_box);

        // 發佈影像
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_img).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        image_pub_->publish(*msg);

        // 🔍 計算最大溫度與平均溫度
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

        if ((max_temp - avg_temp) > TEMP_DIFF_THRESH && has_box) {
            fire_candidate_count_++;
        } else {
            fire_candidate_count_ = 0;
        }

        if (fire_candidate_count_ >= FIRE_HOLD_FRAMES && has_box) {
            int x1 = fire_box.x;
            int y1 = fire_box.y;
            int x2 = fire_box.x + fire_box.width;
            int y2 = fire_box.y + fire_box.height;

            std_msgs::msg::Float32MultiArray temp_msg;
            temp_msg.data = {
                static_cast<float>(max_x),
                static_cast<float>(max_y),
                max_temp,
                static_cast<float>(x1),
                static_cast<float>(y1),
                static_cast<float>(x2),
                static_cast<float>(y2)
            };
            max_temp_pub_->publish(temp_msg);

            RCLCPP_INFO(this->get_logger(), "📡 發佈🔥 %.2f°C (%d,%d) → 框:(%d,%d)~(%d,%d)",
                max_temp, max_x, max_y, x1, y1, x2, y2);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "⚠️ 未達門檻 🔥 %.2f°C (平均: %.2f°C)", max_temp, avg_temp);
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

int frameCallbackStatic(guide_usb_frame_data_t *pVideoData)
{
    return g_node_instance ? g_node_instance->frameCallback(pVideoData) : 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRCameraNode>());

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}

