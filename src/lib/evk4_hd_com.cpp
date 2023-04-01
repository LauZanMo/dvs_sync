#include "evk4_hd_com.h"

#include <dvs_msgs/EventArray.h>
#include <metavision/hal/facilities/i_roi.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace dvs_sync {

bool Equal(const EventCD &a, const EventCD &b) {
    return a.t == b.t && a.p == b.p;
}

Evk4HdCom::Evk4HdCom(const std::string &config_file, const InsProbeCom::Ptr &ins_probe_com)
    : ins_probe_com_(ins_probe_com)
    , nh_("~") {
    // 加载配置文件
    YAML::Node config = YAML::LoadFile(config_file);
    camera_label_     = config["camera_label"].as<std::string>();
    bias_file_        = config["bias_file"].as<std::string>();
    pub_dt_           = 1.0 / config["pub_rate"].as<double>();
    sync_thresh_      = 0.1 / config["sync_rate"].as<double>(); // 同步阈值设置为同步频率的十分之一
    pub_t_offset_     = config["pub_t_offset"].as<bool>();
    down_sample_      = config["down_sample"].as<int>();

    events_pub_ = nh_.advertise<dvs_msgs::EventArray>("events", 1000);
}

void Evk4HdCom::run() {
    // 打开相机
    while (!openCamera()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ROS_INFO_STREAM(prefix_ << "Trying to open camera...");
        if (!ros::ok()) // 外部节点关闭时还未打开相机
            return;
    }

    camera_.add_runtime_error_callback([](const Metavision::CameraException &e) { ROS_WARN_STREAM(e.what()); });

    // 使能外部触发
    camera_.get_device().get_facility<Metavision::I_TriggerIn>()->enable(0);

    // 打印相机信息
    Metavision::CameraConfiguration config = camera_.get_camera_configuration();
    auto &geometry                         = camera_.geometry();
    ROS_INFO_STREAM(prefix_ << "Camera geometry " << geometry.width() << "x" << geometry.height());
    ROS_INFO_STREAM(prefix_ << "Camera serial number: " << config.serial_number);
    latest_sae_.resize(geometry.width() * geometry.height());

    width_  = geometry.width() / down_sample_;
    height_ = geometry.height() / down_sample_;
    // 设置ROI
    if (down_sample_ > 1) {
        std::vector<bool> cols_to_enable(width_, false);
        std::vector<bool> rows_to_enable(height_, false);
        for (int i = 0; i < geometry.width(); i += down_sample_)
            cols_to_enable[i] = true;
        for (int i = 0; i < geometry.height(); i += down_sample_)
            rows_to_enable[i] = true;
        camera_.get_device().get_facility<Metavision::I_ROI>()->set_ROIs(cols_to_enable, rows_to_enable);
    }

    // 相机采集
    camera_.start();

    // 普通事件回调
    camera_.cd().add_callback([this](const EventCD *begin, const EventCD *end) {
        // 有新事件
        if (begin < end) {
            // 记录缓冲区首个事件的时间戳
            if (event_buffer_.empty())
                buffer_start_t_ = begin->t * 1e-6;

            auto inserter = std::back_inserter(event_buffer_);
            std::copy(begin, end, inserter);

            buffer_end_t_ = (end - 1)->t * 1e-6;
        }

        // 缓冲区时间窗长达到阈值
        if (buffer_end_t_ - buffer_start_t_ >= pub_dt_) {
            // 偏移量初始化后才开始发布事件
            if (is_offset_init_) {
                double time_offset = time_offset_;

                dvs_msgs::EventArray msg;
                msg.header.frame_id = camera_label_;
                msg.header.stamp.fromSec(buffer_start_t_ - time_offset);
                msg.width  = width_;
                msg.height = height_;

                msg.events.reserve(event_buffer_.size());

                for (size_t i = 0; i < event_buffer_.size(); i++) {
                    auto &pixel = latest_sae_[event_buffer_[i].x + event_buffer_[i].y * msg.width];
                    if (Equal(pixel, event_buffer_[i]))
                        continue;
                    else
                        pixel = event_buffer_[i];

                    dvs_msgs::Event e;
                    e.x = static_cast<uint16_t>(event_buffer_[i].x) / down_sample_;
                    e.y = static_cast<uint16_t>(event_buffer_[i].y) / down_sample_;
                    e.ts.fromSec(event_buffer_[i].t * 1e-6 - time_offset);
                    e.polarity = static_cast<uint8_t>(event_buffer_[i].p);
                    msg.events.push_back(std::move(e));
                }

                events_pub_.publish(msg);
            }

            event_buffer_.clear();
        }
    });

    // 外部触发事件回调
    camera_.ext_trigger().add_callback([this](const EventExtTrigger *begin, const EventExtTrigger *end) {
        // 有新触发事件
        if (begin < end) {
            // 由于同步信号频率低(1hz)，因此每次处理只会有一个触发事件
            // INS-Probe只在上升沿回传时间戳，因此只处理负极性的触发事件
            auto back = end - 1;
            if (back->p == 0) {
                stamps_.emplace_back(ros::Time::now().toSec(), back->t * 1e-6);

                // 延迟初始化和计算同步偏移量
                if (stamps_.size() > 2)
                    stamps_.pop_front();
                else if (stamps_.size() < 2)
                    return;

                if (!ins_probe_com_->stampAvailable())
                    return;

                // 计算同步偏移量
                auto ins_stamps     = ins_probe_com_->stamps();
                auto &trigger_stamp = stamps_.front();
                for (auto ins_stamp = ins_stamps.rbegin(); ins_stamp != ins_stamps.rend(); ins_stamp++) {
                    // 搜索本地时间最近的INS-Probe时间戳
                    if (std::fabs(trigger_stamp.first - ins_stamp->first) < sync_thresh_) {
                        time_offset_ = trigger_stamp.second - ins_stamp->second;
                        if (pub_t_offset_) {
                            // clang-format off
                            ROS_INFO_STREAM("Time offset: " << std::to_string(time_offset_));
                            ROS_INFO_STREAM("Trigger ROS time: " << std::to_string(trigger_stamp.first));
                            ROS_INFO_STREAM("INS ROS time: " << std::to_string(ins_stamp->first));
                            ROS_INFO_STREAM("Trigger stamp: " << std::to_string(trigger_stamp.second));
                            ROS_INFO_STREAM("INS stamp: " << std::to_string(ins_stamp->second));
                            // clang-format on
                        }
                        break;
                    }
                }

                if (!is_offset_init_)
                    is_offset_init_ = true;
            }
        }
    });

    // 主循环
    ros::Rate loop_rate(5);
    while (ros::ok()) {
        loop_rate.sleep();
    }

    camera_.stop();
}

bool Evk4HdCom::openCamera() {
    try {
        camera_ = Metavision::Camera::from_first_available();

        if (!bias_file_.empty()) {
            ROS_INFO_STREAM("Loading bias file: " << bias_file_);
            camera_.biases().set_from_file(bias_file_);
        }

        return true;
    } catch (Metavision::CameraException &e) {
        ROS_WARN_STREAM(e.what());
    }
    return false;
}

} // namespace dvs_sync