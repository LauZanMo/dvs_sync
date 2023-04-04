#include "evk4_hd_com.h"

#include <dvs_msgs/EventArray.h>

#include <metavision/hal/facilities/i_roi.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <tbb/parallel_for.h>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace dvs_sync {

Evk4HdCom::Evk4HdCom(const std::string &config_file)
    : nh_("~") {
    // 加载配置文件
    YAML::Node config = YAML::LoadFile(config_file);
    camera_label_     = config["camera_label"].as<std::string>();
    bias_file_        = config["bias_file"].as<std::string>();
    pub_wrap_cost_    = config["pub_wrap_cost"].as<bool>();
    use_multithread_  = config["use_multithread"].as<bool>();
    pub_dt_           = 1.0 / config["pub_rate"].as<double>();
    down_sample_      = config["down_sample"].as<int>();

    auto events_topic = config["events_topic"].as<std::string>();
    events_pub_       = nh_.advertise<dvs_msgs::EventArray>(events_topic, 1000);
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

    down_sample_inv_ = 1.0 / down_sample_;
    width_           = geometry.width() / down_sample_;
    height_          = geometry.height() / down_sample_;
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
    stamp_available_ = true;
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

                msg.events.resize(event_buffer_.size());

                ros::Time t0 = ros::Time::now();

                if (use_multithread_) {
                    tbb::parallel_for(tbb::blocked_range<size_t>(0, event_buffer_.size()),
                                      [&](const tbb::blocked_range<size_t> &r) {
                                          for (size_t i = r.begin(); i != r.end(); i++) {
                                              auto &e = msg.events[i];
                                              e.x     = static_cast<uint16_t>(event_buffer_[i].x) * down_sample_inv_;
                                              e.y     = static_cast<uint16_t>(event_buffer_[i].y) * down_sample_inv_;
                                              e.ts.fromSec(event_buffer_[i].t * 1e-6 - time_offset);
                                              e.polarity = static_cast<uint8_t>(event_buffer_[i].p);
                                          }
                                      });
                } else {
                    for (size_t i = 0; i < event_buffer_.size(); i++) {
                        auto &e = msg.events[i];
                        e.x     = static_cast<uint16_t>(event_buffer_[i].x) * down_sample_inv_;
                        e.y     = static_cast<uint16_t>(event_buffer_[i].y) * down_sample_inv_;
                        e.ts.fromSec(event_buffer_[i].t * 1e-6 - time_offset);
                        e.polarity = static_cast<uint8_t>(event_buffer_[i].p);
                    }
                }

                ros::Duration dt = ros::Time::now() - t0;
                if (pub_wrap_cost_) {
                    ROS_INFO_STREAM("pub events cost " << dt.toSec() * 1000 << " ms");
                    ROS_INFO_STREAM("pub events size " << msg.events.size());
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
            // INS-Probe只在上升沿回传时间戳，因此只处理负极性的触发事件
            for (auto it = begin; it != end; it++) {
                if (it->p == 0) {
                    ulock_t lock(mutex_);
                    stamps_.emplace_back(ros::Time::now().toSec(), it->t * 1e-6);
                }
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