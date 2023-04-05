#pragma once

#include "types.h"

#include <atomic>
#include <deque>
#include <memory>
#include <metavision/sdk/driver/camera.h>
#include <mutex>
#include <ros/ros.h>
#include <vector>

namespace dvs_sync {

using Metavision::EventCD;
using Metavision::EventExtTrigger;

class Evk4HdCom {
public:
    using mutex_t = std::mutex;
    using ulock_t = std::unique_lock<mutex_t>;

    typedef std::shared_ptr<Evk4HdCom> Ptr;

    Evk4HdCom(const std::string &config_file);
    ~Evk4HdCom() = default;

    static Ptr create(const std::string &config_file) {
        return std::make_shared<Evk4HdCom>(config_file);
    }

    void run();

    bool stampAvailable() {
        return stamp_available_;
    }

    std::deque<StampType> stamps() {
        ulock_t lock(mutex_);
        return stamps_;
    }

    void eraseStamps(const size_t &idx) {
        ulock_t lock(mutex_);
        for (size_t i = 0; i < idx; i++)
            stamps_.pop_front();
    }

    bool isOffsetInit() {
        return is_offset_init_;
    }

    void updateOffset(const double &offset) {
        is_offset_init_ = true;
        if (time_offset_ > 0.00001 && std::abs(offset - time_offset_) > 0.0001)
            // offset变化过大，不用于更新
            ROS_WARN_STREAM(prefix_ << "Time offset is not stable!");
        else
            time_offset_ = offset;
    }

private:
    bool openCamera();

    Metavision::Camera camera_;

    std::string camera_label_;
    std::string bias_file_;
    bool enable_event_rate_control_;
    uint32_t event_rate_;
    bool pub_wrap_cost_;
    bool use_multithread_;
    double pub_dt_;
    int down_sample_;
    double down_sample_inv_;
    int width_, height_;

    ros::NodeHandle nh_;
    ros::Publisher events_pub_;
    ros::Publisher events_size_pub_;

    std::vector<EventCD> event_buffer_;
    double buffer_start_t_, buffer_end_t_;

    std::mutex mutex_;
    std::atomic<bool> stamp_available_{false};
    std::atomic<bool> is_offset_init_{false};
    std::deque<StampType> stamps_;
    std::atomic<double> time_offset_{0.0};

    std::string prefix_ = "EVK4 HD: ";
};

} // namespace dvs_sync