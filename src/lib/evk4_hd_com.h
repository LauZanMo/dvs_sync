#pragma once

#include "ins_probe_com.h"

#include <atomic>
#include <memory>
#include <metavision/sdk/driver/camera.h>
#include <ros/ros.h>
#include <vector>

namespace dvs_sync {

using Metavision::EventCD;
using Metavision::EventExtTrigger;

class Evk4HdCom {
public:
    typedef std::shared_ptr<Evk4HdCom> Ptr;

    Evk4HdCom(const std::string &config_file, const InsProbeCom::Ptr &ins_probe_com);
    ~Evk4HdCom() = default;

    static Ptr create(const std::string &config_file, const InsProbeCom::Ptr &ins_probe_com) {
        return std::make_shared<Evk4HdCom>(config_file, ins_probe_com);
    }

    void run();

private:
    bool openCamera();

    InsProbeCom::Ptr ins_probe_com_;
    Metavision::Camera camera_;

    std::string camera_label_;
    std::string bias_file_;
    double pub_dt_;
    double sync_thresh_;
    bool pub_t_offset_;

    ros::NodeHandle nh_;
    ros::Publisher events_pub_;

    std::vector<EventCD> event_buffer_, latest_sae_;
    double buffer_start_t_, buffer_end_t_;

    std::deque<StampType> stamps_;
    std::atomic<bool> is_offset_init_{false};
    std::atomic<double> time_offset_{0.0};

    std::string prefix_ = "EVK4 HD: ";
};

} // namespace dvs_sync