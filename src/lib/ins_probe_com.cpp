#include "ins_probe_com.h"

#include <absl/strings/numbers.h>
#include <absl/strings/str_split.h>
#include <ros/ros.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace dvs_sync {

InsProbeCom::InsProbeCom(const std::string &config_file) {
    // 加载配置文件
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &e) {
        ROS_ERROR_STREAM("Failed to open configuration file");
        return;
    }
    auto port     = config["port"].as<std::string>();
    auto baudrate = config["baudrate"].as<int>();
    auto timeout  = config["timeout"].as<uint32_t>();

    // 打开串口
    try {
        serial_.setPort(port);
        serial_.setBaudrate(baudrate);
        auto _timeout = serial::Timeout::simpleTimeout(timeout);
        serial_.setTimeout(_timeout);
        serial_.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM(prefix_ << "Unable to open Ins-Probe port: " << port);
        return;
    }

    if (serial_.isOpen())
        ROS_INFO_STREAM(prefix_ << "Ins-Probe port initialized");
}

void InsProbeCom::run() {
    while (ros::ok()) {
        if (serial_.waitReadable()) {
            // 解析
            std::string data = serial_.readline();
            parse(data);
            ROS_INFO_STREAM(prefix_ << data);
        }
    }
    serial_.close();
}

void InsProbeCom::parse(const std::string &data) {
    std::vector<absl::string_view> v = absl::StrSplit(data, ',');

    // 检查数据
    if (v.size() < 3)
        return;

    // 解析
    if (v[0] == "$CAM") {
        double stamp;
        absl::SimpleAtod(v[2], &stamp);
        ulock_t lock(mutex_);
        stamps_.emplace_back(ros::Time::now().toSec(), stamp);
        while (stamps_.size() > 5)
            stamps_.pop_front();
    }
}

} // namespace dvs_sync