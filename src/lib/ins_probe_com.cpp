#include "ins_probe_com.h"

#include <absl/strings/numbers.h>
#include <absl/strings/str_split.h>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <thread>
#include <vector>

namespace dvs_sync {

InsProbeCom::InsProbeCom(const std::string &config_file)
    : nh_("~") {
    // 加载配置文件
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &e) {
        ROS_ERROR_STREAM("Failed to open configuration file");
        return;
    }

    // 打开数据串口
    YAML::Node serial_config = config["data_serial"];
    openSerial(data_serial_, serial_config);
    // 打开同步串口
    serial_config = config["sync_serial"];
    openSerial(sync_serial_, serial_config);

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1000);
}

void InsProbeCom::run() {
    std::thread sync_thread([&]() {
        while (ros::ok()) {
            if (sync_serial_.waitReadable()) {
                // 解析
                std::string data = sync_serial_.readline();
                parseSync(data);
            }
        }
        sync_serial_.close();
    });

    constexpr double deg2rad = M_PI / 180.0;
    uint8_t data;
    double prev_t = 0;
    sensor_msgs::Imu imu_msg;
    while (ros::ok()) {
        if (data_serial_.waitReadable()) {
            // 解析
            data_serial_.read(&data, 1);
            if (parseData(data)) {
                if (prev_t == 0) { // 第一帧
                    prev_t = imu_data_.week_second;
                } else {
                    double dt_inv = 1.0 / (imu_data_.week_second - prev_t);
                    prev_t        = imu_data_.week_second;

                    // 封装成IMU信息发布
                    imu_msg.header.stamp.fromSec(imu_data_.week_second);
                    imu_msg.header.frame_id       = "insprobe";
                    imu_msg.angular_velocity.x    = imu_data_.gyro_x * dt_inv * deg2rad;
                    imu_msg.angular_velocity.y    = imu_data_.gyro_y * dt_inv * deg2rad;
                    imu_msg.angular_velocity.z    = imu_data_.gyro_z * dt_inv * deg2rad;
                    imu_msg.linear_acceleration.x = imu_data_.acc_x * dt_inv;
                    imu_msg.linear_acceleration.y = imu_data_.acc_y * dt_inv;
                    imu_msg.linear_acceleration.z = imu_data_.acc_z * dt_inv;
                    imu_pub_.publish(imu_msg);
                }
            }
        }
    }
    data_serial_.close();
}

void InsProbeCom::openSerial(serial::Serial &serial, YAML::Node &serial_config) {
    auto port     = serial_config["port"].as<std::string>();
    auto baudrate = serial_config["baudrate"].as<int>();
    auto timeout  = serial_config["timeout"].as<uint32_t>();

    while (!boost::filesystem::exists(port)) {
        ROS_INFO_STREAM("Waiting for Ins-Probe port: " << port);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!ros::ok()) // 外部节点关闭时还未打开串口
            return;
    }

    // 打开串口
    try {
        serial.setPort(port);
        serial.setBaudrate(baudrate);
        auto _timeout = serial::Timeout::simpleTimeout(timeout);
        serial.setTimeout(_timeout);
        serial.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM(prefix_ << "Unable to open Ins-Probe port: " << port);
        return;
    }

    if (serial.isOpen())
        ROS_INFO_STREAM(prefix_ << "Ins-Probe port " << port << " initialized");
}

bool InsProbeCom::parseData(const uint8_t &data) {
    if (data_counter_ == 0)
        checksum_ = 0;

    if (data_counter_ < 2) { // 接收包头
        if (data != packet_ID[data_counter_])
            data_counter_ = 0;
        else
            data_counter_++;
    } else if (data_counter_ < 44) { // 接收数据
        ((uint8_t *) &imu_data_)[data_counter_ - 2] = data;
        checksum_ += data;
        data_counter_++;
    } else { // 接收校验和（这里我只用了第一个校验位）
        data_counter_ = 0;
        if (checksum_ == data) // 检验成功
            return true;
    }
    return false;
}

void InsProbeCom::parseSync(const std::string &data) {
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