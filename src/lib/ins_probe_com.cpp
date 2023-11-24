#include "ins_probe_com.h"

#include <absl/strings/numbers.h>
#include <absl/strings/str_split.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <thread>
#include <vector>

namespace dvs_sync {

InsProbeCom::InsProbeCom(const std::string &config_file, Evk4HdCom::Ptr &evk4_hd_com)
    : nh_("~")
    , evk4_hd_com_(evk4_hd_com) {
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

    auto imu_topic    = config["imu_topic"].as<std::string>();
    auto offset_topic = config["offset_topic"].as<std::string>();
    imu_pub_          = nh_.advertise<sensor_msgs::Imu>(imu_topic, 1000);
    offset_pub_       = nh_.advertise<geometry_msgs::Vector3Stamped>(offset_topic, 1000);

    // 同步频率
    sync_rate_   = config["sync_rate"].as<double>();
    sync_thresh_ = 0.1 / sync_rate_; // 同步阈值设置为同步频率的十分之一
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
    double last_dt_inv = 200.0;
    while (ros::ok()) {
        if (data_serial_.waitReadable()) {
            // 解析
            data_serial_.read(&data, 1);
            if (parseData(data)) {
                if (prev_t == 0) { // 第一帧
                    prev_t = imu_data_.week_second;
                } else {
                    double dt = imu_data_.week_second - prev_t;
                    double dt_inv;
                    if (dt > 0.008 || dt < 0.003) {
                        // 丢数
                        dt_inv = last_dt_inv; // 200Hz
                        ROS_WARN_STREAM("IMU data lost");
                    } else {
                        dt_inv = 1.0 / dt;
                        last_dt_inv = dt_inv;
                    }
                    prev_t = imu_data_.week_second;

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
    } else if (data_counter_ < 45) { // 接收校验和（这里我只用了第一个校验位）
        data_counter_++;
        if (checksum_ == data) // 检验成功
            return true;
    } else { // 忽略第二个校验位
        data_counter_ = 0;
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
        // 相机还未启动，不解析数据
        if (!evk4_hd_com_->stampAvailable())
            return;
        double stamp;
        absl::SimpleAtod(v[2], &stamp);
        stamps_.emplace_back(ros::Time::now().toSec(), stamp);

        // 未初始化offset时，通过本地时间和相机时间进行同步
        if (!evk4_hd_com_->isOffsetInit()) {
            // 等待stamps_中有足够的数据
            if (stamps_.size() < sync_rate_ * 3) // 3秒
                return;

            // 取1秒前的值计算offset
            auto cmp_idx     = stamps_.size() - sync_rate_;
            auto &cmp_stamp  = stamps_[cmp_idx];
            auto evk4_stamps = evk4_hd_com_->stamps();
            for (size_t idx = 0; idx < evk4_stamps.size(); idx++) {
                if (std::abs(evk4_stamps[idx].first - cmp_stamp.first) < sync_thresh_) {
                    double offset = evk4_stamps[idx].second - cmp_stamp.second;
                    evk4_hd_com_->updateOffset(offset);
                    ROS_INFO_STREAM(prefix_ << "EVK4 offset initialized: " << offset);

                    // 两边删除到同步点
                    evk4_hd_com_->eraseStamps(idx);
                    for (size_t i = 0; i < cmp_idx; i++)
                        stamps_.pop_front();

                    break;
                }
            }
        } else {
            // 初始化offset后，通过序列号进行同步
            auto min_size = std::min(stamps_.size(), evk4_hd_com_->stamps().size());
            if (min_size < 2)
                return;

            double inv    = 1.0 / min_size;
            double offset = 0;
            for (size_t i = 0; i < min_size; i++)
                offset += inv * (evk4_hd_com_->stamps()[i].second - stamps_[i].second);
            evk4_hd_com_->updateOffset(offset);

            // 发布offset
            geometry_msgs::Vector3Stamped offset_msg;
            offset_msg.header.stamp.fromSec(stamp);
            offset_msg.vector.x = offset;
            offset_pub_.publish(offset_msg);

            // 删除到同步点-1
            evk4_hd_com_->eraseStamps(min_size - 1);
            for (size_t i = 0; i < min_size - 1; i++)
                stamps_.pop_front();
        }
    }
}

} // namespace dvs_sync