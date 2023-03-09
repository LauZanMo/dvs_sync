#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <serial/serial.h>
#include <yaml-cpp/yaml.h>

namespace dvs_sync {

#pragma pack(push, 1)
struct InsProbeIMU {
    uint16_t week;      ///< GPS周
    double week_second; ///< GPS周内秒
    float gyro_x;       ///< 陀螺x
    float gyro_y;       ///< 陀螺y
    float gyro_z;       ///< 陀螺z
    float acc_x;        ///< 加表x
    float acc_y;        ///< 加表y
    float acc_z;        ///< 加表z
    float reserved1;    ///< 预留1
    int16_t odom;       ///< 里程计
    uint8_t reserved2;  ///< 预留2
    uint8_t reserved3;  ///< 预留3
};
#pragma pack(pop)
static const uint8_t packet_ID[2] = {0x55, 0xAA};

using StampType = std::pair<double, double>; ///< first: 本地时间, second: 传感器时间

class InsProbeCom {
public:
    using mutex_t = std::mutex;
    using ulock_t = std::unique_lock<mutex_t>;

    typedef std::shared_ptr<InsProbeCom> Ptr;

    InsProbeCom(const std::string &config_file);
    ~InsProbeCom() = default;

    static Ptr create(const std::string &config_file) {
        return std::make_shared<InsProbeCom>(config_file);
    }

    void run();

    bool stampAvailable() {
        ulock_t lock(mutex_);
        return !stamps_.empty();
    }

    std::deque<StampType> stamps() {
        ulock_t lock(mutex_);
        return stamps_;
    }

private:
    void openSerial(serial::Serial &serial, YAML::Node &serial_config);
    void parseSync(const std::string &data);
    bool parseData(const uint8_t &data);

    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;

    serial::Serial data_serial_, sync_serial_;

    uint8_t data_counter_{0}, checksum_{0};
    InsProbeIMU imu_data_;

    std::deque<StampType> stamps_;
    mutex_t mutex_;

    std::string prefix_ = "INS Probe: ";
};

} // namespace dvs_sync