#pragma once

#include "evk4_hd_com.h"
#include "types.h"

#include <deque>
#include <memory>
#include <ros/ros.h>
#include <serial/serial.h>
#include <vector>
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

class InsProbeCom {
public:
    typedef std::shared_ptr<InsProbeCom> Ptr;

    InsProbeCom(const std::string &config_file, Evk4HdCom::Ptr &evk4_hd_com);
    ~InsProbeCom() = default;

    static Ptr create(const std::string &config_file, Evk4HdCom::Ptr &evk4_hd_com) {
        return std::make_shared<InsProbeCom>(config_file, evk4_hd_com);
    }

    void run();

private:
    void openSerial(serial::Serial &serial, YAML::Node &serial_config);
    void parseSync(const std::string &data);
    bool parseData(const uint8_t &data);

    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;

    serial::Serial data_serial_, sync_serial_;

    uint8_t data_counter_{0}, checksum_{0};
    InsProbeIMU imu_data_;

    double sync_rate_;
    double sync_thresh_;
    std::deque<StampType> stamps_;
    Evk4HdCom::Ptr evk4_hd_com_;

    std::string prefix_ = "INS Probe: ";
};

} // namespace dvs_sync