#include <dvs_msgs/EventArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

#include <absl/strings/str_format.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std;

void fixImuLost(double start, double end, string imu_topic, rosbag::Bag &bag, ifstream &fp);

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rosbag_imu_fix_node");
    ros::NodeHandle nh("~");

    auto config_file = nh.param<string>("config_file", "");
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &e) {
        ROS_ERROR_STREAM("Failed to open configuration file");
        return 1;
    }

    // 数据文件
    auto raw_bag_file = config["raw_bag_file"].as<string>();
    auto fix_bag_file = config["fix_bag_file"].as<string>();
    auto imu_file     = config["imu_file"].as<string>();

    rosbag::Bag raw_bag(raw_bag_file, rosbag::bagmode::Read);
    rosbag::Bag fix_bag(fix_bag_file, rosbag::bagmode::Write);
    ifstream imufp(imu_file);

    // 话题
    auto events_topic = config["events_topic"].as<string>();
    auto imu_topic    = config["imu_topic"].as<string>();

    vector<string> topics;
    topics.push_back(events_topic);
    topics.push_back(imu_topic);
    rosbag::View view(raw_bag, rosbag::TopicQuery(topics));

    double last_imu_stamp     = 0.0;
    double imu_time_bias      = 0.0;
    double raw_last_imu_stamp = 0.0;

    for (auto &msg : view) {
        auto msg_topic = msg.getTopic();
        if (msg_topic == events_topic) {
            auto events_msg = msg.instantiate<dvs_msgs::EventArray>();
            if (events_msg) {
                // 事件数据正常写入
                fix_bag.write(events_topic, events_msg->header.stamp, events_msg);
            }
        } else if (msg_topic == imu_topic) {
            auto imu_msg = msg.instantiate<sensor_msgs::Imu>();
            if (imu_msg) {
                double weeksec = imu_msg->header.stamp.toSec();
                // 检查是否有imu数据丢失，丢失则补充数据
                if (last_imu_stamp != 0.0) {
                    double dt = weeksec - raw_last_imu_stamp;
                    // 时间同步异常
                    if (dt > 0.1) {
                        imu_time_bias = dt - 0.004974;
                        ROS_ERROR_STREAM(absl::StrFormat("Wrong IMU sync at %0.3lf with dt %0.3lf", weeksec, dt));
                    } else if (dt < 0.003) {
                        imu_time_bias = 0;
                        ROS_ERROR_STREAM(absl::StrFormat("End IMU sync at %0.3lf with dt %0.3lf", weeksec, dt));
                    }

                    weeksec -= imu_time_bias;
                    dt = weeksec - last_imu_stamp;
                    if (dt > 0.008) {
                        ROS_WARN_STREAM(absl::StrFormat("Lost imu from %0.3lf with dt %0.3lf", weeksec, dt));
                        fixImuLost(last_imu_stamp, weeksec, imu_topic, fix_bag, imufp);
                    }
                }
                raw_last_imu_stamp = imu_msg->header.stamp.toSec();
                last_imu_stamp     = weeksec;

                // IMU数据正常写入
                fix_bag.write(imu_topic, imu_msg->header.stamp, imu_msg);
            }
        }
    }

    raw_bag.close();
    fix_bag.close();
    imufp.close();
    ROS_INFO_STREAM("Bag fix done!");

    return 0;
}

void fixImuLost(double start, double end, string imu_topic, rosbag::Bag &bag, ifstream &fp) {
    double imudata[7];
    double last_time, dt;
    last_time = start;

    // 同步到初始时间，不包括
    do {
        fp.read((char *) imudata, sizeof(double) * 7);
        if (fp.eof()) {
            ROS_FATAL_STREAM("Lost data in binary file!");
            break;
        }
    } while (fabs(imudata[0] - start) > 0.0001);

    fp.read((char *) imudata, sizeof(double) * 7);

    // 结束到结束时间，不包括
    while (fabs(imudata[0] - end) > 0.0001) {
        dt        = imudata[0] - last_time;
        last_time = imudata[0];
        if (fabs(dt - 0.005) > 0.0001) {
            dt = 0.004974;
        }

        ROS_WARN_STREAM(absl::StrFormat("Add new IMU at %0.3lf", imudata[0]));

        auto ros_imu = sensor_msgs::ImuPtr(new sensor_msgs::Imu);

        ros_imu->header.stamp.fromSec(imudata[0]);
        ros_imu->header.frame_id = "insprobe";

        ros_imu->angular_velocity.x = imudata[1] / dt;
        ros_imu->angular_velocity.y = imudata[2] / dt;
        ros_imu->angular_velocity.z = imudata[3] / dt;

        ros_imu->linear_acceleration.x = imudata[4] / dt;
        ros_imu->linear_acceleration.y = imudata[5] / dt;
        ros_imu->linear_acceleration.z = imudata[6] / dt;

        bag.write(imu_topic, ros_imu->header.stamp, ros_imu);

        fp.read((char *) imudata, sizeof(double) * 7);
        if (fp.eof()) {
            break;
        }
    }
}