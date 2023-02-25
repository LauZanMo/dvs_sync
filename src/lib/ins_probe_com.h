#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <serial/serial.h>

namespace dvs_sync {

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
    void parse(const std::string &data);

    serial::Serial        serial_;
    std::deque<StampType> stamps_;
    mutex_t               mutex_;

    std::string prefix_ = "INS Probe: ";
};

} // namespace dvs_sync