#pragma once

#include "evk4_hd_com.h"
#include "ins_probe_com.h"

namespace dvs_sync {

class DvsSync {
public:
    DvsSync(const std::string &config_file);
    ~DvsSync() = default;

    void run();

private:
    InsProbeCom::Ptr ins_probe_com_;
    Evk4HdCom::Ptr   evk4_hd_com_;
};

} // namespace dvs_sync