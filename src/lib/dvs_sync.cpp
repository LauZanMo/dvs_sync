#include "dvs_sync.h"

#include <thread>

namespace dvs_sync {

DvsSync::DvsSync(const std::string &config_file) {
    ins_probe_com_ = InsProbeCom::create(config_file);
    evk4_hd_com_   = Evk4HdCom::create(config_file, ins_probe_com_);
}

void DvsSync::run() {
    std::thread evk4_thread(&Evk4HdCom::run, evk4_hd_com_);
    ins_probe_com_->run();
}

} // namespace dvs_sync