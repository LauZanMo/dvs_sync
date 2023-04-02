#include "dvs_sync.h"

#include <thread>

namespace dvs_sync {

DvsSync::DvsSync(const std::string &config_file) {
    evk4_hd_com_   = Evk4HdCom::create(config_file);
    ins_probe_com_ = InsProbeCom::create(config_file, evk4_hd_com_);
}

void DvsSync::run() {
    // 事件相机线程
    std::thread evk4_thread(&Evk4HdCom::run, evk4_hd_com_);

    // insprobe线程
    ins_probe_com_->run();

    // 等待线程结束
    evk4_thread.join();
}

} // namespace dvs_sync