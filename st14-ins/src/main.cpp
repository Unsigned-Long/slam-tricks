#include "ins.h"
#include "artwork/logger/logger.h"
#include "chrono"
#include "thread"

double curTimeStamp() {
    return std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
}

int main() {

    ns_ins::INSSolver solver(WGS1984, curTimeStamp(),
                             Eigen::Quaterniond::Identity(),
                             Position(0.0, 0.0, 0.0),
                             Velocity(0.0, 0.0, 0.0));

    while (true) {
        solver.update(Eigen::Vector3d::Zero(),
                      Eigen::Vector3d::Zero(),
                      curTimeStamp());
        LOG_VAR(solver.getAttBody2NavK());
        LOG_VAR(solver.getPositionK());
        LOG_VAR(solver.getVelocityNavK());
        LOG_ENDL();

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}