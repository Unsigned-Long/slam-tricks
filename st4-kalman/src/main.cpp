#include "artwork/csv/csv.h"
#include "pose_simulation.h"
#include "solver.h"

int main(int argc, char const *argv[]) {
  auto data = ns_st4::simulation(30.0f);
  auto writerTruth = ns_csv::CSVWriter::create("../output/truth.csv");
  auto writerObs = ns_csv::CSVWriter::create("../output/obs.csv");

  writerTruth->writeLine(',', "# fps: " + std::to_string(ns_st4::PoseItem::fps));
  writerTruth->writeLine(',', "# x", "y", "z", "qx", "qy", "qz", "qw");

  writerObs->writeLine(',', "# fps: " + std::to_string(ns_st4::PoseItem::fps));
  writerObs->writeLine(',', "# x", "y", "z", "qx", "qy", "qz", "qw");

  for (const auto &elem : data) {
    const auto &truth = elem.truth();
    const auto &truthTranslation = truth.translation();
    const auto &truthQuat = truth.unit_quaternion();
    writerTruth->writeLine(',', truthTranslation(0), truthTranslation(1), truthTranslation(2),
                           truthQuat.x(), truthQuat.y(), truthQuat.z(), truthQuat.w());

    const auto &obs = elem.obs();
    const auto &obsTranslation = obs.translation();
    const auto &obsQuat = obs.unit_quaternion();
    writerObs->writeLine(',', obsTranslation(0), obsTranslation(1), obsTranslation(2),
                         obsQuat.x(), obsQuat.y(), obsQuat.z(), obsQuat.w());
  }
  ns_st4::visualization(data);

  std::vector<Sophus::SE3f> truth(data.size()), esti(data.size());
  for (int i = 0; i != data.size(); ++i) {
    truth.at(i) = data.at(i).truth();
    esti.at(i) = data.at(i).obs();
  }

  std::cout << "ATS: " << ns_st4::absTrajectoryError(truth, esti) << std::endl;
  return 0;
}
