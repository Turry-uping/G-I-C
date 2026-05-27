#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "gici/evaluation/evaluation_utils.h"
#include "gici/stream/formator.h"

namespace {

void expectNear(double actual, double expected, double eps, const std::string& name)
{
  if (std::fabs(actual - expected) > eps) {
    std::cerr << name << " expected " << expected << " got " << actual << std::endl;
    std::exit(1);
  }
}

void expectTrue(bool value, const std::string& name)
{
  if (!value) {
    std::cerr << name << " expected true" << std::endl;
    std::exit(1);
  }
}

std::string readFile(const std::string& path)
{
  std::ifstream input(path.c_str());
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

}  // namespace

int main()
{
  std::vector<gici::EvaluationTruthSample> truth;
  expectTrue(
      gici::loadInertialExplorerTruth("tools/evaluation/fixtures/ie_truth_sample.txt", truth),
      "loadInertialExplorerTruth");
  if (truth.size() != 3) {
    std::cerr << "truth size expected 3 got " << truth.size() << std::endl;
    return 1;
  }

  expectNear(truth[0].gps_time, 120974.0, 1.0e-9, "gps_time[0]");
  expectNear(truth[0].longitude_deg, 121.0, 1.0e-12, "longitude[0]");
  expectNear(truth[0].latitude_deg, 31.0, 1.0e-12, "latitude[0]");
  expectNear(truth[0].height_m, 10.0, 1.0e-12, "height[0]");
  expectNear(truth[0].heading_deg, 359.0, 1.0e-12, "heading[0]");
  expectNear(truth[1].heading_deg, 1.0, 1.0e-12, "heading[1]");

  gici::EvaluationTruthSample interpolated;
  expectTrue(gici::interpolateTruth(truth, 120974.5, 0, interpolated), "interpolateTruth");
  expectNear(interpolated.gps_time, 120974.5, 1.0e-9, "interpolated gps_time");
  expectNear(interpolated.longitude_deg, 121.000005, 1.0e-12, "interpolated lon");
  expectNear(interpolated.latitude_deg, 31.000010, 1.0e-12, "interpolated lat");
  expectNear(interpolated.height_m, 10.5, 1.0e-12, "interpolated height");
  expectNear(interpolated.heading_deg, 0.0, 1.0e-9, "interpolated heading wrap");

  expectNear(gici::wrapAngleDeg(181.0), -179.0, 1.0e-12, "wrap 181");
  expectNear(gici::wrapAngleDeg(-181.0), 179.0, 1.0e-12, "wrap -181");
  expectNear(gici::angleDiffDeg(1.0, 359.0), 2.0, 1.0e-12, "angle diff across 360");

  Eigen::Vector3d origin_lla(31.0, 121.0, 10.0);
  Eigen::Vector3d truth_lla(31.000010, 121.000005, 10.5);
  Eigen::Vector3d solution_lla(31.000011, 121.000007, 10.8);
  Eigen::Vector3d truth_enu = gici::llaDegToEnuMeters(truth_lla, origin_lla);
  Eigen::Vector3d solution_enu = gici::llaDegToEnuMeters(solution_lla, origin_lla);
  Eigen::Vector3d error_enu = gici::llaDegToEnuMeters(solution_lla, truth_lla);
  expectTrue(std::fabs(truth_enu.x()) > 0.1, "truth east nonzero");
  expectTrue(std::fabs(solution_enu.y()) > 0.1, "solution north nonzero");
  expectTrue(std::fabs(error_enu.z() - 0.3) < 1.0e-9, "up error");

  YAML::Node node;
  node["type"] = "evaluation";
  node["truth_path"] = "tools/evaluation/fixtures/ie_truth_sample.txt";
  node["output_csv"] = "/tmp/gici_evaluation_test.csv";
  node["output_summary"] = "/tmp/gici_evaluation_test_summary.txt";
  node["truth_format"] = "inertial-explorer";
  std::shared_ptr<gici::FormatorBase> formator = gici::makeFormator(node);
  expectTrue(static_cast<bool>(formator), "evaluation formator factory");
  expectTrue(formator->getType() == gici::FormatorType::Evaluation, "evaluation formator type");
  formator.reset();
  const std::string summary = readFile("/tmp/gici_evaluation_test_summary.txt");
  expectTrue(summary.find("translation_error_m") != std::string::npos,
             "summary has translation stats header even with no aligned rows");
  expectTrue(summary.find("skipped_before_truth") != std::string::npos,
             "summary has skip counters");

  std::cout << "evaluation_tests passed" << std::endl;
  return 0;
}
