#include "gici/stream/evaluation_formator.h"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <glog/logging.h>

#include "gici/evaluation/evaluation_utils.h"
#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"

namespace gici {

namespace {

#define LOAD_REQUIRED_STRING(node, key, target) \
  if (!option_tools::safeGet(node, key, &target)) { \
    LOG(FATAL) << "EvaluationFormator: unable to load required option " << key; \
  }

double safeStd(double variance)
{
  return variance > 0.0 ? std::sqrt(variance) : 0.0;
}

double utcTimestampToGpsTow(double timestamp)
{
  int week = 0;
  const gtime_t utc_time = gnss_common::doubleToGtime(timestamp);
  const gtime_t gps_time = utc2gpst(utc_time);
  return time2gpst(gps_time, &week);
}

}  // namespace

EvaluationFormator::EvaluationFormator(const Option& option)
{
  type_ = FormatorType::Evaluation;
  option_ = option;
  initialize();
}

EvaluationFormator::EvaluationFormator(const YAML::Node& node)
{
  type_ = FormatorType::Evaluation;
  LOAD_REQUIRED_STRING(node, "truth_path", option_.truth_path);
  LOAD_REQUIRED_STRING(node, "output_csv", option_.output_csv);
  LOAD_REQUIRED_STRING(node, "output_summary", option_.output_summary);
  option_tools::safeGet(node, "truth_format", &option_.truth_format);
  initialize();
}

EvaluationFormator::~EvaluationFormator()
{
  writeSummary();
}

int EvaluationFormator::decode(
    const uint8_t*, int,
    std::vector<std::shared_ptr<DataCluster>>&)
{
  LOG(ERROR) << "Evaluation decoding not supported!";
  return 0;
}

int EvaluationFormator::encode(const std::shared_ptr<DataCluster>& data, uint8_t*)
{
  if (!data || !data->solution) return 0;
  EvaluationRow row;
  if (buildRow(*data->solution, row)) {
    appendRow(row);
  }
  return 0;
}

void EvaluationFormator::initialize()
{
  if (option_.truth_format != "inertial-explorer") {
    LOG(FATAL) << "Unsupported truth_format: " << option_.truth_format;
  }
  if (!loadInertialExplorerTruth(option_.truth_path, truth_) || truth_.size() < 2) {
    LOG(FATAL) << "Unable to load enough truth samples from " << option_.truth_path;
  }
  csv_.open(option_.output_csv.c_str());
  if (!csv_.is_open()) {
    LOG(FATAL) << "Unable to open evaluation CSV " << option_.output_csv;
  }
  csv_ << std::setprecision(12);
}

void EvaluationFormator::writeHeader()
{
  if (wrote_header_) return;
  csv_ << "timestamp_gpst,truth_gpst,time_offset_s,elapsed_s,"
       << "solution_lon_deg,solution_lat_deg,solution_height_m,"
       << "solution_roll_deg,solution_pitch_deg,solution_yaw_deg,"
       << "truth_lon_deg,truth_lat_deg,truth_height_m,"
       << "truth_heading_deg,truth_pitch_deg,truth_roll_deg,"
       << "solution_east_m,solution_north_m,solution_up_m,"
       << "truth_east_m,truth_north_m,truth_up_m,"
       << "east_error_m,north_error_m,up_error_m,horizontal_error_m,translation_error_m,"
       << "roll_error_deg,pitch_error_deg,yaw_error_deg,attitude_error_deg,"
       << "velocity_east_error_mps,velocity_north_error_mps,velocity_up_error_mps,"
       << "velocity_horizontal_error_mps,velocity_3d_error_mps,"
       << "solution_status,num_satellites,differential_age_s,"
       << "std_east_m,std_north_m,std_up_m,std_roll_deg,std_pitch_deg,std_yaw_deg,"
       << "std_velocity_east_mps,std_velocity_north_mps,std_velocity_up_mps\n";
  wrote_header_ = true;
}

bool EvaluationFormator::buildRow(const Solution& solution, EvaluationRow& row)
{
  if (!solution.coordinate || solution.status == GnssSolutionStatus::None) {
    ++skips_.invalid_solution;
    return false;
  }
  const double solution_gps_tow = utcTimestampToGpsTow(solution.timestamp);
  if (solution_gps_tow < truth_.front().gps_time) {
    ++skips_.before_truth;
    return false;
  }
  if (solution_gps_tow > truth_.back().gps_time) {
    ++skips_.after_truth;
    return false;
  }

  EvaluationTruthSample truth;
  if (!interpolateTruth(truth_, solution_gps_tow, truth_index_, truth)) {
    ++skips_.interpolation_failure;
    return false;
  }
  while (truth_index_ + 1 < truth_.size() && truth_[truth_index_ + 1].gps_time < solution_gps_tow) {
    ++truth_index_;
  }

  Eigen::Vector3d solution_enu = solution.pose.getPosition();
  Eigen::Vector3d solution_lla_rad = solution.coordinate->convert(solution_enu, GeoType::ENU, GeoType::LLA);
  Eigen::Vector3d solution_lla_deg = solution_lla_rad;
  solution_lla_deg.x() *= R2D;
  solution_lla_deg.y() *= R2D;

  Eigen::Vector3d truth_lla_deg(truth.latitude_deg, truth.longitude_deg, truth.height_m);
  if (!has_origin_) {
    origin_lla_deg_ = truth_lla_deg;
    first_timestamp_ = solution_gps_tow;
    has_origin_ = true;
  }

  Eigen::Vector3d solution_local = llaDegToEnuMeters(solution_lla_deg, origin_lla_deg_);
  Eigen::Vector3d truth_local = llaDegToEnuMeters(truth_lla_deg, origin_lla_deg_);
  Eigen::Vector3d error_enu = llaDegToEnuMeters(solution_lla_deg, truth_lla_deg);
  Eigen::Vector3d rpy = quaternionToEulerAngle(solution.pose.getEigenQuaternion()) * R2D;
  Eigen::Vector3d velocity_error = solution.speed_and_bias.head<3>() - truth.velocity_enu_mps;

  row.timestamp_gpst = solution_gps_tow;
  row.truth_gpst = truth.gps_time;
  row.time_offset_s = solution_gps_tow - truth.gps_time;
  row.elapsed_s = solution_gps_tow - first_timestamp_;
  row.solution_lon_deg = solution_lla_deg.y();
  row.solution_lat_deg = solution_lla_deg.x();
  row.solution_height_m = solution_lla_deg.z();
  row.solution_roll_deg = rpy.x();
  row.solution_pitch_deg = rpy.y();
  row.solution_yaw_deg = rpy.z();
  row.truth_lon_deg = truth.longitude_deg;
  row.truth_lat_deg = truth.latitude_deg;
  row.truth_height_m = truth.height_m;
  row.truth_heading_deg = truth.heading_deg;
  row.truth_pitch_deg = truth.pitch_deg;
  row.truth_roll_deg = truth.roll_deg;
  row.solution_east_m = solution_local.x();
  row.solution_north_m = solution_local.y();
  row.solution_up_m = solution_local.z();
  row.truth_east_m = truth_local.x();
  row.truth_north_m = truth_local.y();
  row.truth_up_m = truth_local.z();
  row.east_error_m = error_enu.x();
  row.north_error_m = error_enu.y();
  row.up_error_m = error_enu.z();
  row.horizontal_error_m = std::hypot(row.east_error_m, row.north_error_m);
  row.translation_error_m = error_enu.norm();
  row.roll_error_deg = angleDiffDeg(row.solution_roll_deg, row.truth_roll_deg);
  row.pitch_error_deg = angleDiffDeg(row.solution_pitch_deg, row.truth_pitch_deg);
  row.yaw_error_deg = angleDiffDeg(row.solution_yaw_deg, row.truth_heading_deg);
  row.attitude_error_deg = std::sqrt(row.roll_error_deg * row.roll_error_deg
      + row.pitch_error_deg * row.pitch_error_deg
      + row.yaw_error_deg * row.yaw_error_deg);
  row.velocity_east_error_mps = velocity_error.x();
  row.velocity_north_error_mps = velocity_error.y();
  row.velocity_up_error_mps = velocity_error.z();
  row.velocity_horizontal_error_mps = std::hypot(velocity_error.x(), velocity_error.y());
  row.velocity_3d_error_mps = velocity_error.norm();
  row.solution_status = solutionStatusToString(solution.status);
  row.num_satellites = solution.num_satellites;
  row.differential_age_s = solution.differential_age;
  row.std_east_m = safeStd(solution.covariance(0, 0));
  row.std_north_m = safeStd(solution.covariance(1, 1));
  row.std_up_m = safeStd(solution.covariance(2, 2));
  row.std_roll_deg = safeStd(solution.covariance(3, 3)) * R2D;
  row.std_pitch_deg = safeStd(solution.covariance(4, 4)) * R2D;
  row.std_yaw_deg = safeStd(solution.covariance(5, 5)) * R2D;
  row.std_velocity_east_mps = safeStd(solution.covariance(6, 6));
  row.std_velocity_north_mps = safeStd(solution.covariance(7, 7));
  row.std_velocity_up_mps = safeStd(solution.covariance(8, 8));
  return true;
}

void EvaluationFormator::appendRow(const EvaluationRow& row)
{
  writeHeader();
  rows_.push_back(row);
  csv_ << row.timestamp_gpst << "," << row.truth_gpst << "," << row.time_offset_s << "," << row.elapsed_s << ","
       << row.solution_lon_deg << "," << row.solution_lat_deg << "," << row.solution_height_m << ","
       << row.solution_roll_deg << "," << row.solution_pitch_deg << "," << row.solution_yaw_deg << ","
       << row.truth_lon_deg << "," << row.truth_lat_deg << "," << row.truth_height_m << ","
       << row.truth_heading_deg << "," << row.truth_pitch_deg << "," << row.truth_roll_deg << ","
       << row.solution_east_m << "," << row.solution_north_m << "," << row.solution_up_m << ","
       << row.truth_east_m << "," << row.truth_north_m << "," << row.truth_up_m << ","
       << row.east_error_m << "," << row.north_error_m << "," << row.up_error_m << ","
       << row.horizontal_error_m << "," << row.translation_error_m << ","
       << row.roll_error_deg << "," << row.pitch_error_deg << "," << row.yaw_error_deg << "," << row.attitude_error_deg << ","
       << row.velocity_east_error_mps << "," << row.velocity_north_error_mps << "," << row.velocity_up_error_mps << ","
       << row.velocity_horizontal_error_mps << "," << row.velocity_3d_error_mps << ","
       << row.solution_status << "," << row.num_satellites << "," << row.differential_age_s << ","
       << row.std_east_m << "," << row.std_north_m << "," << row.std_up_m << ","
       << row.std_roll_deg << "," << row.std_pitch_deg << "," << row.std_yaw_deg << ","
       << row.std_velocity_east_mps << "," << row.std_velocity_north_mps << "," << row.std_velocity_up_mps << "\n";
  csv_.flush();
  if (rows_.size() % 100 == 0) {
    writeSummary();
  }
}

void EvaluationFormator::writeSummary()
{
  wrote_summary_ = true;

  std::ofstream output(option_.output_summary.c_str());
  if (!output.is_open()) {
    LOG(ERROR) << "Unable to open evaluation summary " << option_.output_summary;
    return;
  }

  output << "GICI evaluation summary\n";
  output << "aligned_samples: " << rows_.size() << "\n";
  if (!rows_.empty()) {
    output << "timestamp_gpst_start: " << rows_.front().timestamp_gpst << "\n";
    output << "timestamp_gpst_end: " << rows_.back().timestamp_gpst << "\n";
    output << "duration_s: " << rows_.back().elapsed_s << "\n";
  }
  output << "skipped_before_truth: " << skips_.before_truth << "\n";
  output << "skipped_after_truth: " << skips_.after_truth << "\n";
  output << "skipped_invalid_solution: " << skips_.invalid_solution << "\n";
  output << "skipped_interpolation_failure: " << skips_.interpolation_failure << "\n\n";

  std::vector<double> east, north, up, horizontal, translation;
  std::vector<double> roll, pitch, yaw, attitude;
  std::vector<double> velocity_horizontal, velocity_3d;
  for (const EvaluationRow& row : rows_) {
    east.push_back(row.east_error_m);
    north.push_back(row.north_error_m);
    up.push_back(row.up_error_m);
    horizontal.push_back(row.horizontal_error_m);
    translation.push_back(row.translation_error_m);
    roll.push_back(row.roll_error_deg);
    pitch.push_back(row.pitch_error_deg);
    yaw.push_back(row.yaw_error_deg);
    attitude.push_back(row.attitude_error_deg);
    velocity_horizontal.push_back(row.velocity_horizontal_error_mps);
    velocity_3d.push_back(row.velocity_3d_error_mps);
  }
  writeMetricStats(output, "east_error_m", east);
  writeMetricStats(output, "north_error_m", north);
  writeMetricStats(output, "up_error_m", up);
  writeMetricStats(output, "horizontal_error_m", horizontal);
  writeMetricStats(output, "translation_error_m", translation);
  writeMetricStats(output, "roll_error_deg", roll);
  writeMetricStats(output, "pitch_error_deg", pitch);
  writeMetricStats(output, "yaw_error_deg", yaw);
  writeMetricStats(output, "attitude_error_deg", attitude);
  writeMetricStats(output, "velocity_horizontal_error_mps", velocity_horizontal);
  writeMetricStats(output, "velocity_3d_error_mps", velocity_3d);
  writeStatusGroups(output);
}

void EvaluationFormator::writeMetricStats(
    std::ofstream& output,
    const std::string& name,
    const std::vector<double>& values)
{
  const EvaluationStats stats = computeStats(values);
  output << name
         << ": count=" << stats.count
         << ", mean=" << stats.mean
         << ", rms=" << stats.rms
         << ", stddev=" << stats.stddev
         << ", min=" << stats.min
         << ", max=" << stats.max
         << ", max_abs=" << stats.max_abs
         << ", p50=" << stats.p50
         << ", p95=" << stats.p95
         << ", p99=" << stats.p99 << "\n";
}

void EvaluationFormator::writeStatusGroups(std::ofstream& output)
{
  std::map<std::string, std::vector<double>> translation_by_status;
  std::map<std::string, std::vector<double>> horizontal_by_status;
  for (const EvaluationRow& row : rows_) {
    translation_by_status[row.solution_status].push_back(row.translation_error_m);
    horizontal_by_status[row.solution_status].push_back(row.horizontal_error_m);
  }
  output << "\nBy solution_status:\n";
  for (const auto& item : translation_by_status) {
    const EvaluationStats translation = computeStats(item.second);
    const EvaluationStats horizontal = computeStats(horizontal_by_status[item.first]);
    output << item.first
           << ": samples=" << translation.count
           << ", translation_rms=" << translation.rms
           << ", horizontal_rms=" << horizontal.rms << "\n";
  }
}

}  // namespace gici
