#include "gici/evaluation/evaluation_utils.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace gici {

namespace {

bool parseTruthLine(const std::string& line, EvaluationTruthSample& sample)
{
  std::istringstream stream(line);
  int week = 0;
  double utc_time = 0.0;
  if (!(stream >> week >> sample.gps_time >> utc_time >> sample.longitude_deg
        >> sample.latitude_deg >> sample.height_m >> sample.heading_deg
        >> sample.pitch_deg >> sample.roll_deg)) {
    return false;
  }
  return true;
}

double percentileFromSorted(const std::vector<double>& sorted, double percentile)
{
  if (sorted.empty()) return 0.0;
  const double pos = percentile * static_cast<double>(sorted.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(pos));
  const size_t hi = static_cast<size_t>(std::ceil(pos));
  const double ratio = pos - static_cast<double>(lo);
  return sorted[lo] + (sorted[hi] - sorted[lo]) * ratio;
}

}  // namespace

bool loadInertialExplorerTruth(
    const std::string& path,
    std::vector<EvaluationTruthSample>& samples)
{
  samples.clear();
  std::ifstream input(path.c_str());
  if (!input.is_open()) {
    return false;
  }

  std::string line;
  while (std::getline(input, line)) {
    EvaluationTruthSample sample;
    if (!parseTruthLine(line, sample)) {
      continue;
    }
    samples.push_back(sample);
  }

  std::sort(samples.begin(), samples.end(),
            [](const EvaluationTruthSample& lhs, const EvaluationTruthSample& rhs) {
              return lhs.gps_time < rhs.gps_time;
            });
  computeTruthVelocities(samples);
  return !samples.empty();
}

void computeTruthVelocities(std::vector<EvaluationTruthSample>& samples)
{
  if (samples.size() < 2) return;

  for (size_t i = 0; i < samples.size(); ++i) {
    const size_t left = i == 0 ? 0 : i - 1;
    const size_t right = i + 1 >= samples.size() ? samples.size() - 1 : i + 1;
    const double dt = samples[right].gps_time - samples[left].gps_time;
    if (dt <= 0.0) {
      samples[i].velocity_enu_mps.setZero();
      continue;
    }

    const Eigen::Vector3d left_lla(
        samples[left].latitude_deg,
        samples[left].longitude_deg,
        samples[left].height_m);
    const Eigen::Vector3d right_lla(
        samples[right].latitude_deg,
        samples[right].longitude_deg,
        samples[right].height_m);
    samples[i].velocity_enu_mps = llaDegToEnuMeters(right_lla, left_lla) / dt;
  }
}

bool interpolateTruth(
    const std::vector<EvaluationTruthSample>& samples,
    double gps_time,
    size_t start_index,
    EvaluationTruthSample& interpolated)
{
  if (samples.size() < 2) return false;
  size_t index = std::min(start_index, samples.size() - 2);
  while (index + 1 < samples.size() && samples[index + 1].gps_time < gps_time) {
    ++index;
  }
  if (index + 1 >= samples.size()) return false;

  const EvaluationTruthSample& left = samples[index];
  const EvaluationTruthSample& right = samples[index + 1];
  if (gps_time < left.gps_time || gps_time > right.gps_time) {
    return false;
  }

  const double span = right.gps_time - left.gps_time;
  const double ratio = span == 0.0 ? 0.0 : (gps_time - left.gps_time) / span;
  interpolated.gps_time = gps_time;
  interpolated.longitude_deg = left.longitude_deg + (right.longitude_deg - left.longitude_deg) * ratio;
  interpolated.latitude_deg = left.latitude_deg + (right.latitude_deg - left.latitude_deg) * ratio;
  interpolated.height_m = left.height_m + (right.height_m - left.height_m) * ratio;
  interpolated.heading_deg = interpolateAngleDeg(left.heading_deg, right.heading_deg, ratio);
  interpolated.pitch_deg = interpolateAngleDeg(left.pitch_deg, right.pitch_deg, ratio);
  interpolated.roll_deg = interpolateAngleDeg(left.roll_deg, right.roll_deg, ratio);
  interpolated.velocity_enu_mps =
      left.velocity_enu_mps + (right.velocity_enu_mps - left.velocity_enu_mps) * ratio;
  return true;
}

double wrapAngleDeg(double angle_deg)
{
  while (angle_deg >= 180.0) angle_deg -= 360.0;
  while (angle_deg < -180.0) angle_deg += 360.0;
  return angle_deg;
}

double angleDiffDeg(double lhs_deg, double rhs_deg)
{
  return wrapAngleDeg(lhs_deg - rhs_deg);
}

double interpolateAngleDeg(double lhs_deg, double rhs_deg, double ratio)
{
  return wrapAngleDeg(lhs_deg + angleDiffDeg(rhs_deg, lhs_deg) * ratio);
}

Eigen::Vector3d llaDegToEnuMeters(
    const Eigen::Vector3d& lla_deg,
    const Eigen::Vector3d& origin_lla_deg)
{
  const double kWgs84A = 6378137.0;
  const double kWgs84E2 = 6.69437999014e-3;
  const double lat_ref = origin_lla_deg.x() * M_PI / 180.0;
  const double lat = lla_deg.x() * M_PI / 180.0;
  const double lon_ref = origin_lla_deg.y() * M_PI / 180.0;
  const double lon = lla_deg.y() * M_PI / 180.0;
  const double sin_lat = std::sin(lat_ref);
  const double denom = std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  const double n_radius = kWgs84A / denom;
  const double m_radius = kWgs84A * (1.0 - kWgs84E2) / (denom * denom * denom);
  const double east = (lon - lon_ref) * (n_radius + origin_lla_deg.z()) * std::cos(lat_ref);
  const double north = (lat - lat_ref) * (m_radius + origin_lla_deg.z());
  const double up = lla_deg.z() - origin_lla_deg.z();
  return Eigen::Vector3d(east, north, up);
}

std::string solutionStatusToString(GnssSolutionStatus status)
{
  switch (status) {
    case GnssSolutionStatus::Fixed:
      return "fixed";
    case GnssSolutionStatus::Float:
      return "float";
    case GnssSolutionStatus::DGNSS:
      return "dgnss";
    case GnssSolutionStatus::Single:
      return "single";
    case GnssSolutionStatus::DeadReckoning:
      return "dead_reckoning";
    default:
      return "none";
  }
}

EvaluationStats computeStats(const std::vector<double>& values)
{
  EvaluationStats stats;
  stats.count = static_cast<int>(values.size());
  if (values.empty()) return stats;

  stats.min = values.front();
  stats.max = values.front();
  double sum = 0.0;
  double sum_sq = 0.0;
  for (const double value : values) {
    stats.min = std::min(stats.min, value);
    stats.max = std::max(stats.max, value);
    stats.max_abs = std::max(stats.max_abs, std::fabs(value));
    sum += value;
    sum_sq += value * value;
  }
  stats.mean = sum / static_cast<double>(values.size());
  stats.rms = std::sqrt(sum_sq / static_cast<double>(values.size()));

  double variance = 0.0;
  for (const double value : values) {
    const double diff = value - stats.mean;
    variance += diff * diff;
  }
  stats.stddev = std::sqrt(variance / static_cast<double>(values.size()));

  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  stats.p50 = percentileFromSorted(sorted, 0.50);
  stats.p95 = percentileFromSorted(sorted, 0.95);
  stats.p99 = percentileFromSorted(sorted, 0.99);
  return stats;
}

}  // namespace gici
