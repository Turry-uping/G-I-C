# Evaluation Output Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a configurable GICI evaluation output path that reads Inertial Explorer truth data, aligns internal `Solution` epochs, writes rich per-epoch error CSV data, and writes aggregate summary metrics.

**Architecture:** Add a focused `EvaluationFormator` to the existing stream/formator/output-tag architecture. Keep truth parsing and metric computation in a small helper module under `gici/evaluation` so NMEA output remains unchanged and visualization can consume the generated CSV in a separate plotting step.

**Tech Stack:** C++14, Eigen, yaml-cpp, glog, RTKLIB constants/helpers already exposed through `gici/utility/rtklib_safe.h`, existing GICI `GeoCoordinate`, `Solution`, and `Transformation` types, Python 3 standard library for visualization migration.

---

## File Structure

Create:

- `include/gici/evaluation/evaluation_types.h`: plain data structures for IE truth samples, evaluation rows, summary stats, and skip counters.
- `include/gici/evaluation/evaluation_utils.h`: declarations for parsing, interpolation, angle wrapping, ENU conversion, velocity derivation, per-row metric computation, and summary statistics.
- `src/evaluation/evaluation_utils.cpp`: implementation of evaluation helpers.
- `include/gici/stream/evaluation_formator.h`: `EvaluationFormator` declaration and YAML options.
- `src/stream/evaluation_formator.cpp`: output formator implementation that owns truth data, output files, CSV writing, and summary writing.
- `tools/evaluation/evaluation_tests.cpp`: lightweight C++ test executable using `CHECK`/`LOG(FATAL)` style assertions.
- `tools/evaluation/fixtures/ie_truth_sample.txt`: small IE-style fixture for parser/interpolation tests.

Modify:

- `include/gici/stream/formator.h`: add `FormatorType::Evaluation`, include the new formator header or forward declaration, and register `MAKE_FORMATOR(EvaluationFormator)`.
- `src/stream/formator.cpp`: map YAML `type: evaluation` to `EvaluationFormator`.
- `src/utility/option.cpp`: convert `"evaluation"` to `FormatorType::Evaluation`.
- `CMakeLists.txt`: add `src/evaluation` to the shared library source list and add an optional `evaluation_tests` executable.
- `option/pseudo_real_time_estimation_RTK_TC.yaml`: add example `fmt_tc_evaluation` formator and output tag.
- `tools/visualization/plot_position_error.py`: add `evaluation.csv` input mode while retaining old NMEA/truth fallback.
- `tools/visualization/README.md`: document the new primary workflow.
- `docs/superpowers/specs/2026-05-26-evaluation-output-design.md`: update only if implementation discovers a necessary scope correction.

Do not commit generated outputs:

- `output/`
- `tools/visualization/output/`
- local temporary files such as `a.txt`

---

### Task 1: Evaluation Types And Truth Parser

**Files:**
- Create: `include/gici/evaluation/evaluation_types.h`
- Create: `include/gici/evaluation/evaluation_utils.h`
- Create: `src/evaluation/evaluation_utils.cpp`
- Create: `tools/evaluation/fixtures/ie_truth_sample.txt`
- Create: `tools/evaluation/evaluation_tests.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the fixture**

Create `tools/evaluation/fixtures/ie_truth_sample.txt` with this exact content:

```text
Project:     gici
Program:     Inertial Explorer Version 8.90.6611

Week     GPSTime     UTCTime       Longitude       Latitude     H-Ell         Heading           Pitch            Roll
week       (sec)       (sec)           (deg)          (deg)       (m)           (deg)           (deg)           (deg)
2254  120974.000  120956.000  121.0000000000  31.0000000000    10.000    359.0000000000    1.0000000000    -2.0000000000
bad row that must be skipped
2254  120975.000  120957.000  121.0000100000  31.0000200000    11.000      1.0000000000    2.0000000000    -1.0000000000
2254  120976.000  120958.000  121.0000300000  31.0000500000    13.000      3.0000000000    3.0000000000     0.0000000000
```

- [ ] **Step 2: Write the failing parser and angle tests**

Create `tools/evaluation/evaluation_tests.cpp`:

```cpp
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "gici/evaluation/evaluation_utils.h"

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

  std::cout << "evaluation_tests passed" << std::endl;
  return 0;
}
```

- [ ] **Step 3: Add helper declarations**

Create `include/gici/evaluation/evaluation_types.h`:

```cpp
#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace gici {

struct EvaluationTruthSample {
  double gps_time = 0.0;
  double longitude_deg = 0.0;
  double latitude_deg = 0.0;
  double height_m = 0.0;
  double heading_deg = 0.0;
  double pitch_deg = 0.0;
  double roll_deg = 0.0;
  Eigen::Vector3d velocity_enu_mps = Eigen::Vector3d::Zero();
};

struct EvaluationSkipCounters {
  int before_truth = 0;
  int after_truth = 0;
  int invalid_solution = 0;
  int interpolation_failure = 0;
};

struct EvaluationStats {
  int count = 0;
  double mean = 0.0;
  double rms = 0.0;
  double stddev = 0.0;
  double min = 0.0;
  double max = 0.0;
  double max_abs = 0.0;
  double p50 = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
};

}  // namespace gici
```

Create `include/gici/evaluation/evaluation_utils.h`:

```cpp
#pragma once

#include <string>
#include <vector>

#include "gici/evaluation/evaluation_types.h"

namespace gici {

bool loadInertialExplorerTruth(
    const std::string& path,
    std::vector<EvaluationTruthSample>& samples);

void computeTruthVelocities(std::vector<EvaluationTruthSample>& samples);

bool interpolateTruth(
    const std::vector<EvaluationTruthSample>& samples,
    double gps_time,
    size_t start_index,
    EvaluationTruthSample& interpolated);

double wrapAngleDeg(double angle_deg);

double angleDiffDeg(double lhs_deg, double rhs_deg);

double interpolateAngleDeg(double lhs_deg, double rhs_deg, double ratio);

EvaluationStats computeStats(const std::vector<double>& values);

}  // namespace gici
```

- [ ] **Step 4: Run the test to verify it fails before implementation**

Run:

```bash
cmake --build build --target evaluation_tests
```

Expected: fail because `evaluation_tests` target and evaluation headers do not exist yet.

- [ ] **Step 5: Implement parser, interpolation, angle helpers, and stats**

Create `src/evaluation/evaluation_utils.cpp`:

```cpp
#include "gici/evaluation/evaluation_utils.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

#include <glog/logging.h>

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
    const double lat_scale = 111319.49079327357;
    const double lon_scale = lat_scale * std::cos(samples[i].latitude_deg * M_PI / 180.0);
    const double east = (samples[right].longitude_deg - samples[left].longitude_deg) * lon_scale;
    const double north = (samples[right].latitude_deg - samples[left].latitude_deg) * lat_scale;
    const double up = samples[right].height_m - samples[left].height_m;
    samples[i].velocity_enu_mps = Eigen::Vector3d(east / dt, north / dt, up / dt);
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
```

- [ ] **Step 6: Wire evaluation sources and test target into CMake**

Modify the top-level `CMakeLists.txt` source-directory section to include `src/evaluation`:

```cmake
aux_source_directory(src/evaluation DIR_evaluation)
list(APPEND DIR_ALL ${DIR_utility}
                    ${DIR_stream}
                    ${DIR_gnss}
                    ${DIR_imu}
                    ${DIR_vision}
                    ${DIR_estimate}
                    ${DIR_fusion}
                    ${DIR_evaluation})
```

Add this test executable after `gici_main`:

```cmake
add_executable(evaluation_tests tools/evaluation/evaluation_tests.cpp)
target_link_libraries(evaluation_tests
                      ${PROJECT_NAME})
```

- [ ] **Step 7: Run the parser tests**

Run:

```bash
cmake --build build --target evaluation_tests
./build/evaluation_tests
```

Expected:

```text
evaluation_tests passed
```

- [ ] **Step 8: Commit Task 1**

Run:

```bash
git add CMakeLists.txt include/gici/evaluation src/evaluation tools/evaluation
git commit -m "feat: add evaluation truth parsing utilities"
```

---

### Task 2: Per-Solution Metric Row Computation

**Files:**
- Modify: `include/gici/evaluation/evaluation_types.h`
- Modify: `include/gici/evaluation/evaluation_utils.h`
- Modify: `src/evaluation/evaluation_utils.cpp`
- Modify: `tools/evaluation/evaluation_tests.cpp`

- [ ] **Step 1: Add row and status types**

Append these structs to `include/gici/evaluation/evaluation_types.h` before the closing namespace:

```cpp
struct EvaluationRow {
  double timestamp_gpst = 0.0;
  double truth_gpst = 0.0;
  double time_offset_s = 0.0;
  double elapsed_s = 0.0;

  double solution_lon_deg = 0.0;
  double solution_lat_deg = 0.0;
  double solution_height_m = 0.0;
  double solution_roll_deg = 0.0;
  double solution_pitch_deg = 0.0;
  double solution_yaw_deg = 0.0;

  double truth_lon_deg = 0.0;
  double truth_lat_deg = 0.0;
  double truth_height_m = 0.0;
  double truth_heading_deg = 0.0;
  double truth_pitch_deg = 0.0;
  double truth_roll_deg = 0.0;

  double solution_east_m = 0.0;
  double solution_north_m = 0.0;
  double solution_up_m = 0.0;
  double truth_east_m = 0.0;
  double truth_north_m = 0.0;
  double truth_up_m = 0.0;

  double east_error_m = 0.0;
  double north_error_m = 0.0;
  double up_error_m = 0.0;
  double horizontal_error_m = 0.0;
  double translation_error_m = 0.0;

  double roll_error_deg = 0.0;
  double pitch_error_deg = 0.0;
  double yaw_error_deg = 0.0;
  double attitude_error_deg = 0.0;

  double velocity_east_error_mps = 0.0;
  double velocity_north_error_mps = 0.0;
  double velocity_up_error_mps = 0.0;
  double velocity_horizontal_error_mps = 0.0;
  double velocity_3d_error_mps = 0.0;

  std::string solution_status;
  int num_satellites = 0;
  double differential_age_s = 0.0;

  double std_east_m = 0.0;
  double std_north_m = 0.0;
  double std_up_m = 0.0;
  double std_roll_deg = 0.0;
  double std_pitch_deg = 0.0;
  double std_yaw_deg = 0.0;
  double std_velocity_east_mps = 0.0;
  double std_velocity_north_mps = 0.0;
  double std_velocity_up_mps = 0.0;
};
```

- [ ] **Step 2: Write failing metric tests**

Append this block in `tools/evaluation/evaluation_tests.cpp` before the final success print:

```cpp
  Eigen::Vector3d origin_lla(31.0, 121.0, 10.0);
  Eigen::Vector3d truth_lla(31.000010, 121.000005, 10.5);
  Eigen::Vector3d solution_lla(31.000011, 121.000007, 10.8);
  Eigen::Vector3d truth_enu = gici::llaDegToEnuMeters(truth_lla, origin_lla);
  Eigen::Vector3d solution_enu = gici::llaDegToEnuMeters(solution_lla, origin_lla);
  Eigen::Vector3d error_enu = gici::llaDegToEnuMeters(solution_lla, truth_lla);
  expectTrue(std::fabs(truth_enu.x()) > 0.1, "truth east nonzero");
  expectTrue(std::fabs(solution_enu.y()) > 0.1, "solution north nonzero");
  expectTrue(std::fabs(error_enu.z() - 0.3) < 1.0e-9, "up error");
```

- [ ] **Step 3: Add metric helper declarations**

Append to `include/gici/evaluation/evaluation_utils.h`:

```cpp
Eigen::Vector3d llaDegToEnuMeters(
    const Eigen::Vector3d& lla_deg,
    const Eigen::Vector3d& origin_lla_deg);

std::string solutionStatusToString(GnssSolutionStatus status);
```

Also add these includes at the top of `evaluation_utils.h`:

```cpp
#include <Eigen/Core>

#include "gici/estimate/estimator_types.h"
```

- [ ] **Step 4: Run the metric tests to verify failure**

Run:

```bash
cmake --build build --target evaluation_tests
```

Expected: fail because `llaDegToEnuMeters` is declared but not implemented.

- [ ] **Step 5: Implement ENU helper and status conversion**

Append to `src/evaluation/evaluation_utils.cpp` before the closing namespace:

```cpp
Eigen::Vector3d llaDegToEnuMeters(
    const Eigen::Vector3d& lla_deg,
    const Eigen::Vector3d& origin_lla_deg)
{
  constexpr double kWgs84A = 6378137.0;
  constexpr double kWgs84E2 = 6.69437999014e-3;
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
```

- [ ] **Step 6: Replace approximate truth velocity with the shared ENU helper**

In `src/evaluation/evaluation_utils.cpp`, replace the body of `computeTruthVelocities` with:

```cpp
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
```

- [ ] **Step 7: Run metric tests**

Run:

```bash
cmake --build build --target evaluation_tests
./build/evaluation_tests
```

Expected:

```text
evaluation_tests passed
```

- [ ] **Step 8: Commit Task 2**

Run:

```bash
git add include/gici/evaluation src/evaluation tools/evaluation/evaluation_tests.cpp
git commit -m "feat: add evaluation metric primitives"
```

---

### Task 3: EvaluationFormator CSV And Summary Output

**Files:**
- Create: `include/gici/stream/evaluation_formator.h`
- Create: `src/stream/evaluation_formator.cpp`
- Modify: `include/gici/stream/formator.h`
- Modify: `src/stream/formator.cpp`
- Modify: `src/utility/option.cpp`
- Modify: `tools/evaluation/evaluation_tests.cpp`

- [ ] **Step 1: Write failing formator construction test**

Append this include to `tools/evaluation/evaluation_tests.cpp`:

```cpp
#include <yaml-cpp/yaml.h>
#include "gici/stream/formator.h"
```

Append this block before the final success print:

```cpp
  YAML::Node node;
  node["type"] = "evaluation";
  node["truth_path"] = "tools/evaluation/fixtures/ie_truth_sample.txt";
  node["output_csv"] = "/tmp/gici_evaluation_test.csv";
  node["output_summary"] = "/tmp/gici_evaluation_test_summary.txt";
  node["truth_format"] = "inertial-explorer";
  std::shared_ptr<gici::FormatorBase> formator = gici::makeFormator(node);
  expectTrue(static_cast<bool>(formator), "evaluation formator factory");
  expectTrue(formator->getType() == gici::FormatorType::Evaluation, "evaluation formator type");
```

- [ ] **Step 2: Run the test to verify failure**

Run:

```bash
cmake --build build --target evaluation_tests
```

Expected: fail because `FormatorType::Evaluation` and `type: evaluation` are not defined.

- [ ] **Step 3: Declare EvaluationFormator**

Create `include/gici/stream/evaluation_formator.h`:

```cpp
#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gici/evaluation/evaluation_types.h"
#include "gici/stream/formator.h"

namespace gici {

class EvaluationFormator : public FormatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Option {
    std::string truth_path;
    std::string output_csv;
    std::string output_summary;
    std::string truth_format = "inertial-explorer";
  };

  explicit EvaluationFormator(const Option& option);
  explicit EvaluationFormator(const YAML::Node& node);
  ~EvaluationFormator();

  int decode(const uint8_t *buf, int size,
             std::vector<std::shared_ptr<DataCluster>>& data) override;

  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

private:
  void initialize();
  void writeHeader();
  void writeSummary();
  bool buildRow(const Solution& solution, EvaluationRow& row);
  void appendRow(const EvaluationRow& row);

  Option option_;
  std::vector<EvaluationTruthSample> truth_;
  std::vector<EvaluationRow> rows_;
  EvaluationSkipCounters skips_;
  std::ofstream csv_;
  bool wrote_header_ = false;
  bool wrote_summary_ = false;
  size_t truth_index_ = 0;
  bool has_origin_ = false;
  Eigen::Vector3d origin_lla_deg_ = Eigen::Vector3d::Zero();
  double first_timestamp_ = 0.0;
};

}  // namespace gici
```

- [ ] **Step 4: Register the new formator type**

Modify `include/gici/stream/formator.h`:

Add `Evaluation` to `enum class FormatorType`:

```cpp
  Evaluation
```

Include the new header near the bottom after `NmeaFormator` is declared to avoid circular include issues:

```cpp
#include "gici/stream/evaluation_formator.h"
```

Add registration next to the other `MAKE_FORMATOR` calls:

```cpp
MAKE_FORMATOR(EvaluationFormator);
```

Modify `src/utility/option.cpp` in `convert<std::string, FormatorType>`:

```cpp
  MAP_IN_OUT("evaluation", FormatorType::Evaluation);
```

Modify `src/stream/formator.cpp` in `makeFormator`:

```cpp
  MAP_FORMATOR(FormatorType::Evaluation, EvaluationFormator);
```

- [ ] **Step 5: Implement minimal EvaluationFormator**

Create `src/stream/evaluation_formator.cpp`:

```cpp
#include "gici/stream/evaluation_formator.h"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <glog/logging.h>

#include "gici/evaluation/evaluation_utils.h"
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
  if (!solution.coordinate) {
    ++skips_.invalid_solution;
    return false;
  }
  if (solution.timestamp < truth_.front().gps_time) {
    ++skips_.before_truth;
    return false;
  }
  if (solution.timestamp > truth_.back().gps_time) {
    ++skips_.after_truth;
    return false;
  }

  EvaluationTruthSample truth;
  if (!interpolateTruth(truth_, solution.timestamp, truth_index_, truth)) {
    ++skips_.interpolation_failure;
    return false;
  }
  while (truth_index_ + 1 < truth_.size() && truth_[truth_index_ + 1].gps_time < solution.timestamp) {
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
    first_timestamp_ = solution.timestamp;
    has_origin_ = true;
  }

  Eigen::Vector3d solution_local = llaDegToEnuMeters(solution_lla_deg, origin_lla_deg_);
  Eigen::Vector3d truth_local = llaDegToEnuMeters(truth_lla_deg, origin_lla_deg_);
  Eigen::Vector3d error_enu = llaDegToEnuMeters(solution_lla_deg, truth_lla_deg);
  Eigen::Vector3d rpy = quaternionToEulerAngle(solution.pose.getEigenQuaternion()) * R2D;
  Eigen::Vector3d velocity_error = solution.speed_and_bias.head<3>() - truth.velocity_enu_mps;

  row.timestamp_gpst = solution.timestamp;
  row.truth_gpst = truth.gps_time;
  row.time_offset_s = solution.timestamp - truth.gps_time;
  row.elapsed_s = solution.timestamp - first_timestamp_;
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
}

void EvaluationFormator::writeSummary()
{
  if (wrote_summary_) return;
  wrote_summary_ = true;

  std::ofstream output(option_.output_summary.c_str());
  if (!output.is_open()) {
    LOG(ERROR) << "Unable to open evaluation summary " << option_.output_summary;
    return;
  }

  output << "GICI evaluation summary\n";
  output << "aligned_samples: " << rows_.size() << "\n";
  output << "skipped_before_truth: " << skips_.before_truth << "\n";
  output << "skipped_after_truth: " << skips_.after_truth << "\n";
  output << "skipped_invalid_solution: " << skips_.invalid_solution << "\n";
  output << "skipped_interpolation_failure: " << skips_.interpolation_failure << "\n";
}

}  // namespace gici
```

- [ ] **Step 6: Build and run tests**

Run:

```bash
cmake --build build --target evaluation_tests
./build/evaluation_tests
```

Expected:

```text
evaluation_tests passed
```

- [ ] **Step 7: Commit Task 3**

Run:

```bash
git add include/gici/stream/evaluation_formator.h src/stream/evaluation_formator.cpp include/gici/stream/formator.h src/stream/formator.cpp src/utility/option.cpp tools/evaluation/evaluation_tests.cpp
git commit -m "feat: add evaluation output formator"
```

---

### Task 4: Full Summary Statistics

**Files:**
- Modify: `include/gici/stream/evaluation_formator.h`
- Modify: `src/stream/evaluation_formator.cpp`
- Modify: `tools/evaluation/evaluation_tests.cpp`

- [ ] **Step 1: Add summary behavior test**

Append this helper near the top of `tools/evaluation/evaluation_tests.cpp`:

```cpp
#include <fstream>

std::string readFile(const std::string& path)
{
  std::ifstream input(path.c_str());
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}
```

Append this check after the formator factory test:

```cpp
  {
    formator.reset();
    const std::string summary = readFile("/tmp/gici_evaluation_test_summary.txt");
    expectTrue(summary.find("translation_error_m") != std::string::npos,
               "summary has translation stats header even with no aligned rows");
    expectTrue(summary.find("skipped_before_truth") != std::string::npos,
               "summary has skip counters");
  }
```

- [ ] **Step 2: Run the test to verify failure**

Run:

```bash
cmake --build build --target evaluation_tests
./build/evaluation_tests
```

Expected: fail because summary currently does not list metric stats.

- [ ] **Step 3: Add summary helper declarations to EvaluationFormator**

Add private helpers to `include/gici/stream/evaluation_formator.h`:

```cpp
  void writeMetricStats(std::ofstream& output, const std::string& name,
                        const std::vector<double>& values);
  void writeStatusGroups(std::ofstream& output);
```

- [ ] **Step 4: Implement metric stats and status grouping**

Replace `writeSummary()` in `src/stream/evaluation_formator.cpp` with:

```cpp
void EvaluationFormator::writeSummary()
{
  if (wrote_summary_) return;
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
```

Append:

```cpp
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
```

- [ ] **Step 5: Run summary tests**

Run:

```bash
cmake --build build --target evaluation_tests
./build/evaluation_tests
```

Expected:

```text
evaluation_tests passed
```

- [ ] **Step 6: Commit Task 4**

Run:

```bash
git add include/gici/stream/evaluation_formator.h src/stream/evaluation_formator.cpp tools/evaluation/evaluation_tests.cpp
git commit -m "feat: write evaluation summary statistics"
```

---

### Task 5: YAML Configuration Example

**Files:**
- Modify: `option/pseudo_real_time_estimation_RTK_TC.yaml`

- [ ] **Step 1: Add evaluation streamer**

In `option/pseudo_real_time_estimation_RTK_TC.yaml`, add this stream output beside `str_tc_solution_file`:

```yaml
    - streamer:
        tag: str_tc_evaluation_file
        input_tags: [fmt_tc_evaluation]
        type: file
        path: /home/wbz/桌面/GICI/gici-open/output/rtk_tc_evaluation.csv
        enable_time_tag: false
```

- [ ] **Step 2: Add evaluation formator**

In the `formators` list, add:

```yaml
    - formator:
        io: output
        tag: fmt_tc_evaluation
        type: evaluation
        truth_path: /home/wbz/桌面/GICI/gici-open/data/1.1/ground_truth.txt
        output_csv: /home/wbz/桌面/GICI/gici-open/output/rtk_tc_evaluation.csv
        output_summary: /home/wbz/桌面/GICI/gici-open/output/rtk_tc_evaluation_summary.txt
        truth_format: inertial-explorer
```

- [ ] **Step 3: Add output tag**

Change the estimator output tags to:

```yaml
    output_tags: [fmt_tc_solution_file, fmt_tc_evaluation]
```

- [ ] **Step 4: Smoke-test YAML parsing**

Run:

```bash
./build/gici_main option/pseudo_real_time_estimation_RTK_TC.yaml
```

Expected: program starts without `"Option evaluation invalid"` or `"Formator type not supported"` errors. Stop it after it begins processing if the full dataset run is too long.

- [ ] **Step 5: Commit Task 5**

Run:

```bash
git add option/pseudo_real_time_estimation_RTK_TC.yaml
git commit -m "config: enable RTK TC evaluation output example"
```

---

### Task 6: Visualization CSV Input Migration

**Files:**
- Modify: `tools/visualization/plot_position_error.py`
- Modify: `tools/visualization/README.md`

- [ ] **Step 1: Add failing Python smoke check**

Run this command before editing:

```bash
python3 tools/visualization/plot_position_error.py \
  --evaluation-csv output/rtk_tc_evaluation.csv \
  --output-dir tools/visualization/output
```

Expected: fail with `unrecognized arguments: --evaluation-csv`.

- [ ] **Step 2: Add evaluation CSV parser**

In `tools/visualization/plot_position_error.py`, add:

```python
def parse_evaluation_csv(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            row = {}
            for key, value in raw.items():
                if key == "solution_status":
                    row[key] = value
                elif value == "":
                    row[key] = 0.0
                else:
                    row[key] = float(value)
            row["ape_translation_m"] = row["translation_error_m"]
            row["ape_horizontal_m"] = row["horizontal_error_m"]
            row["spatial_error_m"] = row["translation_error_m"]
            row["elapsed"] = row["elapsed_s"]
            rows.append(row)
    return rows
```

- [ ] **Step 3: Add CLI argument and branch main flow**

In `parse_args()`, add:

```python
    parser.add_argument("--evaluation-csv", default=None)
```

At the start of `main()`, replace the existing parse block with:

```python
    if args.evaluation_csv:
        rows = parse_evaluation_csv(Path(args.evaluation_csv))
    else:
        solution_samples = parse_gici_nmea(solution_path)
        truth_samples = parse_truth(truth_path)
        if not solution_samples:
            raise SystemExit(f"No GGA/RMC solution samples found in {solution_path}")
        if len(truth_samples) < 2:
            raise SystemExit(f"Not enough truth samples found in {truth_path}")

        rows = align_errors(solution_samples, truth_samples)
        if not rows:
            raise SystemExit("No overlapping timestamps between solution and truth")
```

Keep the existing `metrics = summarize(rows)` and SVG writing after this block.

- [ ] **Step 4: Run Python fallback regression**

Run:

```bash
python3 tools/visualization/plot_position_error.py \
  --solution output/rtk_tc_solution.txt \
  --truth data/1.1/ground_truth.txt \
  --output-dir /tmp/gici_visualization_fallback
```

Expected:

```text
Aligned samples: <nonzero>
Wrote /tmp/gici_visualization_fallback/ape_timeseries.svg
Wrote /tmp/gici_visualization_fallback/trajectory_comparison.svg
```

- [ ] **Step 5: Run new CSV mode after integration output exists**

Run:

```bash
python3 tools/visualization/plot_position_error.py \
  --evaluation-csv output/rtk_tc_evaluation.csv \
  --output-dir /tmp/gici_visualization_evaluation
```

Expected:

```text
Aligned samples: <nonzero>
Wrote /tmp/gici_visualization_evaluation/ape_timeseries.svg
Wrote /tmp/gici_visualization_evaluation/trajectory_comparison.svg
```

- [ ] **Step 6: Update README workflow**

In `tools/visualization/README.md`, add this primary command:

```bash
python3 tools/visualization/plot_position_error.py \
  --evaluation-csv output/rtk_tc_evaluation.csv \
  --output-dir tools/visualization/output
```

Keep the old NMEA/truth command under a "fallback" heading.

- [ ] **Step 7: Commit Task 6**

Run:

```bash
git add tools/visualization/plot_position_error.py tools/visualization/README.md
git commit -m "feat: read evaluation CSV in visualization tool"
```

---

### Task 7: End-To-End Verification And Push

**Files:**
- No required source changes unless verification exposes a defect.

- [ ] **Step 1: Build core targets**

Run:

```bash
cmake --build build --target gici_main evaluation_tests
```

Expected: both targets build successfully.

- [ ] **Step 2: Run C++ tests**

Run:

```bash
./build/evaluation_tests
```

Expected:

```text
evaluation_tests passed
```

- [ ] **Step 3: Run RTK TC integration**

Run:

```bash
./build/gici_main option/pseudo_real_time_estimation_RTK_TC.yaml
```

Expected:

- `output/rtk_tc_evaluation.csv` exists.
- `output/rtk_tc_evaluation_summary.txt` exists.
- Summary includes `aligned_samples: <nonzero>`.
- Summary includes `translation_error_m`.

- [ ] **Step 4: Run visualization on evaluation CSV**

Run:

```bash
python3 tools/visualization/plot_position_error.py \
  --evaluation-csv output/rtk_tc_evaluation.csv \
  --output-dir /tmp/gici_eval_plots
```

Expected:

- `/tmp/gici_eval_plots/ape.csv`
- `/tmp/gici_eval_plots/ape_timeseries.svg`
- `/tmp/gici_eval_plots/trajectory_comparison.svg`
- `/tmp/gici_eval_plots/enu_components.svg`
- `/tmp/gici_eval_plots/summary.txt`

- [ ] **Step 5: Check git status for generated files**

Run:

```bash
git status --short
```

Expected: no generated output files staged. If generated output files appear as untracked, leave them untracked.

- [ ] **Step 6: Push branch to user's GitHub remote**

Run:

```bash
git push turry evaluation-output
```

Expected: branch updates at `git@github.com:Turry-uping/G-I-C.git`.

---

## Self-Review Notes

- Spec coverage: The plan covers IE truth parsing, GPSTime interpolation, CSV output fields, summary statistics, skip counters, YAML integration, visualization migration, testing, and GitHub branch push.
- Type consistency: `EvaluationTruthSample`, `EvaluationRow`, `EvaluationStats`, `EvaluationSkipCounters`, `EvaluationFormator`, `llaDegToEnuMeters`, `solutionStatusToString`, and `interpolateTruth` are introduced before later tasks use them.
- No generated outputs are committed.
