#pragma once

#include <string>

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

}  // namespace gici
