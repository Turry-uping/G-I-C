#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "gici/estimate/estimator_types.h"
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

Eigen::Vector3d llaDegToEnuMeters(
    const Eigen::Vector3d& lla_deg,
    const Eigen::Vector3d& origin_lla_deg);

std::string solutionStatusToString(GnssSolutionStatus status);

EvaluationStats computeStats(const std::vector<double>& values);

}  // namespace gici
