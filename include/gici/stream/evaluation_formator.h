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
  void writeMetricStats(std::ofstream& output, const std::string& name,
                        const std::vector<double>& values);
  void writeStatusGroups(std::ofstream& output);
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
