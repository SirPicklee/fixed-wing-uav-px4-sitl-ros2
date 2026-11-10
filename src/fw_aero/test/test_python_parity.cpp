// Numerical parity between the C++ port and the original Python model.
//
// The reference CSV is produced by tools/generate_aero_reference.py, which
// runs uavsim.aerodynamics (uav-6dof-autopilot v1.0) on 514 inputs: 14
// hand-picked edge cases covering every branch, plus 500 random cases.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "fw_aero/aerodynamics.hpp"
#include "is_close.hpp"

#ifndef FW_AERO_TEST_DATA_DIR
#error "FW_AERO_TEST_DATA_DIR must be defined by CMake"
#endif

namespace {

constexpr int kNumInputs = 11;
constexpr int kNumOutputs = 12;
constexpr int kNumCols = kNumInputs + kNumOutputs;

const std::array<const char*, kNumOutputs> kOutputNames = {
    "fx", "fy", "fz", "l", "m", "n",
    "alpha", "beta", "airspeed", "qbar", "lift", "drag"};

// Tight tolerances: both implementations perform the same IEEE-754
// operations, so only last-digit rounding differences are expected.
constexpr double kRtol = 1e-9;
constexpr double kAtol = 1e-6;  // N, N m or rad; for values near zero

using Row = std::array<double, kNumCols>;

std::vector<Row> LoadReference(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("Cannot open reference file: " + path);
  }
  std::vector<Row> rows;
  std::string line;
  bool header_skipped = false;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    if (!header_skipped) {
      header_skipped = true;
      continue;
    }
    Row row{};
    std::stringstream ss(line);
    std::string cell;
    int col = 0;
    while (std::getline(ss, cell, ',') && col < kNumCols) {
      row[col++] = std::stod(cell);
    }
    if (col != kNumCols) {
      throw std::runtime_error("Malformed row in reference file: " + line);
    }
    rows.push_back(row);
  }
  return rows;
}

std::array<double, kNumOutputs> ComputeOutputs(const Row& in) {
  const Eigen::Vector3d v_body(in[0], in[1], in[2]);
  const Eigen::Vector3d omega(in[3], in[4], in[5]);
  fw_aero::Controls ctrl;
  ctrl.elevator = in[6];
  ctrl.aileron = in[7];
  ctrl.rudder = in[8];
  ctrl.throttle = in[9];
  const double altitude = in[10];

  const auto res = fw_aero::aero_forces_moments(fw_aero::AircraftParams{},
                                                v_body, omega, ctrl, altitude);
  return {res.force_body.x(),  res.force_body.y(),  res.force_body.z(),
          res.moment_body.x(), res.moment_body.y(), res.moment_body.z(),
          res.alpha,           res.beta,            res.airspeed,
          res.dynamic_pressure, res.lift,           res.drag};
}

}  // namespace

TEST(PythonParity, MatchesReferenceModel) {
  const auto rows =
      LoadReference(std::string(FW_AERO_TEST_DATA_DIR) + "/aero_reference.csv");
  ASSERT_EQ(rows.size(), 514u) << "Unexpected number of reference cases";

  std::array<double, kNumOutputs> max_abs_err{};
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto out = ComputeOutputs(rows[i]);
    for (int k = 0; k < kNumOutputs; ++k) {
      const double expected = rows[i][kNumInputs + k];
      max_abs_err[k] = std::max(max_abs_err[k], std::fabs(out[k] - expected));
      EXPECT_TRUE(fw_aero::test::IsClose(out[k], expected, kRtol, kAtol))
          << "case " << i << ", output " << kOutputNames[k];
    }
  }

  // Report the worst-case discrepancy per output (visible with ctest -V).
  for (int k = 0; k < kNumOutputs; ++k) {
    std::cout << "  max |C++ - Python| for " << kOutputNames[k] << ": "
              << max_abs_err[k] << "\n";
  }
}