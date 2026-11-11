// Dryden turbulence tests. Intensities and length scales are checked against
// hand-computed MIL-F-8785C values; the generated process is checked for its
// defining statistics (variance and correlation over one length scale).
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "fw_aero/wind.hpp"

using fw_aero::DrydenLowAltitude;
using fw_aero::DrydenTurbulence;

namespace {

constexpr double kKnot = 0.514444;  // m/s

// Sample standard deviation and correlation at a lag of `lag` samples.
void Stats(const std::vector<double>& x, int lag, double* std_dev, double* corr) {
  double mean = 0.0;
  for (double v : x) mean += v;
  mean /= x.size();
  double var = 0.0, cov = 0.0;
  for (std::size_t k = 0; k < x.size(); ++k) {
    var += (x[k] - mean) * (x[k] - mean);
    if (k + lag < x.size()) cov += (x[k] - mean) * (x[k + lag] - mean);
  }
  *std_dev = std::sqrt(var / x.size());
  *corr = cov / var;
}

}  // namespace

TEST(Dryden, LowAltitudeScalesAt100m) {
  // h = 328.08 ft, d = 0.177 + 0.000823 h = 0.44701
  // L_u = 328.08 / 0.44701^1.2 = 862.2 ft = 262.8 m, L_w = h = 100 m
  // sigma_w = 0.1 * 15 kt = 0.7717 m/s, sigma_u = 0.7717 / 0.44701^0.4 = 1.0649 m/s
  const auto s = DrydenLowAltitude(100.0, 15.0 * kKnot);
  EXPECT_NEAR(s.length_w, 100.0, 1e-9);
  EXPECT_NEAR(s.length_u, 262.79, 0.01);
  EXPECT_NEAR(s.length_v, s.length_u, 1e-12);
  EXPECT_NEAR(s.sigma_w, 0.77167, 1e-5);
  EXPECT_NEAR(s.sigma_u, 1.0649, 1e-4);
  EXPECT_NEAR(s.sigma_v, s.sigma_u, 1e-12);
}

TEST(Dryden, AltitudeIsClampedToModelRange) {
  // Below 10 ft (on the runway) the 10 ft scales apply; above 1000 ft the
  // 1000 ft scales.
  const auto ground = DrydenLowAltitude(0.5, 10.0);
  const auto ten_ft = DrydenLowAltitude(10.0 * 0.3048, 10.0);
  EXPECT_DOUBLE_EQ(ground.length_w, ten_ft.length_w);
  EXPECT_DOUBLE_EQ(ground.sigma_u, ten_ft.sigma_u);
  const auto high = DrydenLowAltitude(2000.0, 10.0);
  EXPECT_NEAR(high.length_w, 304.8, 1e-9);
}

TEST(Dryden, ZeroIntensityGivesNoGusts) {
  DrydenTurbulence turb(0.0, 1);
  for (int k = 0; k < 100; ++k) {
    EXPECT_EQ(turb.Step(100.0, 45.0, 0.02), Eigen::Vector3d::Zero());
  }
}

TEST(Dryden, SameSeedReproducesTheSequence) {
  DrydenTurbulence a(15.0 * kKnot, 42), b(15.0 * kKnot, 42), c(15.0 * kKnot, 43);
  bool differs = false;
  for (int k = 0; k < 1000; ++k) {
    const Eigen::Vector3d ga = a.Step(100.0, 45.0, 0.02);
    EXPECT_EQ(ga, b.Step(100.0, 45.0, 0.02));
    differs |= (ga - c.Step(100.0, 45.0, 0.02)).norm() > 1e-9;
  }
  EXPECT_TRUE(differs);
}

TEST(Dryden, ProcessHasDrydenVarianceAndCorrelationLength) {
  // 1e6 steps of 20 ms at 45 m/s = 45000 km of flight, about 1.7e5 length
  // scales of w: sample statistics are accurate to a few percent.
  constexpr double kV = 45.0, kDt = 0.02, kH = 100.0;
  const auto s = DrydenLowAltitude(kH, 15.0 * kKnot);
  DrydenTurbulence turb(15.0 * kKnot, 7);
  std::vector<double> u, w;
  u.reserve(1000000);
  w.reserve(1000000);
  for (int k = 0; k < 1000000; ++k) {
    const Eigen::Vector3d g = turb.Step(kH, kV, kDt);
    u.push_back(g.x());
    w.push_back(g.z());
  }
  // Correlation after flying one length scale should be exp(-1) = 0.368.
  const int lag_u = static_cast<int>(std::lround(s.length_u / (kV * kDt)));
  const int lag_w = static_cast<int>(std::lround(s.length_w / (kV * kDt)));
  double sd, corr;
  Stats(u, lag_u, &sd, &corr);
  EXPECT_NEAR(sd / s.sigma_u, 1.0, 0.05);
  EXPECT_NEAR(corr, std::exp(-1.0), 0.03);
  Stats(w, lag_w, &sd, &corr);
  EXPECT_NEAR(sd / s.sigma_w, 1.0, 0.05);
  EXPECT_NEAR(corr, std::exp(-1.0), 0.03);
}
