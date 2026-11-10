// numpy.isclose semantics for gtest, so ported tests keep the exact
// tolerances of the Python originals:  |a - b| <= atol + rtol * |b|
#pragma once

#include <gtest/gtest.h>

#include <cmath>

namespace fw_aero::test {

inline ::testing::AssertionResult IsClose(double a, double b,
                                          double rtol = 1e-5,
                                          double atol = 1e-8) {
  const double tol = atol + rtol * std::fabs(b);
  if (std::fabs(a - b) <= tol) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << a << " is not close to " << b << " (|diff| = " << std::fabs(a - b)
         << " > tol = " << tol << ")";
}

}  // namespace fw_aero::test