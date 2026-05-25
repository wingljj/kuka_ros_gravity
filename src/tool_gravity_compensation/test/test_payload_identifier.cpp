#include <gtest/gtest.h>

#include "tool_gravity_compensation/payload_identifier.h"

using tool_gravity_compensation::LoadInertialProfile;
using tool_gravity_compensation::PayloadMassProperties;
using tool_gravity_compensation::WrenchObservation;
using tool_gravity_compensation::computePayloadFromProfiles;
using tool_gravity_compensation::estimateLoadProfileFromWrenches;

namespace
{
std::array<double, 3> matVec(const std::array<double, 9>& matrix, const std::array<double, 3>& vector)
{
  return {{
    matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
    matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
    matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2]
  }};
}

std::array<double, 3> transposeMatVec(const std::array<double, 9>& matrix, const std::array<double, 3>& vector)
{
  return {{
    matrix[0] * vector[0] + matrix[3] * vector[1] + matrix[6] * vector[2],
    matrix[1] * vector[0] + matrix[4] * vector[1] + matrix[7] * vector[2],
    matrix[2] * vector[0] + matrix[5] * vector[1] + matrix[8] * vector[2]
  }};
}

std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
  return {{
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  }};
}
}  // namespace

TEST(PayloadIdentifier, ComputesPayloadMassWeightAndCenterOfMassByTareSubtraction)
{
  const LoadInertialProfile tool_only{2.0, {0.10, 0.00, 0.20}};
  const LoadInertialProfile total{5.0, {0.22, 0.06, 0.32}};

  const PayloadMassProperties payload = computePayloadFromProfiles(tool_only, total);

  EXPECT_DOUBLE_EQ(payload.mass_kg, 3.0);
  EXPECT_NEAR(payload.weight_n, 29.41995, 1e-9);
  EXPECT_NEAR(payload.com_sensor_m[0], 0.30, 1e-12);
  EXPECT_NEAR(payload.com_sensor_m[1], 0.10, 1e-12);
  EXPECT_NEAR(payload.com_sensor_m[2], 0.40, 1e-12);
}

TEST(PayloadIdentifier, KeepsPayloadResultDistinctFromToolProfile)
{
  const LoadInertialProfile tool_only{1.5, {-0.02, 0.01, 0.12}};
  const LoadInertialProfile total{4.0, {0.10, 0.05, 0.20}};

  const PayloadMassProperties payload = computePayloadFromProfiles(tool_only, total);

  EXPECT_DOUBLE_EQ(payload.mass_kg, 2.5);
  EXPECT_NE(payload.mass_kg, tool_only.mass_kg);
}

TEST(PayloadIdentifier, RejectsNonPositivePayloadAfterTareSubtraction)
{
  const LoadInertialProfile tool_only{4.0, {0.0, 0.0, 0.0}};
  const LoadInertialProfile total{4.0, {0.1, 0.1, 0.1}};

  EXPECT_THROW(computePayloadFromProfiles(tool_only, total), std::invalid_argument);
}

TEST(PayloadIdentifier, RejectsInvalidGravity)
{
  const LoadInertialProfile tool_only{1.0, {0.0, 0.0, 0.0}};
  const LoadInertialProfile total{2.0, {0.1, 0.1, 0.1}};

  EXPECT_THROW(computePayloadFromProfiles(tool_only, total, 0.0), std::invalid_argument);
}

TEST(PayloadIdentifier, EstimatesLoadProfileFromMultipleStaticWrenchOrientations)
{
  const double mass = 3.25;
  const std::array<double, 3> com{{0.08, -0.03, 0.14}};
  const std::array<double, 3> force_bias{{1.1, -0.7, 0.4}};
  const std::array<double, 3> torque_bias{{0.05, -0.02, 0.03}};
  const std::array<double, 3> gravity_force_base{{0.0, 0.0, -mass * tool_gravity_compensation::kStandardGravity}};

  const std::array<std::array<double, 9>, 6> base_R_sensor_values{{
    {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
    {{0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
    {{0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0}},
    {{1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0}},
    {{0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0}},
    {{0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0}}
  }};

  std::vector<WrenchObservation> observations;
  for (const std::array<double, 9>& base_R_sensor : base_R_sensor_values)
  {
    const std::array<double, 3> load_force_sensor = transposeMatVec(base_R_sensor, gravity_force_base);
    const std::array<double, 3> load_torque_sensor = cross(com, load_force_sensor);

    WrenchObservation observation;
    observation.base_R_sensor = base_R_sensor;
    for (std::size_t i = 0; i < 3; ++i)
    {
      observation.force[i] = load_force_sensor[i] + force_bias[i];
      observation.torque[i] = load_torque_sensor[i] + torque_bias[i];
    }
    observations.push_back(observation);
  }

  const LoadInertialProfile profile = estimateLoadProfileFromWrenches(observations);

  EXPECT_NEAR(profile.mass_kg, mass, 1e-9);
  EXPECT_NEAR(profile.com_sensor_m[0], com[0], 1e-9);
  EXPECT_NEAR(profile.com_sensor_m[1], com[1], 1e-9);
  EXPECT_NEAR(profile.com_sensor_m[2], com[2], 1e-9);
}

TEST(PayloadIdentifier, RejectsUnderconstrainedLoadProfileEstimation)
{
  std::vector<WrenchObservation> observations(2);

  EXPECT_THROW(estimateLoadProfileFromWrenches(observations), std::invalid_argument);
}
