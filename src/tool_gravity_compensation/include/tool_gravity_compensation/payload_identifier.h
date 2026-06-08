#ifndef TOOL_GRAVITY_COMPENSATION_PAYLOAD_IDENTIFIER_H
#define TOOL_GRAVITY_COMPENSATION_PAYLOAD_IDENTIFIER_H

#include <array>
#include <stdexcept>
#include <vector>

namespace tool_gravity_compensation
{

constexpr double kStandardGravity = 9.80665;
constexpr double kMinimumPayloadMassKg = 0.05;
constexpr double kMaximumPayloadMassKg = 200.0;
constexpr double kMaximumComDistanceM = 2.0;
constexpr double kMaximumForceMagnitudeN = 10000.0;
constexpr double kMaximumTorqueMagnitudeNm = 1000.0;
constexpr double kMaximumLeastSquaresConditionNumber = 1000.0;
constexpr double kMaximumForceResidualN = 5.0;
constexpr double kMaximumTorqueResidualNm = 0.5;

struct LoadInertialProfile
{
  double mass_kg;
  std::array<double, 3> com_sensor_m;
  double residual_error;
};

struct WrenchObservation
{
  std::array<double, 3> force;
  std::array<double, 3> torque;
  std::array<double, 9> base_R_sensor;
};

struct PayloadMassProperties
{
  double mass_kg;
  double weight_n;
  std::array<double, 3> com_sensor_m;
};

PayloadMassProperties computePayloadFromProfiles(const LoadInertialProfile& tool_only,
                                                 const LoadInertialProfile& tool_plus_payload,
                                                 double gravity = kStandardGravity,
                                                 double minimum_payload_mass_kg = kMinimumPayloadMassKg);

LoadInertialProfile estimateLoadProfileFromWrenches(const std::vector<WrenchObservation>& observations,
                                                    double gravity = kStandardGravity,
                                                    double maximum_condition_number = kMaximumLeastSquaresConditionNumber,
                                                    double maximum_force_residual_n = kMaximumForceResidualN,
                                                    double maximum_torque_residual_nm = kMaximumTorqueResidualNm);

}  // namespace tool_gravity_compensation

#endif  // TOOL_GRAVITY_COMPENSATION_PAYLOAD_IDENTIFIER_H
