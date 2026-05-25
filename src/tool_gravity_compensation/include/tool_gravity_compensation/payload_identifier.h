#ifndef TOOL_GRAVITY_COMPENSATION_PAYLOAD_IDENTIFIER_H
#define TOOL_GRAVITY_COMPENSATION_PAYLOAD_IDENTIFIER_H

#include <array>
#include <stdexcept>
#include <vector>

namespace tool_gravity_compensation
{

constexpr double kStandardGravity = 9.80665;

struct LoadInertialProfile
{
  double mass_kg;
  std::array<double, 3> com_sensor_m;
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
                                                 double gravity = kStandardGravity);

LoadInertialProfile estimateLoadProfileFromWrenches(const std::vector<WrenchObservation>& observations,
                                                    double gravity = kStandardGravity);

}  // namespace tool_gravity_compensation

#endif  // TOOL_GRAVITY_COMPENSATION_PAYLOAD_IDENTIFIER_H
