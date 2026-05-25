#include "tool_gravity_compensation/payload_identifier.h"

#include <Eigen/Dense>
#include <cmath>

namespace tool_gravity_compensation
{

PayloadMassProperties computePayloadFromProfiles(const LoadInertialProfile& tool_only,
                                                 const LoadInertialProfile& tool_plus_payload,
                                                 double gravity)
{
  if (tool_only.mass_kg < 0.0 || tool_plus_payload.mass_kg < 0.0)
  {
    throw std::invalid_argument("profile mass must be non-negative");
  }
  if (gravity <= 0.0)
  {
    throw std::invalid_argument("gravity must be positive");
  }

  const double payload_mass = tool_plus_payload.mass_kg - tool_only.mass_kg;
  if (payload_mass <= 0.0)
  {
    throw std::invalid_argument("payload mass must be positive after tare subtraction");
  }

  PayloadMassProperties result;
  result.mass_kg = payload_mass;
  result.weight_n = payload_mass * gravity;
  for (std::size_t i = 0; i < result.com_sensor_m.size(); ++i)
  {
    const double total_moment = tool_plus_payload.mass_kg * tool_plus_payload.com_sensor_m[i];
    const double tool_moment = tool_only.mass_kg * tool_only.com_sensor_m[i];
    result.com_sensor_m[i] = (total_moment - tool_moment) / payload_mass;
  }
  return result;
}

LoadInertialProfile estimateLoadProfileFromWrenches(const std::vector<WrenchObservation>& observations,
                                                    double gravity)
{
  if (observations.size() < 3)
  {
    throw std::invalid_argument("at least three static wrench observations are required");
  }
  if (gravity <= 0.0)
  {
    throw std::invalid_argument("gravity must be positive");
  }

  Eigen::MatrixXd force_a(static_cast<Eigen::Index>(observations.size() * 3), 4);
  Eigen::VectorXd force_b(static_cast<Eigen::Index>(observations.size() * 3));
  force_a.setZero();

  for (std::size_t i = 0; i < observations.size(); ++i)
  {
    const WrenchObservation& observation = observations[i];
    const Eigen::Matrix3d base_R_sensor =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(observation.base_R_sensor.data());
    const Eigen::Vector3d gravity_per_kg_sensor =
      base_R_sensor.transpose() * Eigen::Vector3d(0.0, 0.0, -gravity);

    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
      const Eigen::Index row = static_cast<Eigen::Index>(i * 3) + axis;
      force_a(row, 0) = gravity_per_kg_sensor(axis);
      force_a(row, axis + 1) = 1.0;
      force_b(row) = observation.force[static_cast<std::size_t>(axis)];
    }
  }

  const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> force_qr(force_a);
  if (force_qr.rank() < 4)
  {
    throw std::invalid_argument("force observations are underconstrained");
  }
  const Eigen::VectorXd force_solution = force_qr.solve(force_b);
  const double mass = force_solution(0);
  if (mass <= 0.0)
  {
    throw std::invalid_argument("estimated load mass must be positive");
  }

  Eigen::MatrixXd torque_a(static_cast<Eigen::Index>(observations.size() * 3), 6);
  Eigen::VectorXd torque_b(static_cast<Eigen::Index>(observations.size() * 3));
  torque_a.setZero();

  for (std::size_t i = 0; i < observations.size(); ++i)
  {
    const WrenchObservation& observation = observations[i];
    const Eigen::Matrix3d base_R_sensor =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(observation.base_R_sensor.data());
    const Eigen::Vector3d load_force_sensor =
      mass * (base_R_sensor.transpose() * Eigen::Vector3d(0.0, 0.0, -gravity));

    Eigen::Matrix3d minus_skew_force;
    minus_skew_force << 0.0, load_force_sensor.z(), -load_force_sensor.y(),
                       -load_force_sensor.z(), 0.0, load_force_sensor.x(),
                        load_force_sensor.y(), -load_force_sensor.x(), 0.0;

    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
      const Eigen::Index row = static_cast<Eigen::Index>(i * 3) + axis;
      torque_a.block<1, 3>(row, 0) = minus_skew_force.block<1, 3>(axis, 0);
      torque_a(row, axis + 3) = 1.0;
      torque_b(row) = observation.torque[static_cast<std::size_t>(axis)];
    }
  }

  const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> torque_qr(torque_a);
  if (torque_qr.rank() < 6)
  {
    throw std::invalid_argument("torque observations are underconstrained");
  }
  const Eigen::VectorXd torque_solution = torque_qr.solve(torque_b);

  LoadInertialProfile profile;
  profile.mass_kg = mass;
  profile.com_sensor_m = {{torque_solution(0), torque_solution(1), torque_solution(2)}};
  return profile;
}

}  // namespace tool_gravity_compensation
