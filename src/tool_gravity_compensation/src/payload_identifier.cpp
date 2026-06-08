#include "tool_gravity_compensation/payload_identifier.h"

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

namespace tool_gravity_compensation
{

PayloadMassProperties computePayloadFromProfiles(const LoadInertialProfile& tool_only,
                                                 const LoadInertialProfile& tool_plus_payload,
                                                 double gravity,
                                                 double minimum_payload_mass_kg)
{
  if (tool_only.mass_kg < 0.0 || tool_plus_payload.mass_kg < 0.0)
  {
    throw std::invalid_argument("profile 质量必须为非负数");
  }
  if (gravity <= 0.0)
  {
    throw std::invalid_argument("重力值必须为正数");
  }
  if (minimum_payload_mass_kg <= 0.0)
  {
    throw std::invalid_argument("minimum_payload_mass_kg 必须为正数");
  }

  const double payload_mass = tool_plus_payload.mass_kg - tool_only.mass_kg;
  if (payload_mass < minimum_payload_mass_kg)
  {
    std::ostringstream stream;
    stream << "皮重扣除后的负载质量低于可靠阈值: "
           << payload_mass << " kg < " << minimum_payload_mass_kg << " kg";
    throw std::invalid_argument(stream.str());
  }
  if (payload_mass > kMaximumPayloadMassKg)
  {
    std::ostringstream stream;
    stream << "负载质量超出最大允许值: "
           << payload_mass << " kg > " << kMaximumPayloadMassKg << " kg";
    throw std::invalid_argument(stream.str());
  }
  if (tool_plus_payload.mass_kg > kMaximumPayloadMassKg)
  {
    std::ostringstream stream;
    stream << "总质量（工具+负载）超出最大允许值: "
           << tool_plus_payload.mass_kg << " kg > " << kMaximumPayloadMassKg << " kg";
    throw std::invalid_argument(stream.str());
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

  // COM distance bounds check
  double com_distance_sq = 0.0;
  for (std::size_t i = 0; i < result.com_sensor_m.size(); ++i)
  {
    com_distance_sq += result.com_sensor_m[i] * result.com_sensor_m[i];
  }
  const double com_distance = std::sqrt(com_distance_sq);
  if (!std::isfinite(com_distance) || com_distance > kMaximumComDistanceM)
  {
    std::ostringstream stream;
    stream << "计算得到的负载质心距离超出最大允许值: "
           << com_distance << " m > " << kMaximumComDistanceM << " m";
    throw std::invalid_argument(stream.str());
  }

  return result;
}

double conditionNumber(const Eigen::MatrixXd& matrix)
{
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values.size() == 0)
  {
    return std::numeric_limits<double>::infinity();
  }

  const double largest = singular_values(0);
  const double smallest = singular_values(singular_values.size() - 1);
  if (smallest <= 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  return largest / smallest;
}

void validateLeastSquaresQuality(const char* label,
                                 const Eigen::MatrixXd& a,
                                 const Eigen::VectorXd& b,
                                 const Eigen::VectorXd& x,
                                 int expected_rank,
                                 double maximum_condition_number,
                                 double maximum_residual)
{
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(a);
  if (svd.rank() < expected_rank)
  {
    throw std::invalid_argument(std::string(label) + " 观测数据欠约束");
  }

  const Eigen::VectorXd singular_values = svd.singularValues();
  const double smallest = singular_values(singular_values.size() - 1);
  const double kRankTolerance = 1e-9;
  if (smallest <= kRankTolerance)
  {
    throw std::invalid_argument(std::string(label) + " 观测数据病态");
  }

  const double condition = conditionNumber(a);
  if (condition > maximum_condition_number)
  {
    std::ostringstream stream;
    stream << label << " 观测数据病态: condition_number="
           << condition << " > " << maximum_condition_number;
    throw std::invalid_argument(stream.str());
  }

  const Eigen::VectorXd residual = a * x - b;
  const double residual_rms = std::sqrt(residual.squaredNorm() / static_cast<double>(residual.size()));
  if (residual_rms > maximum_residual)
  {
    std::ostringstream stream;
    stream << label << " 残差过高: rms=" << residual_rms
           << " > " << maximum_residual;
    throw std::invalid_argument(stream.str());
  }
}

LoadInertialProfile estimateLoadProfileFromWrenches(const std::vector<WrenchObservation>& observations,
                                                    double gravity,
                                                    double maximum_condition_number,
                                                    double maximum_force_residual_n,
                                                    double maximum_torque_residual_nm)
{
  if (observations.size() < 3)
  {
    throw std::invalid_argument("至少需要三个静态力矩观测值");
  }
  if (gravity <= 0.0)
  {
    throw std::invalid_argument("重力值必须为正数");
  }
  if (maximum_condition_number <= 1.0)
  {
    throw std::invalid_argument("maximum_condition_number 必须大于 1");
  }
  if (maximum_force_residual_n <= 0.0 || maximum_torque_residual_nm <= 0.0)
  {
    throw std::invalid_argument("残差阈值必须为正数");
  }

  // NaN/Inf and magnitude guard on input observations
  for (std::size_t i = 0; i < observations.size(); ++i)
  {
    const WrenchObservation& obs = observations[i];
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      if (!std::isfinite(obs.force[axis]) || !std::isfinite(obs.torque[axis]))
      {
        throw std::invalid_argument("观测值包含 NaN 或 Inf 力/力矩数据");
      }
      if (std::abs(obs.force[axis]) > kMaximumForceMagnitudeN ||
          std::abs(obs.torque[axis]) > kMaximumTorqueMagnitudeNm)
      {
        throw std::invalid_argument("观测值力/力矩超出传感器量程");
      }
    }
    for (std::size_t j = 0; j < 9; ++j)
    {
      if (!std::isfinite(obs.base_R_sensor[j]))
      {
        throw std::invalid_argument("观测值旋转矩阵包含 NaN 或 Inf");
      }
    }
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

  const Eigen::VectorXd force_solution =
      force_a.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(force_b);
  validateLeastSquaresQuality("force",
                              force_a,
                              force_b,
                              force_solution,
                              4,
                              maximum_condition_number,
                              maximum_force_residual_n);
  const double mass = force_solution(0);
  if (mass <= 0.0)
  {
    throw std::invalid_argument("估计的负载质量必须为正数");
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

  const Eigen::VectorXd torque_solution =
      torque_a.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(torque_b);
  validateLeastSquaresQuality("torque",
                              torque_a,
                              torque_b,
                              torque_solution,
                              6,
                              maximum_condition_number,
                              maximum_torque_residual_nm);

  const Eigen::VectorXd force_residual = force_a * force_solution - force_b;
  const Eigen::VectorXd torque_residual = torque_a * torque_solution - torque_b;

  LoadInertialProfile profile;
  profile.mass_kg = mass;
  profile.com_sensor_m = {{torque_solution(0), torque_solution(1), torque_solution(2)}};
  profile.residual_error =
      std::sqrt((force_residual.squaredNorm() + torque_residual.squaredNorm()) /
                static_cast<double>(force_residual.size() + torque_residual.size()));
  return profile;
}

}  // namespace tool_gravity_compensation
