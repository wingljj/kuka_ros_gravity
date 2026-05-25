#include "sriforcesensor/wrench_filter.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sriforcesensor
{

WrenchFilterConfig validateWrenchFilterConfig(bool enabled,
                                              int window_size,
                                              double max_stddev_force,
                                              double max_stddev_torque)
{
  if (window_size < 1)
  {
    throw std::invalid_argument("filter window_size must be greater than zero");
  }
  if (max_stddev_force <= 0.0)
  {
    throw std::invalid_argument("max_stddev_force must be greater than zero");
  }
  if (max_stddev_torque <= 0.0)
  {
    throw std::invalid_argument("max_stddev_torque must be greater than zero");
  }
  return WrenchFilterConfig{enabled,
                            static_cast<std::size_t>(window_size),
                            max_stddev_force,
                            max_stddev_torque};
}

MovingAverageWrenchFilter::MovingAverageWrenchFilter(std::size_t window_size)
{
  setWindowSize(window_size);
}

void MovingAverageWrenchFilter::setWindowSize(std::size_t window_size)
{
  if (window_size == 0)
  {
    throw std::invalid_argument("window_size must be greater than zero");
  }
  window_size_ = window_size;
  while (samples_.size() > window_size_)
  {
    samples_.pop_front();
  }
}

void MovingAverageWrenchFilter::reset()
{
  samples_.clear();
}

WrenchSample MovingAverageWrenchFilter::update(const WrenchSample& sample)
{
  samples_.push_back(sample);
  while (samples_.size() > window_size_)
  {
    samples_.pop_front();
  }
  return mean();
}

WrenchSample MovingAverageWrenchFilter::mean() const
{
  WrenchSample result{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (samples_.empty())
  {
    return result;
  }

  for (const WrenchSample& sample : samples_)
  {
    for (std::size_t i = 0; i < result.values.size(); ++i)
    {
      result.values[i] += sample.values[i];
    }
  }
  for (double& value : result.values)
  {
    value /= static_cast<double>(samples_.size());
  }
  return result;
}

WrenchSample MovingAverageWrenchFilter::stddev() const
{
  WrenchSample result{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (samples_.empty())
  {
    return result;
  }

  const WrenchSample avg = mean();
  for (const WrenchSample& sample : samples_)
  {
    for (std::size_t i = 0; i < result.values.size(); ++i)
    {
      const double diff = sample.values[i] - avg.values[i];
      result.values[i] += diff * diff;
    }
  }
  for (double& value : result.values)
  {
    value = std::sqrt(value / static_cast<double>(samples_.size()));
  }
  return result;
}

std::size_t MovingAverageWrenchFilter::windowSize() const
{
  return window_size_;
}

std::size_t MovingAverageWrenchFilter::sampleCount() const
{
  return samples_.size();
}

}  // namespace sriforcesensor
