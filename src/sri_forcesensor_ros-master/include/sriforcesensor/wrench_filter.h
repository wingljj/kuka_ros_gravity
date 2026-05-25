#ifndef SRIFORCESENSOR_WRENCH_FILTER_H
#define SRIFORCESENSOR_WRENCH_FILTER_H

#include <array>
#include <cstddef>
#include <deque>

namespace sriforcesensor
{

struct WrenchSample
{
  std::array<double, 6> values;
};

struct WrenchFilterConfig
{
  bool enabled;
  std::size_t window_size;
  double max_stddev_force;
  double max_stddev_torque;
};

WrenchFilterConfig validateWrenchFilterConfig(bool enabled,
                                              int window_size,
                                              double max_stddev_force,
                                              double max_stddev_torque);

class MovingAverageWrenchFilter
{
public:
  explicit MovingAverageWrenchFilter(std::size_t window_size = 10);

  void setWindowSize(std::size_t window_size);
  void reset();
  WrenchSample update(const WrenchSample& sample);
  WrenchSample mean() const;
  WrenchSample stddev() const;

  std::size_t windowSize() const;
  std::size_t sampleCount() const;

private:
  std::size_t window_size_;
  std::deque<WrenchSample> samples_;
};

}  // namespace sriforcesensor

#endif  // SRIFORCESENSOR_WRENCH_FILTER_H
