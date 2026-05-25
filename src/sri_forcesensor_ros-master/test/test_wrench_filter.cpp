#include <gtest/gtest.h>

#include "sriforcesensor/wrench_filter.h"

using sriforcesensor::MovingAverageWrenchFilter;
using sriforcesensor::WrenchSample;

TEST(MovingAverageWrenchFilter, AveragesOnlyTheConfiguredWindow)
{
  MovingAverageWrenchFilter filter(3);

  filter.update(WrenchSample{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}});
  filter.update(WrenchSample{{2.0, 4.0, 6.0, 8.0, 10.0, 12.0}});
  filter.update(WrenchSample{{3.0, 6.0, 9.0, 12.0, 15.0, 18.0}});
  const WrenchSample after_three = filter.update(WrenchSample{{4.0, 8.0, 12.0, 16.0, 20.0, 24.0}});

  EXPECT_EQ(filter.sampleCount(), 3u);
  EXPECT_DOUBLE_EQ(after_three.values[0], 3.0);
  EXPECT_DOUBLE_EQ(after_three.values[1], 6.0);
  EXPECT_DOUBLE_EQ(after_three.values[5], 18.0);
}

TEST(MovingAverageWrenchFilter, ComputesPopulationStddevForCurrentWindow)
{
  MovingAverageWrenchFilter filter(2);

  filter.update(WrenchSample{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}});
  filter.update(WrenchSample{{3.0, 6.0, 9.0, 12.0, 15.0, 18.0}});
  const WrenchSample stddev = filter.stddev();

  EXPECT_DOUBLE_EQ(stddev.values[0], 1.0);
  EXPECT_DOUBLE_EQ(stddev.values[1], 2.0);
  EXPECT_DOUBLE_EQ(stddev.values[5], 6.0);
}

TEST(MovingAverageWrenchFilter, RejectsZeroWindow)
{
  EXPECT_THROW(MovingAverageWrenchFilter filter(0), std::invalid_argument);
}

TEST(WrenchFilterConfig, AcceptsPositiveRuntimeFilterSettings)
{
  const sriforcesensor::WrenchFilterConfig config =
      sriforcesensor::validateWrenchFilterConfig(true, 12, 1.5, 0.15);

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.window_size, 12u);
  EXPECT_DOUBLE_EQ(config.max_stddev_force, 1.5);
  EXPECT_DOUBLE_EQ(config.max_stddev_torque, 0.15);
}

TEST(WrenchFilterConfig, RejectsUnsafeRuntimeFilterSettings)
{
  EXPECT_THROW(sriforcesensor::validateWrenchFilterConfig(true, 0, 1.0, 0.1),
               std::invalid_argument);
  EXPECT_THROW(sriforcesensor::validateWrenchFilterConfig(true, 10, 0.0, 0.1),
               std::invalid_argument);
  EXPECT_THROW(sriforcesensor::validateWrenchFilterConfig(true, 10, 1.0, -0.1),
               std::invalid_argument);
}
