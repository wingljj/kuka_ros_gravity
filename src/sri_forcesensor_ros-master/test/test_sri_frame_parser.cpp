#include <gtest/gtest.h>

#include "TCPClient.h"

#include <array>
#include <cstring>
#include <vector>

namespace
{
std::array<unsigned char, 31> makeEngineeringFrame(const std::array<float, 6>& values)
{
  std::array<unsigned char, 31> frame{};
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = 0x00;
  frame[3] = 0x1B;
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    unsigned char bytes[4];
    std::memcpy(bytes, &values[i], sizeof(float));
    const std::size_t offset = 6 + i * 4;
    frame[offset + 0] = bytes[0];
    frame[offset + 1] = bytes[1];
    frame[offset + 2] = bytes[2];
    frame[offset + 3] = bytes[3];
  }
  return frame;
}
}  // namespace

TEST(SriEngineeringFrameParser, RejectsShortFramesWithoutProducingValues)
{
  std::vector<unsigned char> stream{0xAA, 0x55, 0x00, 0x1B, 0x01};
  std::array<double, 6> values{{42.0, 42.0, 42.0, 42.0, 42.0, 42.0}};

  EXPECT_FALSE(TCPClient::extractLatestEngineeringFrame(stream, values));
  EXPECT_EQ(stream.size(), 5u);
  for (double value : values)
  {
    EXPECT_DOUBLE_EQ(value, 42.0);
  }
}

TEST(SriEngineeringFrameParser, RejectsNoiseAndKeepsPossiblePartialHeader)
{
  std::vector<unsigned char> stream{0x01, 0x02, 0x03, 0xAA, 0x55};
  std::array<double, 6> values{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  EXPECT_FALSE(TCPClient::extractLatestEngineeringFrame(stream, values));
  ASSERT_EQ(stream.size(), 2u);
  EXPECT_EQ(stream[0], 0xAA);
  EXPECT_EQ(stream[1], 0x55);
}

TEST(SriEngineeringFrameParser, DecodesLatestCompleteFrameAndLeavesTrailingBytes)
{
  const std::array<float, 6> first{{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f}};
  const std::array<float, 6> second{{-7.5f, 8.25f, -9.5f, 10.75f, -11.5f, 12.25f}};
  const std::array<unsigned char, 31> first_frame = makeEngineeringFrame(first);
  const std::array<unsigned char, 31> second_frame = makeEngineeringFrame(second);

  std::vector<unsigned char> stream{0x00, 0x01};
  stream.insert(stream.end(), first_frame.begin(), first_frame.end());
  stream.insert(stream.end(), second_frame.begin(), second_frame.end());
  stream.push_back(0xAA);

  std::array<double, 6> values{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  ASSERT_TRUE(TCPClient::extractLatestEngineeringFrame(stream, values));
  for (std::size_t i = 0; i < second.size(); ++i)
  {
    EXPECT_FLOAT_EQ(static_cast<float>(values[i]), second[i]);
  }
  ASSERT_EQ(stream.size(), 1u);
  EXPECT_EQ(stream[0], 0xAA);
}

TEST(SriEngineeringFrameParser, PublishesCompleteFrameBeforePartialNextHeader)
{
  const std::array<float, 6> values_in{{1.5f, -2.5f, 3.5f, -4.5f, 5.5f, -6.5f}};
  const std::array<unsigned char, 31> frame = makeEngineeringFrame(values_in);

  std::vector<unsigned char> stream(frame.begin(), frame.end());
  stream.push_back(0xAA);
  stream.push_back(0x55);
  stream.push_back(0x00);

  std::array<double, 6> values_out{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  ASSERT_TRUE(TCPClient::extractLatestEngineeringFrame(stream, values_out));
  for (std::size_t i = 0; i < values_in.size(); ++i)
  {
    EXPECT_FLOAT_EQ(static_cast<float>(values_out[i]), values_in[i]);
  }
  ASSERT_EQ(stream.size(), 3u);
  EXPECT_EQ(stream[0], 0xAA);
  EXPECT_EQ(stream[1], 0x55);
  EXPECT_EQ(stream[2], 0x00);
}
