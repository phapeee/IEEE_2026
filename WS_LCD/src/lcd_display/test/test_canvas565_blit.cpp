#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/gfx/Canvas565.hpp"
#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace st7789_ros_wrapper::gfx {
namespace {

std::size_t indexOf(const Canvas565 & canvas, const std::uint16_t x, const std::uint16_t y) {
  return static_cast<std::size_t>(y) * canvas.width() + x;
}

TEST(Canvas565BlitTest, BlitsPixelsAtRequestedLocation) {
  Canvas565 canvas(4, 4);
  canvas.clear(color::kBlack);

  const std::vector<Color565> source = {
    color::kRed, color::kGreen,
    color::kBlue, color::kWhite,
  };

  ASSERT_TRUE(canvas.blit(1, 1, source.data(), 2, 2));
  EXPECT_EQ(canvas.pixels()[indexOf(canvas, 1, 1)], color::kRed);
  EXPECT_EQ(canvas.pixels()[indexOf(canvas, 2, 1)], color::kGreen);
  EXPECT_EQ(canvas.pixels()[indexOf(canvas, 1, 2)], color::kBlue);
  EXPECT_EQ(canvas.pixels()[indexOf(canvas, 2, 2)], color::kWhite);
  EXPECT_EQ(canvas.pixels()[indexOf(canvas, 0, 0)], color::kBlack);
}

TEST(Canvas565BlitTest, ClipsWhenSourceExtendsBeyondCanvas) {
  Canvas565 canvas(4, 4);
  canvas.clear(color::kBlack);

  const std::vector<Color565> source = {
    color::kRed, color::kGreen, color::kBlue,
    color::kWhite, color::kBlack, color::kRed,
  };

  ASSERT_TRUE(canvas.blit(3, 3, source.data(), 3, 2));
  EXPECT_EQ(canvas.pixels()[indexOf(canvas, 3, 3)], color::kRed);
  EXPECT_EQ(canvas.pixels()[indexOf(canvas, 2, 3)], color::kBlack);
}

TEST(Canvas565BlitTest, RejectsInvalidInputWithoutMutatingCanvas) {
  Canvas565 canvas(3, 3);
  canvas.clear(color::kBlue);
  const auto baseline = canvas.pixels();

  EXPECT_FALSE(canvas.blit(0, 0, nullptr, 1, 1));
  EXPECT_FALSE(canvas.blit(0, 0, baseline.data(), 0, 1));
  EXPECT_FALSE(canvas.blit(0, 0, baseline.data(), 1, 0));
  EXPECT_FALSE(canvas.blit(3, 0, baseline.data(), 1, 1));
  EXPECT_FALSE(canvas.blit(0, 3, baseline.data(), 1, 1));
  EXPECT_EQ(canvas.pixels(), baseline);
}

}  // namespace
}  // namespace st7789_ros_wrapper::gfx
