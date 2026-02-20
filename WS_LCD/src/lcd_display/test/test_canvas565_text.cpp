#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/gfx/Canvas565.hpp"
#include "st7789_ros_wrapper/gfx/Color565.hpp"
#include "st7789_ros_wrapper/gfx/FontBitmap.hpp"

namespace st7789_ros_wrapper::gfx {
namespace {

std::size_t countColor(const Canvas565 & canvas, const Color565 color) {
  return static_cast<std::size_t>(std::count(canvas.pixels().begin(), canvas.pixels().end(), color));
}

std::size_t countGlyphBits(const FontBitmap::GlyphRows & rows) {
  std::size_t total = 0;
  for (const auto row : rows) {
    for (std::uint8_t bit = 0; bit < FontBitmap::kGlyphWidth; ++bit) {
      if ((row & (1U << bit)) != 0U) {
        ++total;
      }
    }
  }
  return total;
}

TEST(Canvas565TextTest, DrawCharSetsExpectedPixelCountAtScaleOne) {
  Canvas565 canvas(16, 16);
  canvas.clear(color::kBlack);

  const auto glyph = FontBitmap::glyphRows('A');
  const auto expected_pixels = countGlyphBits(glyph);

  ASSERT_TRUE(canvas.drawChar(0, 0, 'A', color::kWhite, 1));
  EXPECT_EQ(countColor(canvas, color::kWhite), expected_pixels);
}

TEST(Canvas565TextTest, DrawCharScalingExpandsPixelArea) {
  Canvas565 canvas(32, 32);
  canvas.clear(color::kBlack);

  const auto glyph = FontBitmap::glyphRows('A');
  const auto expected_pixels = countGlyphBits(glyph) * 4U;

  ASSERT_TRUE(canvas.drawChar(0, 0, 'A', color::kWhite, 2));
  EXPECT_EQ(countColor(canvas, color::kWhite), expected_pixels);
}

TEST(Canvas565TextTest, DrawTextHandlesNewlineAndCarriageReturn) {
  Canvas565 canvas(24, 24);
  canvas.clear(color::kBlack);

  ASSERT_TRUE(canvas.drawText(0, 0, "A\r\nB", color::kWhite, 1));

  bool found_first_line = false;
  bool found_second_line = false;

  for (std::uint16_t y = 0; y < canvas.height(); ++y) {
    for (std::uint16_t x = 0; x < canvas.width(); ++x) {
      const auto index = static_cast<std::size_t>(y) * canvas.width() + x;
      if (canvas.pixels()[index] != color::kWhite) {
        continue;
      }

      if (y < FontBitmap::kLineHeight) {
        found_first_line = true;
      }
      if (y >= FontBitmap::kLineHeight && y < FontBitmap::kLineHeight * 2U) {
        found_second_line = true;
      }
    }
  }

  EXPECT_TRUE(found_first_line);
  EXPECT_TRUE(found_second_line);
}

TEST(Canvas565TextTest, DrawTextRejectsInvalidInputsWithoutMutatingCanvas) {
  Canvas565 canvas(12, 12);
  canvas.clear(color::kBlack);
  const auto baseline = canvas.pixels();

  EXPECT_FALSE(canvas.drawText(0, 0, "", color::kWhite, 1));
  EXPECT_FALSE(canvas.drawText(0, 0, "Hi", color::kWhite, 0));
  EXPECT_EQ(canvas.pixels(), baseline);
}

TEST(Canvas565TextTest, DrawCharUsesQuestionMarkFallbackForNonAscii) {
  Canvas565 fallback_canvas(16, 16);
  fallback_canvas.clear(color::kBlack);
  ASSERT_TRUE(fallback_canvas.drawChar(0, 0, static_cast<char>(0xFF), color::kWhite, 1));

  Canvas565 question_canvas(16, 16);
  question_canvas.clear(color::kBlack);
  ASSERT_TRUE(question_canvas.drawChar(0, 0, '?', color::kWhite, 1));

  EXPECT_EQ(fallback_canvas.pixels(), question_canvas.pixels());
}

TEST(Canvas565TextTest, DrawCharClipsWhenPartiallyOutOfBounds) {
  Canvas565 canvas(8, 8);
  canvas.clear(color::kBlack);

  ASSERT_TRUE(canvas.drawChar(4, 0, '_', color::kWhite, 1));
  EXPECT_EQ(countColor(canvas, color::kWhite), 4U);
}

}  // namespace
}  // namespace st7789_ros_wrapper::gfx
