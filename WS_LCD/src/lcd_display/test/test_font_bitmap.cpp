#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/gfx/FontBitmap.hpp"

namespace st7789_ros_wrapper::gfx {
namespace {

TEST(FontBitmapTest, ReturnsExpectedGlyphRowsForPrintableAscii) {
  const auto glyph = FontBitmap::glyphRows('A');
  const std::array<std::uint8_t, 8> expected{0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00};
  EXPECT_EQ(glyph, expected);
}

TEST(FontBitmapTest, FallsBackToQuestionMarkForNonAscii) {
  const auto fallback = FontBitmap::glyphRows(static_cast<unsigned char>(0xFF));
  const auto question_mark = FontBitmap::glyphRows('?');
  EXPECT_EQ(fallback, question_mark);
}

TEST(FontBitmapTest, BitLookupUsesGlyphCoordinates) {
  const auto glyph = FontBitmap::glyphRows('!');

  EXPECT_TRUE(FontBitmap::bitIsSet(glyph, 3, 0));
  EXPECT_TRUE(FontBitmap::bitIsSet(glyph, 4, 0));
  EXPECT_FALSE(FontBitmap::bitIsSet(glyph, 0, 0));
  EXPECT_FALSE(FontBitmap::bitIsSet(glyph, 7, 7));
  EXPECT_FALSE(FontBitmap::bitIsSet(glyph, 8, 0));
  EXPECT_FALSE(FontBitmap::bitIsSet(glyph, 0, 8));
}

TEST(FontBitmapTest, PrintableAsciiRangeMatchesExpectedBounds) {
  EXPECT_FALSE(FontBitmap::isPrintable(31));
  EXPECT_TRUE(FontBitmap::isPrintable(32));
  EXPECT_TRUE(FontBitmap::isPrintable(126));
  EXPECT_FALSE(FontBitmap::isPrintable(127));
}

}  // namespace
}  // namespace st7789_ros_wrapper::gfx
