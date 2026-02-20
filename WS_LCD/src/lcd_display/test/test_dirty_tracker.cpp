#include <cstddef>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/gfx/DirtyTracker.hpp"

namespace st7789_ros_wrapper::gfx {
namespace {

TEST(DirtyTrackerTest, TracksSingleRectangle) {
  DirtyTracker tracker(10, 8);

  ASSERT_TRUE(tracker.markRect(2, 3, 4, 2));
  ASSERT_TRUE(tracker.hasDirty());

  const auto dirty = tracker.dirtyRect();
  EXPECT_EQ(dirty.x, 2U);
  EXPECT_EQ(dirty.y, 3U);
  EXPECT_EQ(dirty.width, 4U);
  EXPECT_EQ(dirty.height, 2U);
  EXPECT_EQ(tracker.dirtyPixelCount(), 8U);
}

TEST(DirtyTrackerTest, ClipsRectanglesToCanvasBounds) {
  DirtyTracker tracker(10, 8);

  ASSERT_TRUE(tracker.markRect(8, 6, 5, 4));
  const auto dirty = tracker.dirtyRect();

  EXPECT_EQ(dirty.x, 8U);
  EXPECT_EQ(dirty.y, 6U);
  EXPECT_EQ(dirty.width, 2U);
  EXPECT_EQ(dirty.height, 2U);
}

TEST(DirtyTrackerTest, RejectsZeroOrOutOfBoundsRectangles) {
  DirtyTracker tracker(10, 8);

  EXPECT_FALSE(tracker.markRect(0, 0, 0, 2));
  EXPECT_FALSE(tracker.markRect(0, 0, 2, 0));
  EXPECT_FALSE(tracker.markRect(10, 0, 1, 1));
  EXPECT_FALSE(tracker.markRect(0, 8, 1, 1));
  EXPECT_FALSE(tracker.hasDirty());
}

TEST(DirtyTrackerTest, UnionsMultipleRectangles) {
  DirtyTracker tracker(10, 8);

  ASSERT_TRUE(tracker.markRect(1, 1, 2, 2));
  ASSERT_TRUE(tracker.markRect(4, 3, 3, 2));

  const auto dirty = tracker.dirtyRect();
  EXPECT_EQ(dirty.x, 1U);
  EXPECT_EQ(dirty.y, 1U);
  EXPECT_EQ(dirty.width, 6U);
  EXPECT_EQ(dirty.height, 4U);
}

TEST(DirtyTrackerTest, MarkAllCoversEntireCanvas) {
  DirtyTracker tracker(10, 8);

  tracker.markAll();
  ASSERT_TRUE(tracker.hasDirty());

  const auto dirty = tracker.dirtyRect();
  EXPECT_EQ(dirty.x, 0U);
  EXPECT_EQ(dirty.y, 0U);
  EXPECT_EQ(dirty.width, 10U);
  EXPECT_EQ(dirty.height, 8U);
  EXPECT_EQ(tracker.dirtyPixelCount(), 80U);
  EXPECT_DOUBLE_EQ(tracker.dirtyCoverage(), 1.0);
}

TEST(DirtyTrackerTest, ClearResetsState) {
  DirtyTracker tracker(10, 8);
  ASSERT_TRUE(tracker.markRect(1, 1, 2, 2));

  tracker.clear();

  EXPECT_FALSE(tracker.hasDirty());
  EXPECT_EQ(tracker.dirtyPixelCount(), 0U);
  EXPECT_DOUBLE_EQ(tracker.dirtyCoverage(), 0.0);
}

TEST(DirtyTrackerTest, CanvasResizeClearsDirtyAndUpdatesCoverage) {
  DirtyTracker tracker(10, 8);
  ASSERT_TRUE(tracker.markRect(0, 0, 4, 2));
  EXPECT_DOUBLE_EQ(tracker.dirtyCoverage(), 8.0 / 80.0);

  tracker.setCanvasSize(20, 10);
  EXPECT_FALSE(tracker.hasDirty());
  EXPECT_EQ(tracker.totalPixelCount(), static_cast<std::size_t>(200));

  ASSERT_TRUE(tracker.markRect(0, 0, 4, 2));
  EXPECT_DOUBLE_EQ(tracker.dirtyCoverage(), 8.0 / 200.0);
}

}  // namespace
}  // namespace st7789_ros_wrapper::gfx
