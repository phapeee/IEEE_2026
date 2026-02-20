#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/app/FrameQueue.hpp"

namespace st7789_ros_wrapper::app {
namespace {

QueuedFrame makeFrame(const gfx::Color565 value) {
  QueuedFrame frame;
  frame.kind = QueuedFrameKind::kFullFrame;
  frame.pixels = {value};
  return frame;
}

TEST(FrameQueueTest, PushesAndPopsFramesInOrder) {
  FrameQueue queue(3U);
  ASSERT_TRUE(queue.push(makeFrame(1U)));
  ASSERT_TRUE(queue.push(makeFrame(2U)));

  QueuedFrame first;
  ASSERT_TRUE(queue.waitPop(first));
  ASSERT_EQ(first.pixels.size(), 1U);
  EXPECT_EQ(first.pixels[0], 1U);

  QueuedFrame second;
  ASSERT_TRUE(queue.waitPop(second));
  ASSERT_EQ(second.pixels.size(), 1U);
  EXPECT_EQ(second.pixels[0], 2U);
}

TEST(FrameQueueTest, DropsOldestFrameWhenQueueIsFull) {
  FrameQueue queue(2U);
  ASSERT_TRUE(queue.push(makeFrame(10U)));
  ASSERT_TRUE(queue.push(makeFrame(20U)));
  ASSERT_TRUE(queue.push(makeFrame(30U)));

  EXPECT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.droppedCount(), 1U);

  QueuedFrame first;
  ASSERT_TRUE(queue.waitPop(first));
  EXPECT_EQ(first.pixels[0], 20U);

  QueuedFrame second;
  ASSERT_TRUE(queue.waitPop(second));
  EXPECT_EQ(second.pixels[0], 30U);
}

TEST(FrameQueueTest, StopUnblocksWaitingConsumerAndRejectsPushes) {
  FrameQueue queue(1U);
  auto waiter = std::async(std::launch::async, [&queue]() {
      QueuedFrame frame;
      return queue.waitPop(frame);
    });

  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  queue.stop();

  EXPECT_FALSE(waiter.get());
  EXPECT_FALSE(queue.push(makeFrame(1U)));
  EXPECT_FALSE(queue.accepting());
}

}  // namespace
}  // namespace st7789_ros_wrapper::app
