#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace st7789_ros_wrapper::app {

enum class QueuedFrameKind : std::uint8_t {
  kFullFrame = 0,
  kRegion = 1,
};

struct QueuedFrameRegion {
  std::uint16_t x{0};
  std::uint16_t y{0};
  std::uint16_t width{0};
  std::uint16_t height{0};
};

struct QueuedFrame {
  QueuedFrameKind kind{QueuedFrameKind::kFullFrame};
  QueuedFrameRegion region{};
  std::vector<gfx::Color565> pixels{};
};

class FrameQueue {
public:
  explicit FrameQueue(std::size_t capacity = 3U);

  bool push(QueuedFrame frame);
  bool waitPop(QueuedFrame & frame);
  void stop();
  void clear();

  bool accepting() const;
  std::size_t size() const;
  std::size_t droppedCount() const;

private:
  std::size_t capacity_{3U};
  bool accepting_{true};
  std::size_t dropped_count_{0U};
  std::deque<QueuedFrame> queue_{};
  mutable std::mutex mutex_{};
  std::condition_variable condition_{};
};

}  // namespace st7789_ros_wrapper::app
