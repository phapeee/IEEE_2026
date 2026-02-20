#include "st7789_ros_wrapper/app/FrameQueue.hpp"

#include <algorithm>
#include <utility>

namespace st7789_ros_wrapper::app {

FrameQueue::FrameQueue(const std::size_t capacity)
: capacity_(std::max<std::size_t>(1U, capacity))
{
}

bool FrameQueue::push(QueuedFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!accepting_) {
    return false;
  }

  if (queue_.size() >= capacity_) {
    queue_.pop_front();
    ++dropped_count_;
  }

  queue_.push_back(std::move(frame));
  condition_.notify_one();
  return true;
}

bool FrameQueue::waitPop(QueuedFrame & frame) {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this]() {return !queue_.empty() || !accepting_;});

  if (queue_.empty()) {
    return false;
  }

  frame = std::move(queue_.front());
  queue_.pop_front();
  return true;
}

void FrameQueue::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  accepting_ = false;
  condition_.notify_all();
}

void FrameQueue::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
}

bool FrameQueue::accepting() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return accepting_;
}

std::size_t FrameQueue::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

std::size_t FrameQueue::droppedCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_count_;
}

}  // namespace st7789_ros_wrapper::app
