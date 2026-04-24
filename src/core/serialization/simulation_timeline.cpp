#include "simulation_timeline.hpp"

#include <algorithm>
#include <utility>

namespace phynity::serialization
{

SimulationTimeline::SimulationTimeline(size_t max_frames) : buffer_(max_frames)
{
}

void SimulationTimeline::push(PhysicsSnapshot snapshot)
{
    if (buffer_.empty())
    {
        return;
    }

    buffer_[head_] = std::move(snapshot);
    head_ = (head_ + 1) % buffer_.size();

    if (count_ < buffer_.size())
    {
        ++count_;
    }
}

const PhysicsSnapshot *SimulationTimeline::at(size_t index) const
{
    if (index >= count_)
    {
        return nullptr;
    }

    // index 0 = oldest available
    size_t actual = (head_ + buffer_.size() - count_ + index) % buffer_.size();
    return &buffer_[actual];
}

void SimulationTimeline::clear()
{
    head_ = 0;
    count_ = 0;
}

void SimulationTimeline::truncate(size_t new_size)
{
    if (new_size >= count_)
    {
        return;
    }

    const size_t old_count = count_;
    const size_t oldest = (head_ + buffer_.size() - old_count) % buffer_.size();

    count_ = new_size;
    head_ = (oldest + count_) % buffer_.size();
}

} // namespace phynity::serialization
