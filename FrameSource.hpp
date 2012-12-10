#pragma once

#ifndef __FRAME_SOURCE_HPP__
#define __FRAME_SOURCE_HPP__

#include <memory>
#include <opencv2/core/core.hpp>

class FrameSource
{
public:
    virtual ~FrameSource() {}

    virtual cv::Mat nextFrame() = 0;
};

class FrameSourceQueue
{
public:
    virtual ~FrameSourceQueue() {}

    virtual cv::Mat getDisplayFrame() = 0;
};

std::shared_ptr<FrameSourceQueue> createFrameSourceQueue(const std::shared_ptr<FrameSource>& frameSource, size_t size);

#endif // __FRAME_SOURCE_HPP__
