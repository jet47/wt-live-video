#pragma once

#ifndef __VIDEO_RESOURCE_HPP__
#define __VIDEO_RESOURCE_HPP__

#include <string>
#include <Wt/WResource>
#include "MediaStream.hpp"
#include "FrameSource.hpp"

class VideoResource : public Wt::WResource
{
public:
    VideoResource(int width, int height, double fps, const std::string& mimeType,
                  const std::shared_ptr<FrameSourceQueue>& frameSourceQueue,
                  const std::shared_ptr<MediaStreamFactory>& mediaStreamFactory,
                  Wt::WObject* parent = 0);
    ~VideoResource();

private:
    void handleRequest(const Wt::Http::Request& request, Wt::Http::Response& response);

    class Impl;
    std::unique_ptr<Impl> impl_;
};

#endif // __VIDEO_RESOURCE_HPP__
