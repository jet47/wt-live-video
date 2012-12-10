#pragma once

#ifndef __APPLICATION_HPP__
#define __APPLICATION_HPP__

#include <string>
#include <memory>
#include <Wt/WApplication>
#include "FrameSource.hpp"
#include "MediaStream.hpp"

class Application : public Wt::WApplication
{
public:
    Application(int width, int height, double fps, const std::string& mimeType,
                const std::shared_ptr<FrameSourceQueue>& frameSourceQueue,
                const std::shared_ptr<MediaStreamFactory>& mediaStreamFactory,
                const Wt::WEnvironment& env);
};

#endif // __GL_APPLICATION_HPP__
