#pragma once

#ifndef __MEDIA_STREAM_HPP__
#define __MEDIA_STREAM_HPP__

#include <string>
#include <memory>
#include "IOBuffer.hpp"

class MediaStream
{
public:
    enum PixelFormat_
    {
        GRAY,
        BGR,
        RGB,
        BGRA,
        RGBA
    };

    explicit MediaStream(const std::shared_ptr<IOBuffer>& ioBuffer) : ioBuffer_(ioBuffer) {}
    virtual ~MediaStream() {}

    virtual void addVideoFrame(const unsigned char* data, size_t step, int width, int height, PixelFormat_ pixFmt) = 0;

    virtual int videoWidth() const = 0;
    virtual int videoHeight() const = 0;

    void writeToStream(std::ostream& st) { ioBuffer_->writeToStream(st); }
    bool isBusy() const { return ioBuffer_->isFull(); }

protected:
    void addData(const unsigned char* data, size_t size) { ioBuffer_->addData(data, size); }

private:
    std::shared_ptr<IOBuffer> ioBuffer_;
};

std::shared_ptr<MediaStream> createFFmpegMediaStream(const std::string& mimeType, int width, int height, double fps, size_t innerBufferSize,
                                                     const std::shared_ptr<IOBuffer>& ioBuffer);

class MediaStreamFactory
{
public:
    virtual ~MediaStreamFactory() {}

    virtual std::shared_ptr<MediaStream> create(const std::string& mimeType, int width, int height, double fps) = 0;
};

#endif // __MEDIA_STREAM_HPP__
