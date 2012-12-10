#pragma once

#ifndef __IO_BUFFER_HPP__
#define __IO_BUFFER_HPP__

#include <iosfwd>
#include <memory>

class IOBuffer
{
public:
    virtual ~IOBuffer() {}

    virtual void addData(const unsigned char* data, size_t size) = 0;
    virtual void writeToStream(std::ostream& st) = 0;
    virtual bool isFull() const = 0;
};

std::shared_ptr<IOBuffer> createOneVectorIOBuffer(size_t capacity);

#endif // __IO_BUFFER_HPP__
