#include "IOBuffer.hpp"
#include <cstring>
#include <iostream>
#include <vector>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

namespace
{
    class OneVectorIOBuffer : public IOBuffer
    {
    public:
        explicit OneVectorIOBuffer(size_t capacity);

        void addData(const unsigned char* data, size_t size) override;
        void writeToStream(std::ostream& st) override;
        bool isFull() const override;

    private:
        size_t capacity_;
        std::vector<unsigned char> buf_;
        boost::mutex mtx_;
    };

    OneVectorIOBuffer::OneVectorIOBuffer(size_t capacity) : capacity_(capacity)
    {
        buf_.reserve(capacity_);
    }

    void OneVectorIOBuffer::addData(const unsigned char* data, size_t size)
    {
        while (isFull())
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));

        boost::mutex::scoped_lock lock(mtx_);

        size_t old_size = buf_.size();
        buf_.resize(old_size + size);

        std::memcpy(&buf_[old_size], data, size);
    }

    void OneVectorIOBuffer::writeToStream(std::ostream& st)
    {
        while (buf_.empty())
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));

        boost::mutex::scoped_lock lock(mtx_);

        st.write(reinterpret_cast<const char*>(&buf_[0]), buf_.size());

        buf_.clear();
    }

    bool OneVectorIOBuffer::isFull() const
    {
        return buf_.size() > capacity_;
    }
}

std::shared_ptr<IOBuffer> createOneVectorIOBuffer(size_t capacity)
{
    return std::make_shared<OneVectorIOBuffer>(capacity);
}
