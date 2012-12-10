#include "FrameSource.hpp"
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

namespace
{
    class FrameSourceQueueImpl : public FrameSourceQueue
    {
    public:
        FrameSourceQueueImpl(const std::shared_ptr<FrameSource>& frameSource, size_t size);
        ~FrameSourceQueueImpl();

        cv::Mat getDisplayFrame() override;

        void operator()();

    private:
        std::shared_ptr<FrameSource> frameSource_;
        boost::circular_buffer<cv::Mat> buffer_;
        boost::mutex mtx_;
        std::unique_ptr<boost::thread> thread_;
        volatile bool stop_;
    };

    FrameSourceQueueImpl::FrameSourceQueueImpl(const std::shared_ptr<FrameSource>& frameSource, size_t capacity) :
        frameSource_(frameSource),
        buffer_(capacity),
        stop_(false)
    {
        thread_.reset(new boost::thread(boost::ref(*this)));
    }

    FrameSourceQueueImpl::~FrameSourceQueueImpl()
    {
        stop_ = true;
        thread_->join();
    }

    cv::Mat FrameSourceQueueImpl::getDisplayFrame()
    {
        while (buffer_.empty())
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));

            if (stop_)
                return cv::Mat();
        }

        boost::mutex::scoped_lock lock(mtx_);
        return buffer_.front();
    }

    void FrameSourceQueueImpl::operator()()
    {
        try
        {
            for (;;)
            {
                cv::Mat frame = frameSource_->nextFrame().clone();

                {
                    boost::mutex::scoped_lock lock(mtx_);
                    buffer_.push_back(frame);
                }

                if (stop_ || (boost::this_thread::interruption_enabled() && boost::this_thread::interruption_requested()))
                    break;

                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
        }
        catch (const std::exception& e)
        {
            stop_ = true;
            std::cerr << e.what() << std::endl;
        }
    }
}

std::shared_ptr<FrameSourceQueue> createFrameSourceQueue(const std::shared_ptr<FrameSource>& frameSource, size_t size)
{
    return std::make_shared<FrameSourceQueueImpl>(frameSource, size);
}
