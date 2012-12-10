#include "VideoResource.hpp"
#include <iostream>
#include <vector>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <Wt/Http/Request>
#include <Wt/Http/Response>

namespace
{
    class MediaStreamGenerator
    {
    public:
        MediaStreamGenerator(const std::shared_ptr<FrameSourceQueue>& frameSourceQueue, const std::shared_ptr<MediaStream>& mediaStream);
        ~MediaStreamGenerator();

        void writeToStream(std::ostream& st) { mediaStream_->writeToStream(st); }

        void operator()();

    private:
        std::shared_ptr<FrameSourceQueue> frameSourceQueue_;
        std::shared_ptr<MediaStream> mediaStream_;
        std::unique_ptr<boost::thread> thread_;
    };

    MediaStreamGenerator::MediaStreamGenerator(const std::shared_ptr<FrameSourceQueue>& frameSourceQueue,
                                               const std::shared_ptr<MediaStream>& mediaStream) :
        frameSourceQueue_(frameSourceQueue), mediaStream_(mediaStream)
    {
        thread_.reset(new boost::thread(boost::ref(*this)));
    }

    MediaStreamGenerator::~MediaStreamGenerator()
    {
        thread_->interrupt();
        thread_->join();
    }

    void MediaStreamGenerator::operator()()
    {
        try
        {
            for (;;)
            {
                while (mediaStream_->isBusy())
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1));

                cv::Mat frame = frameSourceQueue_->getDisplayFrame();

                MediaStream::PixelFormat_ pixFmt = frame.channels() == 1 ? MediaStream::GRAY :
                                                   frame.channels() == 3 ? MediaStream::BGR :
                                                   MediaStream::BGRA;

                mediaStream_->addVideoFrame(frame.ptr(), frame.step, frame.cols, frame.rows, pixFmt);

                if (boost::this_thread::interruption_enabled() && boost::this_thread::interruption_requested())
                    break;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}

class VideoResource::Impl
{
public:
    Impl(int width, int height, double fps, const std::string& mimeType,
         const std::shared_ptr<FrameSourceQueue>& frameSourceQueue,
         const std::shared_ptr<MediaStreamFactory>& mediaStreamFactory);

    void handleRequest(const Wt::Http::Request& request, Wt::Http::Response& response);

private:
    int width_;
    int height_;
    double fps_;
    std::string mimeType_;
    std::shared_ptr<FrameSourceQueue> frameSourceQueue_;
    std::shared_ptr<MediaStreamFactory> mediaStreamFactory_;
    std::vector<std::shared_ptr<MediaStreamGenerator>> mediaStreamGenerators_;
    boost::mutex mtx_;
};

VideoResource::Impl::Impl(int width, int height, double fps, const std::string& mimeType,
                          const std::shared_ptr<FrameSourceQueue>& frameSourceQueue,
                          const std::shared_ptr<MediaStreamFactory>& mediaStreamFactory) :
    width_(width), height_(height), fps_(fps), mimeType_(mimeType), frameSourceQueue_(frameSourceQueue), mediaStreamFactory_(mediaStreamFactory)
{
    mediaStreamGenerators_.reserve(10);
}

void VideoResource::Impl::handleRequest(const Wt::Http::Request& request, Wt::Http::Response& response)
{
    Wt::Http::ResponseContinuation* continuation = request.continuation();

    size_t ind;

    if (continuation)
        ind = boost::any_cast<size_t>(continuation->data());
    else
    {
        auto mediaStream = mediaStreamFactory_->create(mimeType_, width_, height_, fps_);
        auto streamGenerator = std::make_shared<MediaStreamGenerator>(frameSourceQueue_, mediaStream);

        {
            boost::mutex::scoped_lock lock(mtx_);
            ind = mediaStreamGenerators_.size();
            mediaStreamGenerators_.push_back(streamGenerator);
        }

        response.setMimeType(mimeType_);
    }

    {
        boost::mutex::scoped_lock lock(mtx_);
        mediaStreamGenerators_[ind]->writeToStream(response.out());
    }

    continuation = response.createContinuation();
    continuation->setData(ind);
}

///////////////////////////////////////////////////////////
// VideoResource

VideoResource::VideoResource(int width, int height, double fps, const std::string& mimeType,
                             const std::shared_ptr<FrameSourceQueue>& frameSourceQueue,
                             const std::shared_ptr<MediaStreamFactory>& mediaStreamFactory,
                             Wt::WObject* parent) :
    Wt::WResource(parent)
{
    impl_.reset(new Impl(width, height, fps, mimeType, frameSourceQueue, mediaStreamFactory));
}

VideoResource::~VideoResource()
{
    beingDeleted();
}

void VideoResource::handleRequest(const Wt::Http::Request& request, Wt::Http::Response& response)
{
    impl_->handleRequest(request, response);
}
