#include <vector>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>
#include "Application.hpp"

/////////////////////////////////////////////////////////////
// FaceDetectionSample

class FaceDetectionSample : public FrameSource
{
public:
    FaceDetectionSample(const std::string& videoFile, const std::string& cascadeFile);

    cv::Mat nextFrame() override;

private:
    cv::Mat readNextFrame();

    std::string videoFile_;
    cv::VideoCapture cap_;

    cv::gpu::CascadeClassifier_GPU d_cascade_;
    cv::Mat gray_;
    cv::gpu::GpuMat d_frame_;
    cv::gpu::GpuMat d_gray_;
    cv::gpu::GpuMat d_objects_;
    std::vector<cv::Rect> faces_;
};

FaceDetectionSample::FaceDetectionSample(const std::string& videoFile, const std::string& cascadeFile) :
    videoFile_(videoFile)
{
    CV_Assert( cap_.open(videoFile_) );

    CV_Assert( d_cascade_.load(cascadeFile) );

    d_cascade_.visualizeInPlace = false;
    d_cascade_.findLargestObject = false;
}

cv::Mat FaceDetectionSample::nextFrame()
{
    cv::Mat frame = readNextFrame();

    d_frame_.upload(frame);

    cv::gpu::cvtColor(d_frame_, d_gray_, cv::COLOR_BGR2GRAY);

    int count = d_cascade_.detectMultiScale(d_gray_, d_objects_);

    if (count == 0)
        faces_.clear();
    else
    {
        faces_.resize(count);
        cv::Mat facesMat(1, count, cv::DataType<cv::Rect>::type, &faces_[0]);
        d_objects_.colRange(0, count).download(facesMat);
    }

    std::for_each(faces_.begin(), faces_.end(), [&frame](cv::Rect rect)
        {
            cv::rectangle(frame, rect, CV_RGB(0, 255, 0), 3);
        });

    return frame;
}

cv::Mat FaceDetectionSample::readNextFrame()
{
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty())
    {
        CV_Assert( cap_.open(videoFile_) );
        cap_ >> frame;
    }

    return frame.clone();
}

/////////////////////////////////////////////////////////////
// MediaStreamFactory

class SampleMediaStreamFactory : public MediaStreamFactory
{
public:
    std::shared_ptr<MediaStream> create(const std::string& mimeType, int width, int height, double fps) override;
};

std::shared_ptr<MediaStream> SampleMediaStreamFactory::create(const std::string& mimeType, int width, int height, double fps)
{
    auto ioBuffer = createOneVectorIOBuffer(100 * 1024);
    return createFFmpegMediaStream(mimeType, width, height, fps, 10 * 1024, ioBuffer);
}

/////////////////////////////////////////////////////////////
// Application

std::shared_ptr<FrameSourceQueue> g_frameSourceQueue;

Wt::WApplication* createApplication(const Wt::WEnvironment& env)
{
    int width = 480;
    int height = 360;
    double fps = 15.0;
    std::string mimeType = "application/ogg";

    auto mediaStreamFactory = std::make_shared<SampleMediaStreamFactory>();

    return new Application(width, height, fps, mimeType, g_frameSourceQueue, mediaStreamFactory, env);
}

int main(int argc, char **argv)
{
    auto frameSource = std::make_shared<FaceDetectionSample>("browser.flv", "haarcascade_frontalface_alt.xml");
    g_frameSourceQueue = createFrameSourceQueue(frameSource, 100);

    int ret = WRun(argc, argv, &createApplication);

    g_frameSourceQueue.reset();

    return ret;
}
