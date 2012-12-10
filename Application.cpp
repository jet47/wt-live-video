#include "Application.hpp"
#include <Wt/WVideo>
#include <Wt/WLink>
#include "VideoResource.hpp"

Application::Application(int width, int height, double fps, const std::string& mimeType,
                         const std::shared_ptr<FrameSourceQueue>& frameSourceQueue, 
                         const std::shared_ptr<MediaStreamFactory>& mediaStreamFactory,
                         const Wt::WEnvironment& env) : 
    WApplication(env)
{
    setTitle("Live Video Demo");

    VideoResource* resource = new VideoResource(width, height, fps, mimeType, frameSourceQueue, mediaStreamFactory, this);

    Wt::WVideo* video = new Wt::WVideo(root());
    video->addSource(Wt::WLink(resource));
}
