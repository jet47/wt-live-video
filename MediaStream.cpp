#include "MediaStream.hpp"
#include <cassert>

extern "C"
{
    #include "libavformat/avformat.h"
    #include "libavformat/avio.h"
    #include "libswscale/swscale.h"
}

namespace
{
    class FFmpegMediaStream : public MediaStream
    {
    public:
        FFmpegMediaStream(const std::string& mimeType, int width, int height, double fps, size_t innerBufferSize,
                          const std::shared_ptr<IOBuffer>& ioBuffer);
        ~FFmpegMediaStream();

        void addVideoFrame(const unsigned char* data, size_t step, int width, int height, PixelFormat_ pixFmt) override;

        int videoWidth() const override;
        int videoHeight() const override;

    private:
        void open(const std::string& mimeType, int width, int height, double fps, size_t innerBufferSize);
        void close();

        void createVideoStream(int width, int height, CodecID codec_id, double fps);

        void addVideoFrame(AVFrame* picture);

        static int writePacket(void* opaque, uint8_t* buf, int buf_size);

        AVFormatContext* context_;
        AVStream* videoStream_;
        uint8_t* videoOutBuf_;
        int videoOutBufSize_;
        AVFrame* writePicture_;
    };

    FFmpegMediaStream::FFmpegMediaStream(const std::string& mimeType, int width, int height, double fps, size_t innerBufferSize,
                                         const std::shared_ptr<IOBuffer>& ioBuffer) :
        MediaStream(ioBuffer),
        videoStream_(nullptr),
        videoOutBuf_(nullptr),
        videoOutBufSize_(0),
        writePicture_(nullptr)
    {
        try
        {
            open(mimeType, width, height, fps, innerBufferSize);
        }
        catch (...)
        {
            close();
            throw;
        }
    }

    FFmpegMediaStream::~FFmpegMediaStream()
    {
        close();
    }

    void FFmpegMediaStream::open(const std::string& mimeType, int width, int height, double fps, size_t innerBufferSize)
    {
        av_register_all();

        // auto detect the output format from the name
        AVOutputFormat* fmt = av_guess_format(nullptr, nullptr, mimeType.c_str());
        if (!fmt)
            throw std::runtime_error("FFmpegMediaStream : Can't find suitable output format");

        // allocate the output media context
        context_ = avformat_alloc_context();
        if (!context_)
            throw std::runtime_error("FFmpegMediaStream : Memory error");

        context_->oformat = fmt;

        // set some options
        context_->max_delay = static_cast<int>(0.7 * AV_TIME_BASE); // This reduces buffer underrun warnings with MPEG

        createVideoStream(width, height, fmt->video_codec, fps);

        uint8_t* buffer = static_cast<uint8_t*>(av_malloc(innerBufferSize));
        if (!buffer)
            throw std::runtime_error("FFmpegMediaStream : Memory error");

        context_->pb = avio_alloc_context(buffer, static_cast<int>(innerBufferSize), AVIO_FLAG_WRITE, this, nullptr, &writePacket, nullptr);
        if (!context_->pb)
        {
            av_free(buffer);
            throw std::runtime_error("FFmpegMediaStream : Memory error");
        }

        context_->pb->seekable = 0;

        if (avformat_write_header(context_, nullptr) < 0)
            throw std::runtime_error("FFmpegMediaStream : Write header error");
    }

    void FFmpegMediaStream::close()
    {
        // write the trailer, if any.  the trailer must be written
        // before you close the CodecContexts open when you wrote the
        // header; otherwise write_trailer may try to use memory that
        // was freed on av_codec_close()

        if (!context_)
            return;

        av_write_trailer(context_);

        // close each codec
        if (videoStream_)
        {
            if (writePicture_)
            {
                if (writePicture_->data[0])
                    av_free(writePicture_->data[0]);

                av_free(writePicture_);
                writePicture_ = nullptr;
            }

            avcodec_close(videoStream_->codec);

            if (videoOutBuf_)
            {
                av_free(videoOutBuf_);
                videoOutBuf_ = nullptr;
                videoOutBufSize_ = 0;
            }

            videoStream_ = nullptr;
        }

        // free the streams
        for (unsigned int i = 0; i < context_->nb_streams; ++i)
        {
            av_freep(&context_->streams[i]->codec);
            av_freep(&context_->streams[i]);
        }

        if (context_->pb)
        {
            av_free(context_->pb->buffer);
            av_free(context_->pb);
        }

        // free the stream
        av_free(context_);

        context_ = nullptr;
    }

    AVFrame* allocPicture(::PixelFormat pix_fmt, int width, int height, bool alloc)
    {
        AVFrame* picture = avcodec_alloc_frame();
        if (!picture)
            return nullptr;

        if(alloc)
        {
            int size = avpicture_get_size(pix_fmt, width, height);
            uint8_t* picture_buf = static_cast<uint8_t*>(av_malloc(size));

            if (!picture_buf)
            {
                av_free(picture);
                return nullptr;
            }

            avpicture_fill(reinterpret_cast<AVPicture*>(picture), picture_buf, pix_fmt, width, height);
        }

        return picture;
    }

    void FFmpegMediaStream::createVideoStream(int width, int height, CodecID codec_id, double fps)
    {
        ::PixelFormat pixFmt = PIX_FMT_YUV420P;
        double bitrate_scale = 1.0;

        // set a few optimal pixel formats for lossless codecs of interest..
        switch (codec_id)
        {
        case CODEC_ID_JPEGLS:
            pixFmt = PIX_FMT_BGR24;
            break;

        case CODEC_ID_HUFFYUV:
            pixFmt = PIX_FMT_YUV422P;
            break;

        case CODEC_ID_MJPEG:
        case CODEC_ID_LJPEG:
            pixFmt = PIX_FMT_YUVJ420P;
            bitrate_scale = 3.0;
            break;

        default:
            // good for lossy formats, MPEG, etc.
            pixFmt = PIX_FMT_YUV420P;
            break;
        }

        int bitRate = static_cast<int>(std::min(bitrate_scale * fps * width * height, std::numeric_limits<int>::max() / 2.0));

        videoStream_ = avformat_new_stream(context_, nullptr);
        if (!videoStream_)
            throw std::runtime_error("FFmpegMediaStream : Memory error");

        AVCodecContext* c = videoStream_->codec;

        c->codec_id = codec_id;
        c->codec_type = AVMEDIA_TYPE_VIDEO;

        // put sample parameters
        c->bit_rate = bitRate;

        unsigned long long lbit_rate = static_cast<unsigned long long>(bitRate);
        lbit_rate += (bitRate / 4);
        lbit_rate = std::min(lbit_rate, static_cast<unsigned long long>(std::numeric_limits<int>::max()));
        c->bit_rate_tolerance = static_cast<int>(lbit_rate);

        // resolution must be a multiple of two
        c->width = width;
        c->height = height;

        // took advice from
        // http://ffmpeg-users.933282.n4.nabble.com/warning-clipping-1-dct-coefficients-to-127-127-td934297.html
        c->qmin = 3;

        // time base: this is the fundamental unit of time (in seconds) in terms
        // of which frame timestamps are represented. for fixed-fps content,
        // timebase should be 1/framerate and timestamp increments should be
        // identically 1.

        int frame_rate = static_cast<int>(fps + 0.5);
        int frame_rate_base = 1;
        while (std::fabs(static_cast<double>(frame_rate) / frame_rate_base) - fps > 0.001)
        {
            frame_rate_base *= 10;
            frame_rate = static_cast<int>(fps * frame_rate_base + 0.5);
        }
        c->time_base.den = frame_rate;
        c->time_base.num = frame_rate_base;

        // emit one intra frame every twelve frames at most
        c->gop_size = 12;
        c->pix_fmt = pixFmt;

        if (c->codec_id == CODEC_ID_MPEG2VIDEO)
            c->max_b_frames = 2;
        if (c->codec_id == CODEC_ID_MPEG1VIDEO)
        {
            // Needed to avoid using macroblocks in which some coeffs overflow.
            // This does not happen with normal video, it just happens here as
            // the motion of the chroma plane does not match the luma plane.

            c->mb_decision = 2;
        }

        // some formats want stream headers to be separate
        if (context_->oformat->flags & AVFMT_GLOBALHEADER)
            c->flags |= CODEC_FLAG_GLOBAL_HEADER;

        // find the video encoder
        AVCodec* codec = avcodec_find_encoder(c->codec_id);
        if (!codec)
            throw std::runtime_error("FFmpegMediaStream : Can't find suitable video codec");

        // open the codec
        if (avcodec_open2(c, codec, nullptr) < 0)
            throw std::runtime_error("FFmpegMediaStream : Can't open video codec");

        // adjust time base for supported framerates
        if (codec && codec->supported_framerates)
        {
            AVRational req = {frame_rate, frame_rate_base};
            const AVRational* best = nullptr;
            AVRational best_error = {std::numeric_limits<int>::max(), 1};

            for(const AVRational* p = codec->supported_framerates; p->den != 0; ++p)
            {
                AVRational error= av_sub_q(req, *p);

                if (error.num < 0)
                    error.num *= -1;

                if (av_cmp_q(error, best_error) < 0)
                {
                    best_error = error;
                    best = p;
                }
            }

            c->time_base.den = best->num;
            c->time_base.num = best->den;
        }

        if (!(context_->oformat->flags & AVFMT_RAWPICTURE))
        {
            // allocate output buffer
            // XXX: API change will be done
            // buffers passed into lav* can be allocated any way you prefer,
            // as long as they're aligned enough for the architecture, and
            // they're freed appropriately (such as using av_free for buffers
            // allocated with av_malloc)

            videoOutBufSize_ = width * height * 4;
            videoOutBuf_ = static_cast<uint8_t*>(av_malloc(videoOutBufSize_));
        }

        // allocate the encoded raw picture
        writePicture_ = allocPicture(c->pix_fmt, c->width, c->height, true);
        if (!writePicture_)
            throw std::runtime_error("FFmpegMediaStream : Memory error");
    }

    int FFmpegMediaStream::videoWidth() const
    {
        return videoStream_->codec->width;
    }

    int FFmpegMediaStream::videoHeight() const
    {
        return videoStream_->codec->height;
    }

    void FFmpegMediaStream::addVideoFrame(const unsigned char* data, size_t step, int width, int height, PixelFormat_ pixFmt)
    {
        static const ::PixelFormat ffmpegFmts[] =
        {
            PIX_FMT_GRAY8, PIX_FMT_BGR24, PIX_FMT_RGB24, PIX_FMT_BGRA, PIX_FMT_RGBA
        };

        AVCodecContext* c = videoStream_->codec;

        SwsContext* swsCtx = sws_getContext(width, height, ffmpegFmts[pixFmt],
                                            c->width, c->height, c->pix_fmt,
                                            SWS_BICUBIC, nullptr, nullptr, nullptr);

        const uint8_t* srcSlice[] = { data, nullptr };
        int srcStride[] = { static_cast<int>(step), 0 };

        if (sws_scale(swsCtx, srcSlice, srcStride, 0, height, writePicture_->data, writePicture_->linesize) < 0)
            throw std::runtime_error("FFmpegMediaStream : Error while converting video frame");

        sws_freeContext(swsCtx);

        addVideoFrame(writePicture_);
    }

    void FFmpegMediaStream::addVideoFrame(AVFrame* picture)
    {
        AVCodecContext* c = videoStream_->codec;

        if (context_->oformat->flags & AVFMT_RAWPICTURE)
        {
            // raw video case. The API will change slightly in the near futur for that
            AVPacket pkt;
            av_init_packet(&pkt);

            pkt.flags |= AV_PKT_FLAG_KEY;
            pkt.stream_index = videoStream_->index;
            pkt.data = reinterpret_cast<uint8_t*>(picture);
            pkt.size = static_cast<int>(sizeof(AVPicture));

            if (av_interleaved_write_frame(context_, &pkt) != 0)
                throw std::runtime_error("FFmpegMediaStream : Error while writing video frame");
        }
        else
        {
            // encode the image
            int outSize = avcodec_encode_video(c, videoOutBuf_, videoOutBufSize_, picture);

            // if zero size, it means the image was buffered
            if (outSize > 0)
            {
                AVPacket pkt;
                av_init_packet(&pkt);

                if (c->coded_frame->pts != AV_NOPTS_VALUE)
                    pkt.pts = av_rescale_q(c->coded_frame->pts, c->time_base, videoStream_->time_base);

                if(c->coded_frame->key_frame)
                    pkt.flags |= AV_PKT_FLAG_KEY;

                pkt.stream_index = videoStream_->index;
                pkt.data = videoOutBuf_;
                pkt.size = outSize;

                // write the compressed frame in the media file
                if (av_interleaved_write_frame(context_, &pkt) != 0)
                    throw std::runtime_error("FFmpegMediaStream : Error while writing video frame");
            }
        }
    }

    int FFmpegMediaStream::writePacket(void* opaque, uint8_t* buf, int buf_size)
    {
        FFmpegMediaStream* thiz = (FFmpegMediaStream*)opaque;

        thiz->addData(buf, buf_size);

        return buf_size;
    }
}

std::shared_ptr<MediaStream> createFFmpegMediaStream(const std::string& mimeType, int width, int height, double fps, size_t innerBufferSize,
                                                     const std::shared_ptr<IOBuffer>& ioBuffer)
{
    return std::make_shared<FFmpegMediaStream>(mimeType, width, height, fps, innerBufferSize, ioBuffer);
}
