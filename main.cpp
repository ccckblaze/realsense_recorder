// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

// FFmpeg switch
#define USE_FFMPEG 1;

// FFmpeg
#ifdef USE_FFMPEG
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}
#endif

// Boost
#include <boost/filesystem.hpp>

// std
#include <string>
#include <vector>

using namespace std;
using namespace cv;

const std::string windowName = "Recording";
const int dst_width = 640;
const int dst_height = 480;
const AVRational dst_fps = {30, 1};

bool record(rs::device* dev, std::string basePath, float timeLimit = 1 * 60 * 60);

int main(int argc, char* argv[])
{
#ifdef USE_FFMPEG
    // initialize FFmpeg library
    av_register_all();
#endif

    // get records folder
    std::string currentBinPath = boost::filesystem::path(argv[0]).remove_filename().string();
    std::cout << "Record at: " << currentBinPath << std::endl;

    // create realsense device
    rs::context ctx;
    rs::device * dev = ctx.get_device(0);

    // configure infrared stream to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);

    // configure depth stream in order to IR stream run properly
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);

    // open color stream
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);

    // disable IR emitter(for outdoor use)
    if (dev->supports_option(rs::option::r200_emitter_enabled))
    {
        int value = !dev->get_option(rs::option::r200_emitter_enabled);
        std::cout << "Setting emitter to " << value << std::endl;
        dev->set_option(rs::option::r200_emitter_enabled, value);
    }

    // enable auto exposure(for outdoor usage)
    if (dev->supports_option(rs::option::r200_lr_auto_exposure_enabled))
    {
        int value = !dev->get_option(rs::option::r200_lr_auto_exposure_enabled);
        std::cout << "Setting auto exposure to " << value << std::endl;
        dev->set_option(rs::option::r200_lr_auto_exposure_enabled, value);
    }
    // display the image in GUI
    namedWindow(windowName, WINDOW_AUTOSIZE );
 
    // start streaming
    dev->start();

    // camera warmup - Dropped frames to allow stabilization
    for(int i = 0; i < 40; i++)
        dev->wait_for_frames();

    // loop
    while(true){ // ESC
        // record to a file per limit time
        if(!record(dev, currentBinPath/*, 60*/)){
            break;
        }
    }

    // stop streaming
    dev->stop();

    waitKey(0);

    return 0;
}

bool record(rs::device* dev, std::string basePath, float timeLimit){
    bool shouldContinue = true; 

    // create Folder
    boost::filesystem::create_directory(basePath + "/records");

    // get file name
    char fileName[255] = {0};
    char strTime[20] = {0};
    time_t now = time(NULL);
    strftime(strTime, 20, "%Y-%m-%d_%H:%M:%S", localtime(&now));
    sprintf(fileName, "%s/records/%s.avi", basePath.c_str(), strTime); 
    std::cout << "Start recording: " << fileName << std::endl;

    // get clock
    clock_t beginClock = clock();
   
#ifdef USE_FFMPEG
    // open output IO context
    AVFormatContext* outctx = nullptr;
    int ret = avformat_alloc_output_context2(&outctx, nullptr, nullptr, fileName);
    if (ret < 0) {
        std::cerr << "fail to avformat_alloc_output_context2(" << fileName<< "): ret=" << ret;
        return false;
    }
    // open output IO context
    ret = avio_open2(&outctx->pb, fileName, AVIO_FLAG_WRITE, nullptr, nullptr);
    if (ret < 0) {
        std::cerr << "fail to avio_open2: ret=" << ret;
        return false;
    }

    // create new video stream
    AVCodec* vcodec = avcodec_find_encoder(outctx->oformat->video_codec);
    AVStream* vstrm = avformat_new_stream(outctx, vcodec);
    if (!vstrm) {
        std::cerr << "fail to avformat_new_stream";
        return false;
    }
    avcodec_get_context_defaults3(vstrm->codec, vcodec);
    vstrm->codec->width = dst_width;
    vstrm->codec->height = dst_height;
    vstrm->codec->pix_fmt = vcodec->pix_fmts[0];
    vstrm->codec->time_base = vstrm->time_base = av_inv_q(dst_fps);
    vstrm->r_frame_rate = vstrm->avg_frame_rate = dst_fps;
    if (outctx->oformat->flags & AVFMT_GLOBALHEADER)
        vstrm->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    // open video encoder
    ret = avcodec_open2(vstrm->codec, vcodec, nullptr);
    if (ret < 0) {
        std::cerr << "fail to avcodec_open2: ret=" << ret;
        return false;
    }

    std::cout
        << "outfile: " << fileName << "\n"
        << "format:  " << outctx->oformat->name << "\n"
        << "vcodec:  " << vcodec->name << "\n"
        << "size:    " << dst_width << 'x' << dst_height << "\n"
        << "fps:     " << av_q2d(dst_fps) << "\n"
        << "pixfmt:  " << av_get_pix_fmt_name(vstrm->codec->pix_fmt) << "\n"
        << std::flush;

    // initialize sample scaler
    SwsContext* swsctx = sws_getCachedContext(
        nullptr, dst_width, dst_height, AV_PIX_FMT_BGR24,
        dst_width, dst_height, vstrm->codec->pix_fmt, SWS_BICUBIC, nullptr, nullptr, nullptr);
    if (!swsctx) {
        std::cerr << "fail to sws_getCachedContext";
        return false;
    }

    // allocate frame buffer for encoding
    AVFrame* frame = av_frame_alloc();
    std::vector<uint8_t> framebuf(avpicture_get_size(vstrm->codec->pix_fmt, dst_width, dst_height));
    avpicture_fill(reinterpret_cast<AVPicture*>(frame), framebuf.data(), vstrm->codec->pix_fmt, dst_width, dst_height);
    frame->width = dst_width;
    frame->height = dst_height;
    frame->format = static_cast<int>(vstrm->codec->pix_fmt);

    // write header
    avformat_write_header(outctx, nullptr); // VideoWriter
#elif
    VideoWriter outputVideo;

    // set video format
    outputVideo.open(fileName, CV_FOURCC('X', 'V', 'I', 'D'), 30, Size(640, 480), true);

    // check if opened
    if (!outputVideo.isOpened())
    {
        std::cout  << "Could not open the output video for write: " << std::endl;
        return false;
    }
#endif

    // loop
    Mat lastFrame;   // frame to encode
    double beginTimestamp = dev->get_frame_timestamp(rs::stream::color);
    double lastFrameTimestamp = beginTimestamp; 
#ifdef USE_FFMPEG
    int64_t frame_pts = 0;
    unsigned nb_frames = 0;
    int got_pkt = 0;
#endif
    // check time over
    while(((float)(clock() - beginClock) / CLOCKS_PER_SEC) < timeLimit){
        // check ESC
        if(waitKey(1) == 27){
            shouldContinue = false;
            std::cout << "ESCed" << std::endl;
            break;
        } 
        // wait for new frames
        dev->wait_for_frames();

        // calc frame skips
        double currentFrameTimestamp = dev->get_frame_timestamp(rs::stream::color);
        int frameSkips = int((currentFrameTimestamp - lastFrameTimestamp) / (1000.0 / 30));

        // creating OpenCV matrix from IR image
        Mat ir(Size(640, 480), CV_8UC1, (void*)dev->get_frame_data(rs::stream::infrared), Mat::AUTO_STEP);

        // apply Histogram Equalization
        equalizeHist( ir, ir );
        applyColorMap(ir, ir, COLORMAP_JET);

        // creating OpenCV matrix from IR image
        Mat color(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color), Mat::AUTO_STEP);
        // debug Info
        std::cout << "cft: " << currentFrameTimestamp << "; lft: " << lastFrameTimestamp 
                  << "; skip: " << frameSkips << std::endl;

        // write frame
        for(int i = 0; i < frameSkips; i++){
#ifdef USE_FFMPEG
            // convert cv::Mat(OpenCV) to AVFrame(FFmpeg)
            const int stride[] = { static_cast<int>(lastFrame.step[0]) };
            sws_scale(swsctx, &lastFrame.data, stride, 0, lastFrame.rows, frame->data, frame->linesize);
            frame->pts = frame_pts++;

            // encode video frame
            AVPacket pkt;
            pkt.data = nullptr;
            pkt.size = 0;
            av_init_packet(&pkt);
            ret = avcodec_encode_video2(vstrm->codec, &pkt, !shouldContinue ? nullptr : frame, &got_pkt);
            if (ret < 0) {
                std::cerr << "fail to avcodec_encode_video2: ret=" << ret << "\n";
                break;
            }
            if (got_pkt) {
                // rescale packet timestamp
                pkt.duration = 1;
                av_packet_rescale_ts(&pkt, vstrm->codec->time_base, vstrm->time_base);
                // write packet
                av_write_frame(outctx, &pkt);
                std::cout << nb_frames << '\r' << std::flush;  // dump progress
                ++nb_frames;
            }
            av_free_packet(&pkt);
#elif
            outputVideo.write(lastFrame);
#endif
        }

        imshow(windowName, ir);

        // regular time
	lastFrame = color.clone();
        lastFrameTimestamp = currentFrameTimestamp;
    }
#ifdef USE_FFMPEG
    // write trailer 
    av_write_trailer(outctx);
    std::cout << nb_frames << " frames encoded" << std::endl;

    // close ffmpeg io
    av_frame_free(&frame);
    avcodec_close(vstrm->codec);
    avio_close(outctx->pb);
    avformat_free_context(outctx);
 
#elif
    // close VideoWriter
    outputVideo.release();
#endif

    if(!shouldContinue){
        return false;
    }

    return true;
}

