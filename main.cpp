// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;

int main()
{
    // Create folder
    boost::filesystem::create_directory("records");

    // Get file name
    char fileName[255] = {0};
    time_t now = time(NULL);
    strftime(fileName, 255, "records/%Y-%m-%d_%H:%M:%S.avi", localtime(&now));
    std::cout << "Start recording: " << fileName << std::endl;

    // VideoWriter
    VideoWriter outputVideo;

    // Set video format
    outputVideo.open(fileName, CV_FOURCC('X', 'V', 'I', 'D'), 30, Size(640, 480), true);

    // Check if opened
    if (!outputVideo.isOpened())
    {
        std::cout  << "Could not open the output video for write: " << std::endl;
        return -1;
    }

    // Create realsense device
    rs::context ctx;
    rs::device * dev = ctx.get_device(0);

    // Configure Infrared stream to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);

    // We must also configure depth stream in order to IR stream run properly
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);

    // Open color stream
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);

    // Disable IR emitter(for outdoor use)
    //if (dev->supports_option(rs::option::r200_emitter_enabled))
    //{
    //    int value = !dev->get_option(rs::option::r200_emitter_enabled);
    //    std::cout << "Setting emitter to " << value << std::endl;
    //    dev->set_option(rs::option::r200_emitter_enabled, value);
    //}

    // Enable auto exposure(for outdoor usage)
    if (dev->supports_option(rs::option::r200_lr_auto_exposure_enabled))
    {
        int value = !dev->get_option(rs::option::r200_lr_auto_exposure_enabled);
        std::cout << "Setting auto exposure to " << value << std::endl;
        dev->set_option(rs::option::r200_lr_auto_exposure_enabled, value);
    }

    // Start streaming
    dev->start();

    // Camera warmup - Dropped frames to allow stabilization
    for(int i = 0; i < 40; i++)
        dev->wait_for_frames();

    // Loop
    double lastFrameTimestamp = dev->get_frame_timestamp(rs::stream::color);
    Mat lastFrame;
    while(waitKey(1) != 27){
        // Wait for new frames
        dev->wait_for_frames();

        // Calc frame skips
        double currentFrameTimestamp = dev->get_frame_timestamp(rs::stream::color);
        int frameSkips = int((currentFrameTimestamp - lastFrameTimestamp) / (1000.0 / 30));

        // Creating OpenCV matrix from IR image
        Mat ir(Size(640, 480), CV_8UC1, (void*)dev->get_frame_data(rs::stream::infrared), Mat::AUTO_STEP);

        // Apply Histogram Equalization
        equalizeHist( ir, ir );
        applyColorMap(ir, ir, COLORMAP_JET);

        // Creating OpenCV matrix from IR image
        Mat color(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color), Mat::AUTO_STEP);
        std::cout << frameSkips << std::endl;
        // Write frame
        for(int i = 0; frameSkips > 0 && i < frameSkips; i++){
            outputVideo.write(lastFrame);
        }

        // Display the image in GUI
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", ir);

        // regular time
	lastFrame = color.clone();
        lastFrameTimestamp = currentFrameTimestamp;
    }
  
    // Close VideoWriter
    outputVideo.release();

    // Stop streaming
    dev->stop();

    waitKey(0);

    return 0;
}
