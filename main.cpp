// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
#include <string>

using namespace std;
using namespace cv;

std::string windowName = "Recording";

bool record(rs::device* dev, std::string basePath, float timeLimit = 24 * 60 * 60){
    bool shouldContinue = true; 

    // Get file name
    char fileName[255] = {0};
    char strTime[20] = {0};
    time_t now = time(NULL);
    strftime(strTime, 20, "%Y-%m-%d_%H:%M:%S", localtime(&now));
    sprintf(fileName, "%s/records/%s.avi", basePath.c_str(), strTime); 
    std::cout << "Start recording: " << fileName << std::endl;

    // Get clock
    clock_t beginClock = clock();

    // VideoWriter
    VideoWriter outputVideo;

    // Set video format
    outputVideo.open(fileName, CV_FOURCC('X', 'V', 'I', 'D'), 30, Size(640, 480), true);

    // Check if opened
    if (!outputVideo.isOpened())
    {
        std::cout  << "Could not open the output video for write: " << std::endl;
        return false;
    }

    // Loop
    double beginTimestamp = dev->get_frame_timestamp(rs::stream::color);
    double lastFrameTimestamp = beginTimestamp; 
    Mat lastFrame;
    // Check time over
    while(((float)(clock() - beginClock) / CLOCKS_PER_SEC) < timeLimit){
        // Check ESC
        if(waitKey(1) == 27){
            shouldContinue = false;
            std::cout << "ESCed" << std::endl;
            break;
        } 
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
        //std::cout << frameSkips << std::endl;

        // Write frame
        for(int i = 0; frameSkips > 0 && i < frameSkips; i++){
            outputVideo.write(lastFrame);
        }

        imshow(windowName, ir);

        // regular time
	lastFrame = color.clone();
        lastFrameTimestamp = currentFrameTimestamp;
    }
  
    // Close VideoWriter
    outputVideo.release();

    if(!shouldContinue){
        return false;
    }

    return true;
}

int main(int argc, char* argv[])
{
    // Create records folder
    std::string currentBinPath = boost::filesystem::path(argv[0]).remove_filename().string();
    boost::filesystem::create_directory(currentBinPath);

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
    if (dev->supports_option(rs::option::r200_emitter_enabled))
    {
        int value = !dev->get_option(rs::option::r200_emitter_enabled);
        std::cout << "Setting emitter to " << value << std::endl;
        dev->set_option(rs::option::r200_emitter_enabled, value);
    }

    // Enable auto exposure(for outdoor usage)
    if (dev->supports_option(rs::option::r200_lr_auto_exposure_enabled))
    {
        int value = !dev->get_option(rs::option::r200_lr_auto_exposure_enabled);
        std::cout << "Setting auto exposure to " << value << std::endl;
        dev->set_option(rs::option::r200_lr_auto_exposure_enabled, value);
    }
    // Display the image in GUI
    namedWindow(windowName, WINDOW_AUTOSIZE );
 
    // Start streaming
    dev->start();

    // Camera warmup - Dropped frames to allow stabilization
    for(int i = 0; i < 40; i++)
        dev->wait_for_frames();

    // Loop
    while(true){ // ESC
        // record to a file per limit time
        if(!record(dev, currentBinPath/*, 60*/)){
            break;
        }
    }

    // Stop streaming
    dev->stop();

    waitKey(0);

    return 0;
}
