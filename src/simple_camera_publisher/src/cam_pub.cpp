#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <cstdlib>
#include <string>
#include <vector>

#define CVWIN_OUT "cam_pub output"

/**
 * Image publisher for cv images
 * Publishes images at ~30fps
 */
int main(int argc, char** argv)
{
    //Initialize and set up ROS
    ros::init(argc, argv, "cam_pub");

    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image_raw", 1);

    std::string source;
    cv::VideoCapture cap;

    ros::Rate loop_rate(30);

    int empty_frame_count = 0;
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    // Open the video source
    bool cam_opened = false;
    if (nh.getParam("source", source) && source != "")
    {
        cap.open(source, cv::CAP_V4L2);
        ROS_INFO_STREAM("cam_pub: publishing using video source " << source << "...");
        cam_opened = true;
    }

    if (!cam_opened)
    {
        cap.open(0, cv::CAP_V4L2);
        ROS_WARN_STREAM("param 'source' not defined, using default camera");
    }

    // Check if video device can be opened with the given index
    if(!cap.isOpened())
    {
        ROS_ERROR_STREAM("video device cannot be opened");
        return 1;
    }else{
        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
	cap.set(cv::CAP_PROP_FOURCC, codec);
    }


    ////////////////
    // Parameters //
    ////////////////

    // Flip along horizontal axis?
    bool hflip = false;
    nh.getParam("hflip", hflip);
    if (hflip) ROS_INFO_STREAM(source << ": hflip active!");

    // Show output
    bool show_output = false;
    nh.getParam("show_output", show_output);
    if (show_output) ROS_INFO_STREAM(source << ": show_output active!");



    //////////////////
    // Publish Loop //
    //////////////////

    while (nh.ok())
    {
        cap >> frame;

        // Check if grabbed frame is actually full with some content
        if(!frame.empty())
        {
            empty_frame_count = 0;

            // Flip the image upside down
            if (hflip) cv::flip(frame, frame, -1);

            if(show_output)
            {
                cv::imshow(CVWIN_OUT, frame);
            }

            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }
        else
        {
            empty_frame_count++;
            if (empty_frame_count > 20)
            {
                ROS_ERROR_STREAM("Could not read input, closing cam pub");
                return 1;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
