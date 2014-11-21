#ifndef IMAGE_SEQUENCE_PUBLISHER_NODE_H_
#define IMAGE_SEQUENCE_PUBLISHER_NODE_H_

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <image_transport/image_transport.h>

namespace bfs = boost::filesystem;

/**
 * This class publishes a sequence of images from a directory to a sensor_msgs/Image topic at the specfied frame 
 * rate of the video. The images are read using OpenCV and converted to the ros Image message using cv_bridge
 */
class ImageSequencePublisherNode
{
    public:
        /**
         * Constructor
         *
         * @param nh
         * ROS NodeHandle object
         */
        ImageSequencePublisherNode(ros::NodeHandle &nh);

        /**
         * Destructor
         */
        virtual ~ImageSequencePublisherNode();


    private:
        /**
         * Reads images in image_directory in alphabetical order and publishes them to output image topic
         * at frame_rate (set by using the parameter server)
         *
         * @param image_directory
         * Full path to directory containing images to be published
         *
         * @param frame_rate
         * Rate (per second) at which images should be published
         */
        void publish_images(const bfs::path &image_directory, double frame_rate);

    private:
        /**
         * ROS NodeHandle object
         */
        ros::NodeHandle nh_;

        /**
         * ImageTransport object
         */
        image_transport::ImageTransport it_;
        
        /**
         * Image publisher
         */
        image_transport::Publisher image_publisher_;
};

#endif
