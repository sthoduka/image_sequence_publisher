/* image_sequence_publisher_node.cpp
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <image_sequence_publisher/image_sequence_publisher_node.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

ImageSequencePublisherNode::ImageSequencePublisherNode(ros::NodeHandle &nh) : nh_(nh), it_(nh)
{
    image_publisher_ = it_.advertise("output_image", 1);

    std::string input_directory;    

    if (!nh_.getParam("input_dir", input_directory))
    {
        ROS_ERROR("Input directory with images needs to be specified");
    }
    else
    {
        double frame_rate;
        nh_.param<double>("frame_rate", frame_rate, 30.0);
        publish_images(input_directory, frame_rate);
    }
}

ImageSequencePublisherNode::~ImageSequencePublisherNode()
{
}

void ImageSequencePublisherNode::publish_images(const bfs::path &image_directory, double frame_rate)
{
    if (!bfs::exists(image_directory))
    {
        ROS_ERROR("%s does not exist", image_directory.string().c_str());
        return;
    }
    if (!bfs::is_directory(image_directory))
    {
        ROS_ERROR("%s is not a directory", image_directory.string().c_str());
        return;
    }

    int number_of_frames = 0;

    std::set<std::string> sorted_paths;
    bfs::directory_iterator end;
    for (bfs::directory_iterator iter(image_directory); iter != end; ++iter)
    {
        if (!bfs::is_directory(*iter))
        {
            sorted_paths.insert(iter->path().string());
        }
    }

    ROS_INFO("Started publishing images");
    ros::Rate loop_rate(frame_rate);

    std::set<std::string>::iterator iter;
    for (iter = sorted_paths.begin(); iter != sorted_paths.end(); ++iter)
    {
        cv::Mat frame = cv::imread(*iter, CV_LOAD_IMAGE_COLOR);
        if (!frame.data)
        {
            ROS_WARN("%s is not an image file", iter->c_str());
            continue;
        }

        number_of_frames++;
        cv_bridge::CvImage frame_msg;
        frame_msg.encoding = sensor_msgs::image_encodings::BGR8;
        frame_msg.image = frame;
        image_publisher_.publish(frame_msg.toImageMsg());
        loop_rate.sleep();
    }

    ROS_INFO("Number of frames processed: %i", number_of_frames);
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "image_sequence_publisher");

    ros::NodeHandle n("~");

    ROS_INFO("[image_sequence_publisher] node started");

    ImageSequencePublisherNode image_sequence_publisher_node(n); 

    return 0;
}
