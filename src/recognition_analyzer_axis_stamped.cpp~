#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cob_people_detection_msgs/DetectionArray.h>

//#include <cob_people_detection_msgs/ColorDepthImageArray.h>
//#include <ros/package.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

std::string set="set_01";
int face_dets=0;
int right_labels_counter=0;
int total_det=0;
int total_rec=0;

void callback1(const std_msgs::String::ConstPtr& msg)
{
    std::string helper = msg->data.c_str();
    unsigned substring = helper.find_last_of("/");
    if (set != helper.substr(substring+1))
    {
        if (face_dets>0) std::cout << "percentage right recognition of " << set << ": \n" << right_labels_counter << "/" << face_dets << ": " << (float)right_labels_counter/face_dets << std::endl; 
        set = helper.substr(substring+1);
        total_det += face_dets;
        total_rec += right_labels_counter;
        std::cout << "========== totals so far ========== \n detected faces: " << total_det << "\n recognized faces: " << total_rec <<std::endl;
        right_labels_counter = 0;
        face_dets = 0;
    }
}

void labelCallback(const cob_people_detection_msgs::DetectionArray::ConstPtr& msg)
{
    if (msg->detections.size() == 1)
    {
        face_dets++;
        std::string label = msg->detections[0].label;
        label = label.substr(0, label.size()-2);
        if (label == set || label == set+"_synth")
        {
            right_labels_counter++;
            return;
        }
        else if (label == "set_6" && set == "set_06") right_labels_counter++;
        //^special case for mistyped set_6 in one of the early tdata files
    }
    else
    {
        std::cout << "no detection in received message!";
    }
}

int main(int argc, char **argv)
{
    //launch listener node
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    std::string set;
    //subscribe cob_people_detection recognitions and correct set name published along with heads
    ros::Subscriber label_sub = n.subscribe("/cob_people_detection/face_recognizer/face_recognitions", 1, labelCallback);
    ros::Subscriber path_sub = n.subscribe("/set_path", 1, callback1);
    ros::spin();
    
    return 0;
}
