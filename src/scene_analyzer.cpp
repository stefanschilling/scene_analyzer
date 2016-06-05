//this program determines the amount of heads and faces
//found in each picture published by the cpp_scene_pub
// and processed through cob_people detection for the topics
//"/cob_people_detection/face_detector/face_positions" and
//"/cob_people_detection/face_detector/head_positions"

//it creates an output file named heads_and_faces.txt

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cob_people_detection_msgs/DetectionArray.h>

#include <cob_people_detection_msgs/ColorDepthImageArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "scene_analyzer/stamped_string.h"

#include <fstream>

std::string path= " ";
std::string file= " ";
int total_received_msgs=0;
int msg_in_set=0;
int face_dets=0;
int head_dets=0;
int head_and_face_dets=0;
int face_in_set=0;
int head_in_set=0;
int head_and_face_in_set=0;

void inputCallback(const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& head_positions_msg, const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& face_positions_msg, const scene_analyzer::stamped_string::ConstPtr& set_msg, const scene_analyzer::stamped_string::ConstPtr& file_msg)
{
    std::ofstream output_textfile("/home/stefan/rgbd_db_tools/heads_and_faces.txt", std::ios_base::app);

    bool same_set = true;
    if (set_msg->data != path) same_set=false;
    file = file_msg->data;
    path = set_msg->data;
    if (file == "derp")
    {
        output_textfile << "\n totals - messages: " << msg_in_set << "heads: " << head_in_set << ", faces: " << face_in_set << ", heads with face: " << head_and_face_in_set<< "\n";
        head_dets += head_in_set;
        face_dets += face_in_set;
        head_and_face_dets += head_and_face_in_set;
        total_received_msgs += msg_in_set;
        head_in_set=face_in_set=head_and_face_in_set=msg_in_set=0;
        output_textfile << "\n \n";
        output_textfile << "----------------------------------\n";
        output_textfile << "Overall totals: \n messages: "<< total_received_msgs << "\n heads: " << head_dets << "\n faces: " << face_dets << "\n heads with face: " << head_and_face_dets; 
        std::cout << "Received this many messages: "<< total_received_msgs << std::endl;
        std::cout << "And found: " << std::endl;
        std::cout << "Heads: " << head_dets<<"\n";
        std::cout << "Faces: " << face_dets <<"\n";
        std::cout << "Heads with Faces: " << head_and_face_dets << std::endl;
        return;
    }
    
	//output_textfile.open("/home/stefan/rgbd_db_tools/heads_faces_features.txt");
    if (!same_set)
    {   
        if (msg_in_set>1) output_textfile << "\n totals - messages: " << msg_in_set << ", heads: " << head_in_set << ", faces: " << face_in_set << ", heads with face: " << head_and_face_in_set<< "\n"; 
        head_dets += head_in_set;
        face_dets += face_in_set;
        head_and_face_dets += head_and_face_in_set;
        total_received_msgs += msg_in_set;
        head_in_set=face_in_set=msg_in_set=head_and_face_in_set=0;
        output_textfile << "\n";
        output_textfile << " ----- ";
        output_textfile << "Set: " << path;
        output_textfile << " ----- \n";
    }
    msg_in_set++;
    int heads, faces, hf;
    heads=faces=hf=0;
    heads=head_positions_msg->head_detections.size();
    head_in_set += heads;
    for (int i =0; i<heads; i++)
    {
        faces=face_positions_msg->head_detections[i].face_detections.size();
        face_in_set += faces;
        if (faces==1)
        {
            head_and_face_in_set++;
            hf++;
        }
        if (faces >1 ) std::cout << "more than 1 face!!";
    }
    if (heads > 1) std::cout << "more than 1 head!!";

    output_textfile << file << " - heads: " << heads << ", faces: " << faces << ", heads with face: " << hf << std::endl;
    output_textfile.close();
}

int main(int argc, char **argv)
{
    std::ofstream output_textfile;
	output_textfile.open("heads_and_faces.txt");
	output_textfile.clear();
	output_textfile << "Head and Face Detection across Sets" <<std::endl;
	output_textfile << "============================================" << std::endl;
    output_textfile.close();
    //launch listener node
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::ColorDepthImageArray, cob_people_detection_msgs::ColorDepthImageArray,scene_analyzer::stamped_string,scene_analyzer::stamped_string> >* sync_input_;

    std::string set;
    message_filters::Subscriber<cob_people_detection_msgs::ColorDepthImageArray> head_msg_sub;
    message_filters::Subscriber<cob_people_detection_msgs::ColorDepthImageArray> face_msg_sub;
    message_filters::Subscriber<scene_analyzer::stamped_string> set_msg_sub;
    message_filters::Subscriber<scene_analyzer::stamped_string> file_msg_sub;
    head_msg_sub.subscribe(n, "/cob_people_detection/head_detector/head_positions", 1);
    face_msg_sub.subscribe(n, "/cob_people_detection/face_detector/face_positions", 1);
    set_msg_sub.subscribe(n, "/set_path",1);
    file_msg_sub.subscribe(n, "/file_name",1);

    sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::ColorDepthImageArray, cob_people_detection_msgs::ColorDepthImageArray, scene_analyzer::stamped_string, scene_analyzer::stamped_string> >(4);
    sync_input_->connectInput( head_msg_sub,face_msg_sub, set_msg_sub, file_msg_sub);
    sync_input_->registerCallback(boost::bind(&inputCallback, _1, _2,_3,_4));
    //writeText();
    ros::spin();
    
    return 0;
}
