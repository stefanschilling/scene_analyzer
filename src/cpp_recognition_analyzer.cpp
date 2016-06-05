#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cob_people_detection_msgs/DetectionArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "scene_analyzer/stamped_string.h"


int face_dets=0;
int right_labels_counter=0;
int total_det=0;
int total_rec=0;

class face_recognition_analyzer
{
	public:

	face_recognition_analyzer()
	{
        this->set="set_01";
        this->total_received_msgs=0;
        this->total_detections=0;
        this->total_correct_recognitions=0;
        recognitions_per_set.clear();
        detections_per_set.clear();
        messages_per_set.clear();
        published_file.clear();
        recognized_label.clear();
	}
  
  
	// Destructor
	~face_recognition_analyzer()
	{
	}
    
    void inputCallback(const cob_people_detection_msgs::DetectionArray::ConstPtr& recognitions, const scene_analyzer::stamped_string::ConstPtr& set_path)
    {
        int total_det, face_dets, total_rec, right_labels_counter;
        std::string set_path_temp = set_path->data.c_str();
        unsigned substring = set_path_temp.find_last_of("/");
        //check if published set is still the same
        if (set != set_path_temp.substr(substring+1))
        {
            if (face_dets>0) std::cout << "percentage right recognition of " << set << ": \n" << right_labels_counter << "/" << face_dets << ": " << (float)right_labels_counter/face_dets << std::endl; 
            //if published set changed, change string to next set and save current detection and recognition.
            set = set_path_temp.substr(substring+1);
            total_det += face_dets;
            total_rec += right_labels_counter;
            std::cout << "========== totals so far ========== \n detected faces: " << total_det << "\n recognized faces: " << total_rec <<std::endl;
            right_labels_counter = 0;
            face_dets = 0;
        }
        
        //after finding what set is published, check if the face was recognized correctly
        if (recognitions->detections.size() == 1)
        {
            //increment counter for detected faces
            face_dets++;
            std::string label = recognitions->detections[0].label;
            //check label against published set, increment counter for correct recognition.
            if (label == set || label == set+"_synth")
            {
                right_labels_counter++;
                return;
            }
            else if (label == "set_6" && set == "set_06") right_labels_counter++;
            //^special case for mistyped set_6 in one of the early tdata files
        }
        else if (recognitions->detections.size() > 1) std::cout << "multiple detections in this message!" << std::endl;
        else
        {
            std::cout << "no detection in received message!" << std::endl;
        }
    }
    protected:
    std::string set;
    int total_received_msgs;
    int total_detections;
    int total_correct_recognitions;
    
    std::vector<int>recognitions_per_set,messages_per_set,detections_per_set;
    std::vector<std::string>published_file,recognized_label;
    
    
};

int main(int argc, char **argv)
{
    //launch listener node
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::DetectionArray, scene_analyzer::stamped_string> >* sync_input_;

    message_filters::Subscriber<cob_people_detection_msgs::DetectionArray> det_msg_sub;
    message_filters::Subscriber<scene_analyzer::stamped_string> set_msg_sub;

    det_msg_sub.subscribe(n, "/cob_people_detection/face_recognizer/face_recognitions", 1);
    set_msg_sub.subscribe(n, "/set_path", 1);

    sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::DetectionArray, scene_analyzer::stamped_string> >(2);
    sync_input_->connectInput(det_msg_sub, set_msg_sub);
    
    face_recognition_analyzer analyzer;
    sync_input_->registerCallback(boost::bind(&analyzer.inputCallback, _1, _2));
    ros::spin();

    return 0;
}
