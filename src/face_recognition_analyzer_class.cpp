#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cob_people_detection_msgs/DetectionArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "scene_analyzer/stamped_string.h"

class face_recognition_analyzer
{
    protected:
        std::string set;
        int total_received_msgs;
        int total_detections;
        int total_correct_recognitions;        
        int face_dets;
        int right_labels_counter;
        int total_det;
        int total_rec;

        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::DetectionArray, scene_analyzer::stamped_string> >* sync_input_;
        message_filters::Subscriber<cob_people_detection_msgs::DetectionArray> det_msg_sub;
        message_filters::Subscriber<scene_analyzer::stamped_string> set_msg_sub;

        std::vector<int>recognitions_per_set,messages_per_set,detections_per_set;
        std::vector<std::string>published_file,recognized_label;
        
    public:
	face_recognition_analyzer(ros::NodeHandle n)
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
        
        det_msg_sub.subscribe(n, "/cob_people_detection/face_recognizer/face_recognitions", 1);
        set_msg_sub.subscribe(n, "/set_path", 1);
        sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::DetectionArray, scene_analyzer::stamped_string> >(2);
        sync_input_->connectInput(det_msg_sub, set_msg_sub);
        sync_input_->registerCallback(boost::bind(&face_recognition_analyzer::inputCallback,this, _1, _2));
	}
  
	// Destructor
	~face_recognition_analyzer()
	{
	}
    
    void inputCallback(const cob_people_detection_msgs::DetectionArray::ConstPtr& recognitions, const scene_analyzer::stamped_string::ConstPtr& set_path)
    {
        
    }
};

int main(int argc, char **argv)
{
    //launch listener node
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    face_recognition_analyzer analyzer(n);

    ros::spin();

    return 0;
}
