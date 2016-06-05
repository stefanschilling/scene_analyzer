#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cob_people_detection_msgs/DetectionArray.h>

#include <cob_people_detection_msgs/ColorDepthImageArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "scene_analyzer/stamped_string.h"

#include <fstream>
#include <time.h>

std::string set="none";
int total_received_msgs=0;
int total_detections=0;
int total_correct_recognitions=0;
int set_messages=0;
int set_detections=0;
int set_correct_recognitions=0;
bool write_block=true;

std::vector<int> recognitions_per_set,messages_per_set,detections_per_set;
std::vector<std::string> published_set, set_published_files, set_recognized_labels;
std::vector<std::vector<std::string> > recognized_labels, published_files;

void inputCallback(const cob_people_detection_msgs::DetectionArray::ConstPtr& recognitions, const scene_analyzer::stamped_string::ConstPtr& set_path, const scene_analyzer::stamped_string::ConstPtr& file_name)
{
    std::string set_path_temp = set_path->data.c_str();
    unsigned substring = set_path_temp.find_last_of("/");
    //check if published set is still the same
    if (set != set_path_temp.substr(substring+1))
    {
        if (set_detections>0) std::cout << "percentage right recognition of " << set << ": \n" << set_correct_recognitions << "/" << set_detections << ": " << (float)set_correct_recognitions/set_detections << std::endl; 
        //if published set changed, change string to next set and save current detection and recognition.
        if(set != "none")
        {
            published_set.push_back(set);
            detections_per_set.push_back(set_detections);
            recognitions_per_set.push_back(set_correct_recognitions);
            messages_per_set.push_back(set_messages);
            recognized_labels.push_back(set_recognized_labels);
            published_files.push_back(set_published_files);
            
            set_detections = 0;
            set_correct_recognitions = 0;
            set_messages =0;
            set_recognized_labels.clear();
            set_published_files.clear();
        } 
        set = set_path_temp.substr(substring+1);
    }
    
    if(set != "none") set_messages++;
    if (set_published_files.size() == 0 || file_name->data != set_published_files[set_published_files.size()-1])
    {
        set_published_files.push_back(file_name->data.c_str());
        //after finding what set is published, check if the face was recognized correctly
        if (recognitions->detections.size() == 1)
        {
            //increment counter for detected faces
            set_detections++;
            std::string label = recognitions->detections[0].label;
            set_recognized_labels.push_back(label);
            //check label against published set, increment counter for correct recognition.
            if (label == set || label == set+"_synth")
            {
                set_correct_recognitions++;
                return;
            }
            else if (label == "set_6" && set == "set_06") set_correct_recognitions++;
            //^special case for mistyped set_6 in one of the early tdata files
        }
        else if (recognitions->detections.size() > 1) 
        {
            set_recognized_labels.push_back("Error: Multiple Detections");
            std::cout << "multiple detections in this message!" << std::endl;
        }
        else
        {
            set_recognized_labels.push_back("Error: No Detection");
            std::cout << "no detection in received message!" << std::endl;
        }
    }
    else
    {
        if (set_detections>0) std::cout << "percentage right recognition of " << set << ": \n" << set_correct_recognitions << "/" << set_detections << ": " << (float)set_correct_recognitions/set_detections << std::endl; 
        published_set.push_back(set);
        detections_per_set.push_back(set_detections);
        recognitions_per_set.push_back(set_correct_recognitions);
        messages_per_set.push_back(set_messages);
        recognized_labels.push_back(set_recognized_labels);
        published_files.push_back(set_published_files);
        std::cout<< "msg per set size after pushback " << messages_per_set.size() << std::endl;
        
        int total_det, total_rec, total_msg;
        total_det=total_rec=total_msg = 0;
        std::string path = "face_rec_";
        //get date
        time_t now;
        char date_c[24];
        now = time(NULL);
        if (now != -1) strftime(date_c, 24, "%d_%m_%Y_%H_%M_%S", gmtime(&now));
        std::string date(date_c);
        date.append(".txt");
        path.append(date);
        //create date-tagged txt file as detailed recognition output
        if (messages_per_set.size()!=0)
        {
            std::cout << "finish and reset" << std::endl;
            std::cout << "printing " << messages_per_set.size() << " sets " << std::endl;
            std::cout << "published_set.size() ? " << published_set.size() << std::endl;
            //create timestamped output txt and reset counters
            std::ofstream output_textfile("output.txt", std::ios_base::app);
            for (int i=0; i<published_set.size(); i++)
            {
                std::cout << " trying to write stuff - published set entry: " << i << std::endl;
                if (messages_per_set[i] >0)
                {   
                    std::cout << "messages per set > 0 - should be writing stuff" <<std::endl;
                    total_det += detections_per_set[i];
                    total_rec += recognitions_per_set[i];
                    total_msg += messages_per_set[i];
                    output_textfile << published_set[i] << " - Messages: " << messages_per_set[i] << ", Detected Faces: " << detections_per_set[i] << ", correctly recognized Faces: " << recognitions_per_set[i] << "\n";
                    if (detections_per_set[i]>0) output_textfile << "Percentage correct: " << (float) recognitions_per_set[i]/detections_per_set[i] << "/n";
                    output_textfile << "Image | Recognized as \n";
                    for (int j=0; j<recognized_labels[i].size();j++)
                    {
                        output_textfile << published_files[i][j] << " | " << recognized_labels[i][j]<< "\n"; 
                    }
                }
                else std::cout << "empty entry msgs per set entry (0) " << std::cout;
            }
            
            output_textfile << "\n \n Total Messages: " << total_msg << "\n Total Detection: " << total_det << "\n Total Correct Recognitions: " << total_rec << "\n";
            output_textfile << "Total Percentage correct: " << (float) total_rec / total_det;
            output_textfile.close();
            recognized_labels.clear();
            published_files.clear();
            published_set.clear();
            messages_per_set.clear();
            detections_per_set.clear();
            recognitions_per_set.clear();
            set_detections = 0;
            set_correct_recognitions = 0;
            set_messages =0;
            set_recognized_labels.clear();
            set_published_files.clear();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::DetectionArray, scene_analyzer::stamped_string, scene_analyzer::stamped_string> >* sync_input_;

    message_filters::Subscriber<cob_people_detection_msgs::DetectionArray> det_msg_sub;
    message_filters::Subscriber<scene_analyzer::stamped_string> set_msg_sub;
    message_filters::Subscriber<scene_analyzer::stamped_string> file_name_sub;
    
    det_msg_sub.subscribe(n, "/cob_people_detection/face_recognizer/face_recognitions", 1);
    set_msg_sub.subscribe(n, "/set_path", 1);
    file_name_sub.subscribe(n, "/file_name",1);
    
    sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::DetectionArray, scene_analyzer::stamped_string, scene_analyzer::stamped_string> >(3);
    sync_input_->connectInput(det_msg_sub, set_msg_sub, file_name_sub);
    sync_input_->registerCallback(boost::bind(&inputCallback, _1, _2, _3));
    ros::spin();
}
