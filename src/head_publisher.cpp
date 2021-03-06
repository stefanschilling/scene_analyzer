#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem.hpp"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <cob_people_detection_msgs/DetectionArray.h>
#include <cob_people_detection_msgs/ColorDepthImageArray.h>
#include "scene_analyzer/stamped_string.h"


class head_publisher
{
	public:
	// Constructor
	head_publisher()
	{
		this->persp=1;
		this->shot=0;
		this->dummy_dm = cv::Mat::zeros(10,10,CV_32FC3);
		this->dummy_img = cv::Mat::zeros(10,10,CV_8UC3);
		this->file="no_set_specified";

		head_pub_ = nh.advertise<cob_people_detection_msgs::ColorDepthImageArray>("/cob_people_detection/head_detector/head_positions",1);
		set_pub_ = nh.advertise<scene_analyzer::stamped_string>("set_path",1);
		file_name_pub_ = nh.advertise<scene_analyzer::stamped_string>("file_name",1);
        end_pub_ = nh.advertise<scene_analyzer::stamped_string>("end_of_head_pub",1);
	}

	// Destructor
	~head_publisher()
	{
	}

	void read()
	{
		//path to image files
		path = "/home/stefan/rgbd_db_heads/";
		std::stringstream jpg_stream,depth_stream, set_path_stream, file_name_stream;
		set_path_stream <<path.c_str()<<file.c_str();
		set_path.data = set_path_stream.str();
		file_name_stream << std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot;
		file_name.data = file_name_stream.str();
		depth_stream<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_d.xml";
		jpg_stream<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_c.bmp";

		//read file if it exists, use dummy image otherwise
		std::ifstream f(jpg_stream.str().c_str());
		if (f.good()) 
		{
			f.close();
			//read bmp image
			img=cv::imread(jpg_stream.str().c_str());
			//img=cv::imread("/home/stefan/rgbd_db/007_2_c.bmp");
			img.convertTo(img,CV_8UC3);

			//read depth xml
			cv::FileStorage fs_read2(depth_stream.str().c_str(),cv::FileStorage::READ);
			fs_read2["depthmap"]>> dm;
			fs_read2.release();
			//std::cout << "depthmap read... size: " << dm.size() << " rows: " << dm.rows << " cols: " << dm.cols << std::endl;
		} 
		else
		{
			dm = dummy_dm;
			img = dummy_img;
		}
	}

	void publish()
	{
		if(img.rows>10)
		{
			//publish head images as head detections.
			//size of head detection = image size
			cob_people_detection_msgs::ColorDepthImageArray image_array;
			image_array.head_detections.resize(1);
			cv_bridge::CvImage cv_ptr;
			image_array.head_detections[0].head_detection.x = 0;
			image_array.head_detections[0].head_detection.y = 0;
			image_array.head_detections[0].head_detection.width = img.cols;
			image_array.head_detections[0].head_detection.height = img.rows;
			cv_ptr.image = dm;
			cv_ptr.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
			image_array.head_detections[0].depth_image = *(cv_ptr.toImageMsg());
			cv_ptr.image = img;
			cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
			image_array.head_detections[0].color_image = *(cv_ptr.toImageMsg());
			image_array.header.frame_id="/camera/";
			image_array.header.stamp = ros::Time::now();
            file_name.header.stamp = ros::Time::now();
            set_path.header.stamp = ros::Time::now();
			head_pub_.publish(image_array);
			file_name_pub_.publish(file_name);
			set_pub_.publish(set_path);
		}
		else if (file == "dummy/") 
        {
            set_path.header.stamp = ros::Time::now();
            set_path.data = "dummy";
            set_pub_.publish(set_path);
            end_msg.data = "end";
            end_msg.header.stamp = ros::Time::now();
            end_pub_.publish(end_msg);
        }
    }

	ros::NodeHandle nh;
	int persp;
	int shot;
	std::string file;

	protected:
	cv::Mat img,dm;
	ros::Publisher head_pub_,set_pub_, end_pub_,file_name_pub_;
	std::string path;
	scene_analyzer::stamped_string set_path, file_name, end_msg;
	cv::Mat dummy_img, dummy_dm;
};

int main (int argc, char** argv)
{
    //get perspectives to publish
	std::string line;
	std::vector<int> perspective_filter;
	std::ifstream persp_file("/home/stefan/rgbd_db_tools/perspectives.txt");
	int filter_it=0;
	if (persp_file.is_open())
	{
		int persp;
		std::cout << "Perspectives used to test recognition: ";
		while (getline(persp_file, line))
		{
			std::cout << line << ", ";
			std::istringstream (line) >> persp;
			perspective_filter.push_back(persp);
		}
		std::cout << std::endl;
	}
    
    //get sets to publish
	std::vector<std::string> set_filter;
	std::string set_list;
	std::cout << "Set List Number (1-20): ";
	std::cin >> set_list;
	set_list = "/home/stefan/rgbd_db_tools/sets/set"+set_list+".txt";
	std::ifstream set_file(set_list.c_str());
	int set_it=0;
	if (set_file.is_open())
	{
		std::string set;
		std::cout << "sets in this Set List: ";
		while (getline(set_file, line))
		{
			std::cout << line << ", ";
			//std::stringstream (line) >> set;
			set_filter.push_back(line);
		}
		std::cout << std::endl;
	}

    //launch publisher note, set publshing rate
	ros::init (argc, argv, "head_publisher");
	head_publisher hp;
	ros::Rate loop_rate(7);
	hp.persp=perspective_filter[filter_it];
	hp.file=set_filter[set_it];
    
    int last_shot, last_persp;
	for (int i=0; i<20; i++)
	{
		hp.publish();
		ros::spinOnce ();
		loop_rate.sleep();
	}
	std::cout <<"warmup done, start publishing head images."<<std::endl;
    
	while (ros::ok())
	{
		hp.shot++;
		if(hp.shot==4)
		{
			hp.shot=1;
			filter_it++;
			if(filter_it < perspective_filter.size())
			{
			hp.persp=perspective_filter[filter_it];
			}
			//hp.persp++;
		}
		if(filter_it == perspective_filter.size())
		{   
			//go to next set
			std::cout << "next set" << std::endl;
			filter_it=0;
			set_it++;
			hp.persp = perspective_filter[filter_it];
			if(set_it == set_filter.size()) 
			{
				std::cout << "end of set list, repeat last image..." << std::endl;
                hp.shot = last_shot;
                hp.persp=  last_persp;
                hp.read();
                for (int i=0; i<20; i++)
                {
                    hp.publish();
                    
                    ros::spinOnce ();
                    loop_rate.sleep();
                }
				break;
			}            
			hp.file = set_filter[set_it];
			//loop_rate.sleep();
		}

		if(hp.persp==14)
		{
			loop_rate.sleep();
			break;
		}
		//std::cout<<"SET: " << set_filter[set_it] <<" / PERSPECTIVE:"<<hp.persp<<std::endl;
        last_shot = hp.shot;
        last_persp = hp.persp;
		hp.read();
		hp.publish();
		//hp.write();
		ros::spinOnce ();
		loop_rate.sleep();
	}
}
