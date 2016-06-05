#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include <boost/thread/mutex.hpp>
#include "boost/filesystem/path.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/filesystem.hpp"
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>

#include <fstream>

#include "scene_analyzer/stamped_string.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class scene_publisher
{
	public:

	// Constructor
	scene_publisher()
	{
		this->persp=1;
		this->shot=0;
        this->file="set_01";
        this->total_published_msgs=0;
        
		//file="-1";
		//nh.param("/cob_people_detection/face_recognizer/file",file,file);
		//std::cout<<"input file: "<<file<<std::endl;

		scene_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points",1);
		img_pub_ = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_color",1);

		set_pub_ = nh.advertise<scene_analyzer::stamped_string>("set_path",1);
		file_name_pub_ = nh.advertise<scene_analyzer::stamped_string>("file_name",1);
	}

	// Destructor
	~scene_publisher()
	{
	}

	void process()
	{
        
		pc.clear();
		path = "/home/stefan/rgbd_db/";

		std::stringstream jpg_stream,depth_stream, set_path_stream, file_name_stream;

		set_path_stream <<path.c_str()<<file.c_str();
		set_path.data = set_path_stream.str();
		file_name_stream << std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot;
		file_name.data = file_name_stream.str();

		depth_stream<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_d.xml";
		jpg_stream<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_c.bmp";
        cv::Mat img,dm;
        //dm=cv::Mat(640,480,CV_64FC1);
		//std::cout<<jpg_stream.str()<<"\n "<<depth_stream.str()<<std::endl;
        if (file != "derp")
        {
            //read bmp image
            img=cv::imread(jpg_stream.str().c_str());
            //img=cv::imread("/home/stefan/rgbd_db/007_2_c.bmp");
            img.convertTo(img,CV_8UC3);
            cv::resize(img,img,cv::Size(640,480));
            //std::cout << "img read... size: " << img.size() << " rows: " << img.rows << " cols: " << img.cols << std::endl;

            //read depth xml
            cv::FileStorage fs_read2(depth_stream.str().c_str(),cv::FileStorage::READ);
            fs_read2["depth"]>> dm;
            fs_read2.release();
            //std::cout << "depthmap read... size: " << dm.size() << " rows: " << dm.rows << " cols: " << dm.cols << std::endl;
        }
        else
        {
            std::cout << "Dummy img and dm created" << std::endl;
            dm=cv::Mat::zeros(480,640,CV_64FC1);
            img=cv::Mat::zeros(480,640,CV_8UC3);
            file_name.data = "derp";
        }

		//set parameters
		pc.width=640;
		pc.height=480;
		//cam mat found in scene publisher
		cv::Mat cam_mat =(cv::Mat_<double>(3,3)<< 524.90160178307269,0.0,320.13543361773458,0.0,525.85226379335393,240.73474482242005,0.0,0.0,1.0);
		//I don't know how they were determined. AAU was unable to locate the kinect used for this, so could not help solve the issue.
		//However, these values are very close to those found for the kinect rgb camera at nicolas.burrus.name/index.php/Research/KinectCalibration, so they should give good results if the deviation between devices is not great.

		// compensate for kinect offset
		cv::Vec3f rvec,tvec;
		cv::Mat dist_coeff;

		tvec=cv::Vec3f(0.03,0.0,0.0);
		rvec=cv::Vec3f(0.0,0.0,0);
		dist_coeff=cv::Mat::zeros(1,5,CV_32FC1);
		cv::Mat pc_trans=cv::Mat::zeros(640,480,CV_64FC1);

		//convert depth to xyz
		pcl::PointXYZRGB pt;
		int failctr=0;
		for(int r=0;r<dm.rows;r++)
		{
			for(int c=0;c<dm.cols;c++)
			{
				//estimate of point
				cv::Point3f pt_3f;
				pt_3f.x=(c-320);
				pt_3f.y=(r-240);
				pt_3f.z=525;

				double nrm=cv::norm(pt_3f);
				pt_3f.x/=nrm;
				pt_3f.y/=nrm;
				pt_3f.z/=nrm;

				pt_3f=pt_3f*dm.at<float>(r,c);
				//pt_3f=pt_3f*dm.at<double>(r,c);
				std::vector<cv::Point3f> pt3_vec;

				pt3_vec.push_back(pt_3f);
				std::vector<cv::Point2f> pt2_vec;

				cv::projectPoints(pt3_vec,rvec,tvec,cam_mat,dist_coeff,pt2_vec);

				int x_t,y_t;
				x_t=round(pt2_vec[0].x);
				y_t=round(pt2_vec[0].y);
				if(x_t<0 ||x_t>640 ||y_t<0 ||y_t>480)
					{
					failctr++;
					continue;
					}
				//pc_trans.at<double>(y_t,x_t)=dm.at<double>(r,c);
                pc_trans.at<float>(y_t,x_t)=dm.at<float>(r,c);

			}
		}

		//std::cout << "pc_trans created... points dropped: " << failctr << ", size: " << pc_trans.size() << " rows: " << pc_trans.rows << " cols: " << pc_trans.cols << std::endl;
		//std::cout << "start iterating to add points..." << std::endl;
		for(int j =0; j< dm.rows;j++)
		{
			for(int i =0; i< dm.cols; i++)
			{
				//create points from new depthmap pc_trans
				cv::Point3f pt_;
				pt_.x=(i-320);
				pt_.y=(j-240);
				pt_.z=525;

				double nrm=cv::norm(pt_);
				pt_.x/=nrm;
				pt_.y/=nrm;
				pt_.z/=nrm;

				//pt_=pt_*pc_trans.at<float>(j,i);
				pt_=pt_*pc_trans.at<float>(j,i);

				pt.x = pt_.x;
				pt.y = pt_.y;
				pt.z = pt_.z;

				//add color to points, not shifted
				uint32_t rgb = (static_cast<uint32_t>(img.at<cv::Vec3b>(j,i)[0]) << 0 |static_cast<uint32_t>(img.at<cv::Vec3b>(j,i)[1]) << 8 | static_cast<uint32_t>(img.at<cv::Vec3b>(j,i)[2]) << 16);
				pt.rgb = *reinterpret_cast<float*>(&rgb);
				pc.points.push_back(pt);
			}
		}
		cv_bridge::CvImage cv_ptr;
		cv_ptr.image = img;
		cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
		out_img = *(cv_ptr.toImageMsg());
		pcl::toROSMsg(pc,out_pc2);
    }

	void publish()
	{
		out_pc2.header.frame_id="/camera/";
		out_pc2.header.stamp = ros::Time::now();
		scene_pub_.publish(out_pc2);

		out_img.header.frame_id="/camera/";
		out_img.header.stamp = ros::Time::now();
		img_pub_.publish(out_img);
        
        set_path.header.stamp = ros::Time::now();
		set_pub_.publish(set_path);
		
        file_name.header.stamp =ros::Time::now();
        file_name_pub_.publish(file_name);
        total_published_msgs++;
	}
    
    void write()
    {
        
    }
    

	ros::NodeHandle nh;
	int persp;
	int shot;
    std::string file;
    int total_published_msgs;

	protected:
		ros::Publisher scene_pub_;
		ros::Publisher img_pub_;
		sensor_msgs::PointCloud2 out_pc2;
		pcl::PointCloud<pcl::PointXYZRGB> pc;
		sensor_msgs::Image out_img;
		std::string path;
        
		ros::Publisher set_pub_;
		scene_analyzer::stamped_string set_path;

		ros::Publisher file_name_pub_;
		scene_analyzer::stamped_string file_name;
};



int main (int argc, char** argv)
{
	std::string line;
	std::vector<int> perspective_filter;
	//std::ifstream persp_file("/home/stefan/rgbd_db_tools/perspectives.txt");
    std::ifstream persp_file("/home/stefan/rgbd_db_tools/training_perspectives.txt");
	int filter_it=0;
	if (persp_file.is_open())
	{
		int persp;
        std::cout << "Perspectives used to check recognition: ";
		while (getline(persp_file, line))
		{
			std::cout << line << ", ";
			std::istringstream (line) >> persp;
			perspective_filter.push_back(persp);
		}
        std::cout << std::endl;

	}
    
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

	ros::init (argc, argv, "scene_publisher");

	scene_publisher sp;
	ros::Rate loop_rate(0.6);
	sp.persp=perspective_filter[filter_it];
    sp.file=set_filter[set_it];
    for (int i =0; i<5; i++)
    {
        ros::spinOnce ();
		loop_rate.sleep();
    }
    
	while (ros::ok())
	{
		sp.shot++;
		if(sp.shot==4)
		{
			sp.shot=1;
			filter_it++;
			if(filter_it < perspective_filter.size())
			{
				sp.persp=perspective_filter[filter_it];
			}
			//sp.persp++;
		}
		if(filter_it == perspective_filter.size())
		{
            //go to next set
            std::cout << "next set" << std::endl;
            filter_it=0;
            set_it++;
            sp.persp = perspective_filter[filter_it];
            if(set_it == set_filter.size()) 
            {
                std::cout << sp.total_published_msgs;
                std::cout << "end of set list, publish 1 more image to get output from analyzer, then break..." << std::endl;
                sp.persp = perspective_filter[0];
                sp.file = "derp";
                sp.shot = 1;
                sp.process();
                sp.publish();
                ros::spinOnce ();
                loop_rate.sleep();
                sp.publish();
                ros::spinOnce ();
                loop_rate.sleep();
                break;
            }            
            sp.file = set_filter[set_it];
			//loop_rate.sleep();
		}
        
		if(sp.persp==14)
		{
			loop_rate.sleep();
			break;
		}
		//std::cout<<"SET: " << set_filter[set_it] <<" / PERSPECTIVE:"<<sp.persp<<std::endl;
		sp.process();
		sp.publish();
        //sp.write();
		ros::spinOnce ();
		loop_rate.sleep();
	}
}
