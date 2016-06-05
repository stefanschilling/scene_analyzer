//publish one pointcloud endlessly for testing purposes
//use frame /camera/ in rviz if no transform available
//Done: add transform to flip perspective horizontal, called the frame /map/ to make it the default when opening rviz (camera z-axis -> map x-axis)

//TODO, overall: find sources for applied methods, write down for use in writing thesis.

//TODO: separate cloud creation into one cloud per image
//TODO: initial transformation of faces to closely match each other
//TODO: implement method to identify matchable areas - trim background, don't include points of face occluded in either perspective.
//TODO: implement method for iterative matching of these areas
//TODO: create new img+dm from matched samples
//TODO: Review combined image by transforming it into point cloud again and publishing to rviz
//TODO: Add more samples to the combined image
//TODO: decide if the matching process should include any kind of weighting based on perspectives and/or order of matching (likely should give existing combination a higher trust than new additions?)

//TODO, non-priority: Why is only the last transform published while the ones before remain unknown?

#include <fstream>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>


#include "scene_analyzer/stamped_string.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class scene_publisher
{
	public:

	// Constructor
	scene_publisher()
	{
		this->persp=7;
		this->shot=1;
        this->file="set_01";
        this->total_published_msgs=0;
        
		//file="-1";
		//nh.param("/cob_people_detection/face_recognizer/file",file,file);
		//std::cout<<"input file: "<<file<<std::endl;

		scene_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points",1);
		img_pub_ = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_color",1);

        scene_pub_2_ = nh.advertise<sensor_msgs::PointCloud2>("/camera2/depth_registered/points",1);
		img_pub_2_ = nh.advertise<sensor_msgs::Image>("/camera2/rgb/image_color",1);

		set_pub_ = nh.advertise<scene_analyzer::stamped_string>("set_path",1);
		file_name_pub_ = nh.advertise<scene_analyzer::stamped_string>("file_name",1);
	}

	// Destructor
	~scene_publisher()
	{
	}

    void normalizePerspective(cv::Mat& dm, cv::Vec3f& nose)
    {
        Eigen::Vector3f nose_vec;
        //nose_vec << nose3d_.x, nose3d_.y, nose3d_.z;
        cv::Mat temp=dm.clone();
        cv::Vec3f zero_test;
        zero_test.zeros();
        for (int j = 0; j< dm.rows; j++)
        {
            for (int i = 0; i < dm.cols; i++)
            {
                temp.at<cv::Vec3f>(i,j) = dm.at<cv::Vec3f>(i,j) - nose;
//                std::cout << "calculating : \n " << dm.at<cv::Vec3f>(i,j) << "\n minus \n" << dm.at<cv::Vec3f>(nose_.x,nose_.y) << "\n for result: \n" << temp.at<cv::Vec3f>(i,j) << std::endl;
            }
        }
        temp.copyTo(dm);
        
        //Eigen::Vector3f x_new, y_new, z_new, eye_middle, temp,helper;
        //cv::Point3f eye_middle_cv; 
        //eye_middle_cv = leye3d_+((reye3d_-leye3d_)*0.5);
        //eye_middle<<eye_middle_cv.x,eye_middle_cv.y,eye_middle_cv.z;
        ////std::cout << "eye middle: " << eye_middle_cv << "as point3f: " << eye_middle<<std::endl;
        //x_new<<reye3d_.x-leye3d_.x,reye3d_.y-leye3d_.y,reye3d_.z-leye3d_.z;
        //temp<<nose3d_.x-eye_middle[0],nose3d_.y-eye_middle[1],nose3d_.z-eye_middle[2];
        //z_new=x_new.cross(temp);
        //x_new.normalize();
        //z_new.normalize();
        //y_new=x_new.cross(z_new);
        ////std::cout << "x new: " << x_new <<"\ny new: " << y_new << "\nz_new" <<z_new << std::endl;

        ////find transform
        //Eigen::Affine3f T_norm;
        //helper << dm.at<cv::Vec3f>(nose_.x,nose_.y)[0],dm.at<cv::Vec3f>(nose_.x,nose_.y)[1],dm.at<cv::Vec3f>(nose_.x,nose_.y)[2];
        ////	std::cout << "TESTING TRANSFORM" << std::endl;
        ////	std::cout << "nose: " << nose_pt << "\nreye" << reye_pt << "\nleye" << leye_pt << std::endl;
        //pcl::getTransformationFromTwoUnitVectorsAndOrigin(x_new,y_new,helper,T_norm);
        //Eigen::Vector3f pt_3f;
        //for ( int i = 0; i< dm.rows;i++)
        //{
            //for (int j = 0; j< dm.cols;j++)
            //{
                //pt_3f << dm.at<cv::Vec3f>(i,j)[0],dm.at<cv::Vec3f>(i,j)[1], dm.at<cv::Vec3f>(i,j)[2];
                //pt_3f = T_norm*pt_3f;
                ////dm.at<cv::Vec3f>(i,j) = (cv::Vec3f)(pt_3f[0],pt_3f[1],pt_3f[2]);
                //dm.at<cv::Vec3f>(i,j)[0] = pt_3f[0];
                //dm.at<cv::Vec3f>(i,j)[1] = pt_3f[1];
                //dm.at<cv::Vec3f>(i,j)[2] = pt_3f[2];
                
            //}
        //}
    }
        
    void FeatureTo2DCoords(CvSeq* seq, cv::Point2f& xy)
    {
        //std::cout << seq << std::endl;
        cv::Rect* seq_det=(cv::Rect*)cvGetSeqElem(seq,0);
        xy.x=(float)seq_det->x+seq_det->width/2;
        xy.y=(float)seq_det->y+seq_det->height/2;
    }

    void featureCoordsTo3DPoint(cv::Mat& xyz_mat, cv::Vec3f& nose)
    {
        //std::cout << "featureCoordsTo3dPoint reading nose point at " << nose_ << "\nin xyz_mat: " << xyz_mat.at<cv::Vec3f>(nose_.x,nose_.y) << std::endl;
        nose = xyz_mat.at<cv::Vec3f>(nose_.y,nose_.x);
        nose3d_ = xyz_mat.at<cv::Vec3f>(nose_.y,nose_.x);
        leye3d_ = xyz_mat.at<cv::Vec3f>(leye_.y,leye_.x);
        reye3d_ = xyz_mat.at<cv::Vec3f>(reye_.y,reye_.x);
    }

    void markFeature(cv::Mat& img)
    {
        //std::cout << "MARK FEATURES!" <<std::endl;

        CvSeq* seq;
        double scale=img.cols/160.0;
        IplImage ipl_img=(IplImage)img;

        std::string def_classifier_directory=   "/opt/ros/groovy/share/OpenCV/haarcascades/"  ;
        std::string eye_r_path,eye_path,eye_l_path,nose_path,face_path;

        face_path= def_classifier_directory+ "haarcascade_frontalface_alt2.xml";
        eye_r_path=    def_classifier_directory+ "haarcascade_mcs_righteye.xml";
        eye_l_path=    def_classifier_directory+ "haarcascade_mcs_lefteye.xml";
        nose_path=     def_classifier_directory+ "haarcascade_mcs_nose.xml";
        //std::cout << "cascade path: " << nose_path << std::endl;
        CvHaarClassifierCascade* nose_cascade;     ///< OpenCv haarclassifier cascade for nose
        CvMemStorage* nose_storage;                ///< Pointer to OpenCv memory storage

        CvHaarClassifierCascade* eye_l_cascade;    ///< OpenCv haarclassifier cascade for left eye
        CvMemStorage* eye_l_storage;    ///< Pointer to OpenCv memory storage

        CvHaarClassifierCascade* eye_r_cascade;    ///< OpenCv haarclassifier cascade for right eye
        CvMemStorage* eye_r_storage;    ///< Pointer to OpenCv memory storage

        CvHaarClassifierCascade* m_face_cascade = (CvHaarClassifierCascade*)cvLoad(face_path.c_str(), 0, 0, 0 );	//"ConfigurationFiles/haarcascades/haarcascade_frontalface_alt2.xml", 0, 0, 0 );
        CvMemStorage* m_storage = cvCreateMemStorage(0);
        double faces_increase_search_scale = 1.1;		// The factor by which the search window is scaled between the subsequent scans
        int faces_drop_groups = 2;					// Minimum number (minus 1) of neighbor rectangles that makes up an object.
        int faces_min_search_scale_x = 20;			// Minimum search scale x
        int faces_min_search_scale_y = 20;			// Minimum search scale y
        CvSeq* faces = cvHaarDetectObjects(&ipl_img,	m_face_cascade,	m_storage, faces_increase_search_scale, faces_drop_groups, CV_HAAR_DO_CANNY_PRUNING, cv::Size(faces_min_search_scale_x, faces_min_search_scale_y));
        //if (faces->total != 1) std::cout << "error with face detection in source image"<<std::endl; return false;
        cv::Rect* face = (cv::Rect*)cvGetSeqElem(faces, 0);
        cv::Mat source_img;

        img(cv::Rect(face->x,face->y, face->width,face->height)).copyTo(source_img);

        //load classifiers and find features
        eye_r_cascade=(CvHaarClassifierCascade*) cvLoad(eye_r_path.c_str(),0,0,0);
        eye_r_storage=cvCreateMemStorage(0);

        eye_l_cascade=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
        eye_l_storage=cvCreateMemStorage(0);

        nose_cascade=(CvHaarClassifierCascade*) cvLoad(nose_path.c_str(),0,0,0);
        nose_storage=cvCreateMemStorage(0);

        seq=cvHaarDetectObjects(&ipl_img,nose_cascade,nose_storage,1.1,1,0,cv::Size(15*scale,15*scale));
        if (seq->total == 0) std::cout << "nose det failed. Abort!" <<std::endl;
        cv::Point2f point_feat;
        cv::Mat harrr;
        FeatureTo2DCoords(seq, point_feat);
        nose_ = point_feat;
        cv::Rect* draw = (cv::Rect*)cvGetSeqElem(seq,0);
        img.copyTo(harrr);
        cv::rectangle(harrr, cv::Point(draw->x, draw->y), cv::Point(draw->x + draw->width, draw->y + draw->height), cv::Scalar(0,0,255), 1, 8, 0);
        //std::cout << "nose: " <<nose<<std::endl;

        seq=cvHaarDetectObjects(&ipl_img,eye_l_cascade,eye_l_storage,1.1,1,0,cv::Size(15*scale,15*scale));
        if (seq->total == 0) std::cout << "leye det failed. Abort!"<<std::endl;
        FeatureTo2DCoords(seq, point_feat);
        leye_ = point_feat;
        draw = (cv::Rect*)cvGetSeqElem(seq,0);
        cv::rectangle(harrr, cv::Point(draw->x, draw->y), cv::Point(draw->x + draw->width, draw->y + draw->height), cv::Scalar(0,0,255), 1, 8, 0);
        //std::cout << "left eye: "<< leye<<std::endl;

        seq=cvHaarDetectObjects(&ipl_img,eye_r_cascade,eye_r_storage,1.1,1,0,cv::Size(15*scale,15*scale));
        if (seq->total == 0) std::cout << "reye det failed. Abort!"<<std::endl;
        FeatureTo2DCoords(seq, point_feat);
        reye_ = point_feat;
        draw = (cv::Rect*)cvGetSeqElem(seq,0);
        cv::rectangle(harrr, cv::Point(draw->x, draw->y), cv::Point(draw->x + draw->width, draw->y + draw->height), cv::Scalar(0,0,255), 1, 8, 0);

        cv::circle(harrr, nose_, 3, cv::Scalar(0,255,0));
        cv::circle(harrr, reye_, 3, cv::Scalar(0,255,0));
        cv::circle(harrr, leye_, 3, cv::Scalar(0,255,0));

        cv::line(harrr, nose_, nose_, cv::Scalar(255,0,0));
        //std::cout << "marking nose_: " << nose_ << " in img with color: " << harrr.at<cv::Vec3b>(nose_.x,nose_.y) << std::endl;
        //cv::imshow("Feature Cam", harrr);
        //cv::waitKey();
        
        harrr.copyTo(img);
    }
    
	void process()
	{
		pc1.clear();
        pc2.clear();
        pc3.clear();
		path = "/home/stefan/rgbd_db_heads/";

		std::stringstream jpg_stream,depth_stream,jpg_stream2,depth_stream2, set_path_stream, file_name_stream;

		set_path_stream <<path.c_str()<<file.c_str();
		set_path.data = set_path_stream.str();
		file_name_stream << std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot;
		file_name.data = file_name_stream.str();

		depth_stream<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_d.xml";
		jpg_stream<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_c.bmp";
        cv::Mat img,dm, img2, dm2;
        cv::Vec3f nose1,nose2;
        //dm=cv::Mat(640,480,CV_64FC1);
		//std::cout<<jpg_stream.str()<<"\n"<<depth_stream.str()<<std::endl;
        if (file != "dummy")
        {
            //read bmp image
            img=cv::imread(jpg_stream.str().c_str());
            //img=cv::imread("/home/stefan/rgbd_db/007_2_c.bmp");
            img.convertTo(img,CV_8UC3);

            //read depth xml
            cv::FileStorage fs_read2(depth_stream.str().c_str(),cv::FileStorage::READ);
            fs_read2["depthmap"]>> dm;
            fs_read2.release();
            
            //mark features to find them in rviz
            //find related 3D coordinates, normalize perspective: nose -> origin. eye-eye vector parallel to x axis.
            markFeature(img);
            featureCoordsTo3DPoint(dm, nose1);
            normalizePerspective(dm, nose1);
        }
        depth_stream.clear();
        jpg_stream.clear();
        //^clear cmd not working? -_-
		depth_stream2<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp+1<<"_"<<shot<<"_d.xml";
		jpg_stream2<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp+1<<"_"<<shot<<"_c.bmp";
        //std::cout<<jpg_stream2.str()<<"\n"<<depth_stream2.str()<<std::endl;

        if (file != "dummy")
        {
            //read bmp image
            img2=cv::imread(jpg_stream2.str().c_str());
            //img=cv::imread("/home/stefan/rgbd_db/007_2_c.bmp");
            img2.convertTo(img2,CV_8UC3);
            //read depth xml
            cv::FileStorage fs_read2(depth_stream2.str().c_str(),cv::FileStorage::READ);
            //fs_read2["depth"]>> dm;
            fs_read2["depthmap"]>> dm2;
            fs_read2.release();
            
            //detect and mark features
            //retrieve feature 3d coordinates, normalize perspective: nose -> origin, eye-eye vector parallel to x axis
            markFeature(img2);
            featureCoordsTo3DPoint(dm2,nose2);
            //std::cout << "difference in nose position: " << nose1-nose2 << std::endl;
            //std::cout << "nose 2 position in 2d image: " << nose_ << std::endl;
            //std::cout << "Nose 2 Position DM2 before transform: \n" << nose3d_ <<std::endl;
            cv::Vec3f move = nose2-nose1;
            normalizePerspective(dm2, nose2);
            }
        else
        {
            std::cout << "Dummy img and dm created" << std::endl;
            dm=cv::Mat::zeros(480,640,CV_64FC1);
            img=cv::Mat::zeros(480,640,CV_8UC3);
            file_name.data = "dummy";
        }

		//set parameters, cloud width and height. (must: cloud.width*cloud.height=cloud.points.size)
		pc1.width=dm.cols;
		pc1.height=dm.rows;
        pc2.width=dm2.cols;
		pc2.height=dm2.rows;
        pc3.width=dm.cols*dm.rows+dm2.cols*dm2.rows;
        pc3.height=1;

		pcl::PointXYZRGB pt;
		for(int i =0; i< dm.rows;i++)
		{
			for(int j =0; j< dm.cols; j++)
			{
				//create points from new depthmap pc_trans
				pt.x=dm.at<cv::Vec3f>(j,i)[0];
				pt.y=dm.at<cv::Vec3f>(j,i)[1];
				pt.z=dm.at<cv::Vec3f>(j,i)[2];
                //std::cout << pt.x << "," <<pt.y << "," <<pt.z << std::endl;

				//add color to points, not shifted
				uint32_t rgb = (static_cast<uint32_t>(img.at<cv::Vec3b>(j,i)[0]) << 0 |static_cast<uint32_t>(img.at<cv::Vec3b>(j,i)[1]) << 8 | static_cast<uint32_t>(img.at<cv::Vec3b>(j,i)[2]) << 16);
				pt.rgb = *reinterpret_cast<float*>(&rgb);
                //if (i==nose1.y && j == nose1.x) std::cout << "NOSE1 PT AT ADDED TO CLOUD: " << pt << std::endl;
				pc1.points.push_back(pt);
                pc3.points.push_back(pt);
			}
		}
        //std::cout<< "Pts in cloud after first img added: " <<pc3.points.size() << std::endl;
        
		for(int i =0; i< dm2.rows;i++)
		{
			for(int j =0; j< dm2.cols; j++)
			{
                //if (i==nose2.y && j == nose2.x) std::cout << "NOSE2 PT AT " << nose2 << "\nBEFORE ADDING TO CLOUD: " << dm2.at<cv::Vec3f>(j,i) << std::endl;

				//create points from new depthmap pc_trans
				pt.x=dm2.at<cv::Vec3f>(j,i)[0];
				pt.y=dm2.at<cv::Vec3f>(j,i)[1];
				pt.z=dm2.at<cv::Vec3f>(j,i)[2];
                //std::cout << pt.x << "," <<pt.y << "," <<pt.z << std::endl;

				//add color to points, not shifted
				uint32_t rgb = (static_cast<uint32_t>(img2.at<cv::Vec3b>(j,i)[0]) << 0 |static_cast<uint32_t>(img2.at<cv::Vec3b>(j,i)[1]) << 8 | static_cast<uint32_t>(img2.at<cv::Vec3b>(j,i)[2]) << 16);
				pt.rgb = *reinterpret_cast<float*>(&rgb);
                //if (i==nose2.y && j == nose2.x) std::cout << "NOSE2 PT AT ADDED TO CLOUD: " << pt << std::endl;

				pc2.points.push_back(pt);
                pc3.points.push_back(pt);
			}
		}
        //std::cout<< "Pts in cloud after second img added: " <<pc3.points.size() << std::endl;

		cv_bridge::CvImage cv_ptr;
		cv_ptr.image = img;
		cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
		out_img_1 = *(cv_ptr.toImageMsg());
		out_img_2 = *(cv_ptr.toImageMsg());
		//pcl::toROSMsg(pc1,out_pc2_1);
        //pcl::toROSMsg(pc2,out_pc2_2);
        pcl::toROSMsg(pc3,out_pc2_1);
        std::cout << "-----------------------------------------\n end of processing, new cycle \n " << std::endl;
    }

	void publish()
	{
		out_pc2_1.header.frame_id="/camera/";
		out_pc2_1.header.stamp = ros::Time::now();
		scene_pub_.publish(out_pc2_1);

		out_img_1.header.frame_id="/camera/";
		out_img_1.header.stamp = ros::Time::now();
		img_pub_.publish(out_img_1);
        
        //out_pc2_2.header.frame_id="/camera2/";
		//out_pc2_2.header.stamp = ros::Time::now();
		//scene_pub_2_.publish(out_pc2_2);

		//out_img_2.header.frame_id="/camera2/";
		//out_img_2.header.stamp = ros::Time::now();
		//img_pub_2_.publish(out_img_2);
        
        set_path.header.stamp = ros::Time::now();
		set_pub_.publish(set_path);
		
        file_name.header.stamp =ros::Time::now();
        file_name_pub_.publish(file_name);
        total_published_msgs++;
        
        transform.setOrigin( tf::Vector3(0.0,2.0,0.0));
        transform.setRotation(tf::createQuaternionFromRPY(3.14/2, 0,0));
        transform2=transform;

        tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera/", "/map/"));
        //tf_br_2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/camera2/", "/map/"));
	}
    
    void write()
    {
        
    }
    
	ros::NodeHandle nh;
    tf::TransformBroadcaster tf_br,tf_br_2;
    tf::Transform transform, transform2;
	int persp, shot, total_published_msgs;
    std::string file;

	protected:
		ros::Publisher scene_pub_,scene_pub_2_;
		ros::Publisher img_pub_, img_pub_2_;
        ros::Publisher set_pub_, file_name_pub_;
        
		sensor_msgs::PointCloud2 out_pc2_1, out_pc2_2;
        sensor_msgs::Image out_img_1, out_img_2;

		pcl::PointCloud<pcl::PointXYZRGB> pc1,pc2,pc3;
        
		scene_analyzer::stamped_string set_path, file_name;
        std::string path;
        cv::Point2f leye_,reye_,nose_;
        cv::Point3f leye3d_,reye3d_,nose3d_;
};



int main (int argc, char** argv)
{
	std::string line;
	std::vector<int> perspective_filter;
    
    perspective_filter.push_back(7);
    
	std::vector<std::string> set_filter;
    std::string set_list;

	ros::init (argc, argv, "scene_publisher");

	scene_publisher sp;
	ros::Rate loop_rate(0.6);
	

	while (ros::ok())
	{
		//std::cout<<"SET: " << set_filter[set_it] <<" / PERSPECTIVE:"<<sp.persp<<std::endl;
		sp.process();
		sp.publish();
        //sp.write();
		ros::spinOnce ();
		loop_rate.sleep();
	}
}
