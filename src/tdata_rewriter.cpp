//#include <iostream>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem.hpp"

void rewrite_tdata(std::string& training_path,std::string tdata, std::string& add_label)
{
    // read out existing tdata and label-to-add from other tdata, write new tdata xml with combined content.

	cv::FileStorage fileStorageRead(training_path + "tdata2_combo.xml", cv::FileStorage::READ);
	if (!fileStorageRead.isOpened())
	{
		std::cout << "Error: load Training Data: Can't open " << training_path+"tdata2_combo.xml" << ".\n" << std::endl;
	}

	// labels
	std::vector<std::string> face_labels, face_images, face_depths;
	face_labels.clear();
	face_images.clear();
	face_depths.clear();
	int old_number_entries = (int)fileStorageRead["number_entries"];
	// read current entries
	for(int i=0; i<old_number_entries; i++)
	{
		// labels
		std::ostringstream tag_label1, tag_label2, tag_label3;
		tag_label1 << "label_" << i;
		std::string label = (std::string)fileStorageRead[tag_label1.str().c_str()];
		face_labels.push_back(label);
		tag_label2 << "image_" << i;
		std::string image = (std::string)fileStorageRead[tag_label2.str().c_str()];
		face_images.push_back(image);
		tag_label3 << "depthmap_" << i;
		std::string depth = (std::string)fileStorageRead[tag_label3.str().c_str()];
		face_depths.push_back(depth);
	}
    fileStorageRead.release();
    
    // read entries from other tdata to add
    cv::FileStorage fileStorageRead2(training_path + tdata, cv::FileStorage::READ);
	if (!fileStorageRead2.isOpened())
	{
		std::cout << "Error: load Training Data: Can't open " << training_path+tdata << ".\n" <<std::endl;
	}

    old_number_entries = (int)fileStorageRead2["number_entries"];
	// read current entries
	for(int i=0; i<old_number_entries; i++)
	{
		// labels
		std::ostringstream tag_label1, tag_label2, tag_label3;
		tag_label1 << "label_" << i;        
        std::string label = (std::string)fileStorageRead2[tag_label1.str().c_str()];
        if (label == add_label)
        {
            face_labels.push_back(label);
            tag_label2 << "image_" << i;
            std::string image = (std::string)fileStorageRead2[tag_label2.str().c_str()];
            face_images.push_back(image);
            tag_label3 << "depthmap_" << i;
            std::string depth = (std::string)fileStorageRead2[tag_label3.str().c_str()];
            face_depths.push_back(depth);
        }
	}
    fileStorageRead2.release();

	//create tdata2.xml with newly added images.
	cv::FileStorage fileStorageWrite(training_path+"tdata2_combo.xml", cv::FileStorage::WRITE);
	if (fileStorageWrite.isOpened())
	{
		fileStorageWrite << "number_entries" << (int)face_labels.size();
		for(int i=0; i<face_labels.size(); i++)
		{
			std::ostringstream tag, tag2, tag3;
			std::ostringstream shortname_img, shortname_depth;
            // face label
            tag << "label_" << i;
            fileStorageWrite << tag.str().c_str() << face_labels[i].c_str();

            // face images
            shortname_img << "img/" << i << ".bmp";
            tag2 << "image_" << i;
            fileStorageWrite << tag2.str().c_str() << face_images[i].c_str();

            // depth images
            shortname_depth << "depth/" << i << ".xml";
            tag3 << "depthmap_" << i;
            fileStorageWrite << tag3.str().c_str() << face_depths[i].c_str();
        
		}
		fileStorageWrite.release();
		face_labels.clear();
		face_images.clear();
		face_depths.clear();
	}
}

bool list_existing_tdata(std::string& training_path, std::vector<std::string>& tdata_ex)
{
    boost::filesystem::path tdata_path = training_path;
    if (exists(tdata_path))
    {
        boost::filesystem::directory_iterator end_itr;
        for(boost::filesystem::directory_iterator itr(training_path); itr!=end_itr; ++itr)
        {
            if (boost::filesystem::is_regular_file(itr->status()) && itr->path().extension() == ".xml")
            {
                tdata_ex.push_back(itr->path().c_str());
            }
        }
    }
}

bool list_tdata_labels(std::string& tdata, std::vector<std::string>& labels)
{
    //std::cout << "received tdata: " << tdata << std::endl;
    
    cv::FileStorage fileStorage(tdata, cv::FileStorage::READ);
    if(!fileStorage.isOpened()) 
    {
        std::cout << "tdata cannot be opened: " << tdata <<std::endl;
        return false;
    }
    labels.clear();
    int number_entries = (int)fileStorage["number_entries"];
    
    //std::cout << "entries in tdata: " << number_entries << std::endl;
    
    for(int i=0; i<number_entries; i++)
    {
        // labels
        std::ostringstream tag_label;
        bool exists=false;
        tag_label << "label_" << i;
        std::string label = (std::string)fileStorage[tag_label.str().c_str()];
        //std::cout << "step: " << i << ", label: " << label << std::endl;
        for (int j=0; j<labels.size(); j++)
        {   
            if (labels[j] == label) exists = true;
        }
        if (!exists) labels.push_back(label);        
    }
    //std::cout << "checkup: labels size?" << labels.size() << std::endl;
}

int main (int argc, char** argv)
{
    std::cout << "Trying to mix tdata from several different creation methods" << std::endl;
    std::string training_path = "/home/stefan/.ros/cob_people_detection/files/training_data/";
    std::vector<std::string> tdata_vec;
    std::vector<std::string> labels_vec1, labels_vec2;
    std::string input;
    
    //get available tdata
    std::vector<std::string> tdata_ex;
    list_existing_tdata(training_path, tdata_ex);
    for (int i =0; i < tdata_ex.size(); i++)
    {
        std::cout << i << ": " << tdata_ex[i].substr(tdata_ex[i].find_last_of("/\\")) << "\n";
    }
    std::cout<<std::endl;
    
    //select tdata
    //TODO: unrestricted tdata source amount 
    while (tdata_vec.size() < 2)
    {   
        std::cout << "tdata to use: ";
        std::cin >> input;
        std::istringstream set_file(input);
        int i;
        set_file>>i;   
        tdata_vec.push_back(tdata_ex[i]);
    }
    
    //select labels (set_xx) per tdata
    //TODO: unrestricted tdata source amount: use vector<vector<string>> instead of labels_vec1,2... -> labels_vec[i][j]
    std::cout << "content of tdata_vec: " << tdata_vec[0] << ", " << tdata_vec[1] << std::endl;
    std::vector<std::string> labels;
        
    //TODO - DONE: give label list as with tdata above 
    std::cout << "labels available in first tdata" << std::endl;
    list_tdata_labels(tdata_vec[0],labels);
    for (int i=0; i<labels.size();i++)
    {
        std::cout << i << ": " << labels[i] << "\n";
    }
    std::cout << std::endl;
    
    //select labels from 1st tdata
    while (input != "q")
    {
        std::cout << "label from " << tdata_vec[0] << " (enter q to stop): ";
        std::cin >> input;
        if (input != "q") labels_vec1.push_back(input);
    }
    //reset input
    input = " ";
    std::cout << "selected from first tdata: " << std::endl;
    for(int i=0; i< labels_vec1.size(); i++)
    {
        std::cout << labels_vec1[i] << ", ";
    }
    
    labels.clear();
    std::cout << std::endl;
    std::cout << "labels available in second tdata" << std::endl;
    list_tdata_labels(tdata_vec[1],labels);
    for (int i=0; i<labels.size();i++)
    {
        std::cout << i << ": " << labels[i] << "\n";
    }
    std::cout << std::endl;
    
    //select labels from 2nd tdata
    while (input != "q")
    {
        std::cout << "label from " << tdata_vec[1] << " (enter q to stop): ";
        std::cin >> input;
        if (input != "q") labels_vec2.push_back(input);
    }
    
    std::cout << "selected from second tdata: " << std::endl;
    for(int i=0; i< labels_vec2.size(); i++)
    {
        std::cout << labels_vec2[i] << ", ";
    }
    std::cout << std::endl;
    
    // Fixed settings until input implementation is complete
    std::string tdata1 = "tdata_center.xml";
    std::string tdata2 = "tdata_24_004.xml";
    std::vector<std::string> labels1,labels2;
    labels1.push_back("set_22");
    labels1.push_back("set_23");
    labels1.push_back("set_24");
    labels1.push_back("set_25");
    labels1.push_back("set_26");
    labels1.push_back("set_27");
    labels1.push_back("set_28");
    labels1.push_back("set_29"); 
    labels1.push_back("set_30");
    labels2.push_back("set_31");
    for (int i =0; i< labels1.size(); i++)
    {
        //rewrite_tdata(training_path,tdata1,labels1[i]);
        //std::cout << "added " << labels1[i] << "./n" <<std::endl;
    }
    for (int i =0; i< labels2.size(); i++)
    {
        //rewrite_tdata(training_path,tdata2,labels2[i]);
        //std::cout << "added " << labels2[i] << "./n" <<std::endl;
    }
}
