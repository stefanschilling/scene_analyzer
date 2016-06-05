/* function to combine two depth and image matrices (at a time) into 
 * one through iterative closest point procedure.
 * 
 * Method:
 * read images (to include in cob_people_detection, add to training)
 * use OpenCV FLANN to create kd-tree and find nearest neighbours 
 * find transform
 * apply transform
 * repeat until threshold for minimal movement is reached
 * combine points if very close and similar
 * discard outlier points (small tolerance for normal-vector shift 
 * in neighbour points)
 *  
 * Output matrix will be of greater size to allow more detail.
 * Output matrix can not be sorted top left to bottom right as
 * combination may yield points that would have normally been occluded
 * in the kinect view. (maybe sort by depth, followed by x,y?)
 */
 
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

class findNearestNeighbours
{
    public:
    
    //construct
    findNearestNeighbours()
    {
        
    }
    //destruct
    ~findNearestNeighbours()
    {
    }
    
    //find closest neighbour 
    //should probably find neighbours of whole matrix, then match closest points 
    //and work down to worst matching
    void findNeighbours(cv::Mat matTarget, cv::Mat matAdded, cv::Vec3f vecToNeighbour)
    {
    }
    
    //find threshold to cut off outlier points
    //take average of distances?
    float computeDistanceThreshold(cv::Mat mat_distances)
    {
        float f_distance_threshold;
        return f_distance_threshold;
    }
    
    //remove points with matches outside of distance threshold from further consideration
    //they may still be used in the combined image, but not for finding the neighbours and transform
    void removeUnmatchedPoints(cv::Mat matAdded, float f_dist_threshold)
    {
    }
    
    //find transformation from nearest vectors to nearest neighbours
    //input matAdded: matrix to add to target matrix
    //input matVectorsToNeighbour: matrix containing vectors to nearest neighbour
    //output transform: transformation to use on matAdded 
    void findTransform(cv::Mat matAdded, cv::Mat matVectorsToNeighbour)
    {
    }
    
    //apply transformation 
    //input matAdded: matrix to add to target matrix
    //input transform: transformation to nearest neighbours found
    //output matAdded_T
    void applyTransform(cv::Mat matAdded, cv::Mat matAdded_T)
    {
    }
    
    //after distance threshhold converged sufficiently, add new matrix to combined construct
    //input matTarget: Target Matrix
    //input matAdded: Matrix to add to Target
    //output matCombined: combination of the two. Will be initialized as zero-matrix of greater size to allow for adding more points.
    void combineMatrices(cv::Mat matTarget, cv::Mat matAdded, cv::Mat matCombined)
    {
    }
    
    //clean up combined matrix: 
    //points that are very close together will be united
    //points that are very far away may be removed 
    void cleanCombination(cv::Mat matCombined, float f_min_distance, float f_max_distance)
    {
        
    }
    
    protected:
};

int main (int argc, char** argv)
{
    //create class instance
    findNearestNeighbours();
    //load test matrices
    std::string path = "/home/stefan/rgbd_db/";
    std::string file="set_01";
    std::stringstream img_stream1, img_stream2,depth_stream1,depth_stream2;
    int persp, shot;
    persp=7;
    shot=1;
    depth_stream1<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_d.xml";
    img_stream1<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_c.bmp";
    
    persp++;
    depth_stream2<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_d.xml";
    img_stream2<<path<<file.c_str()<<"/"<<std::setw(3)<<std::setfill('0')<<persp<<"_"<<shot<<"_c.bmp";

    //TODO: should be checking if path lead to existing files :)
    //mats to fill with target and added mat
    cv::Mat img1, dm1, img2, dm2;
    //load image matrix
    img1=cv::imread(img_stream1.str().c_str());
    img2=cv::imread(img_stream2.str().c_str());
    //read xyz matrix
    cv::FileStorage fs_read(depth_stream1.str().c_str(),cv::FileStorage::READ);
    fs_read["depth"]>> dm1;
    fs_read.release();
    
    //can we reuse this or do we need to make a new one?
    cv::FileStorage fs_read2(depth_stream2.str().c_str(), cv::FileStorage::READ);
    fs_read2["depth"]>> dm2;
    fs_read2.release();
    std::cout <<"dm1 size: "<< dm1.size()<< "\ndm2 size: "<< dm2.size() << std::endl;
    
    return 0;
}
