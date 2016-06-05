#include "boost/filesystem.hpp"

int main(int argc, char **argv)
{
    boost::filesystem::create_directory("/home/stefan/create_this_dir");
    std::string dir1, dir2;
    dir1 = "/home/stefan/create_dir_with_strings";
    dir2 = "/now/";
    std::cout << "create single new folder test" << std::endl;
    boost::filesystem::create_directory(dir1);
    std::cout << "done, create subfolder..." << std::endl;

    dir1 = dir1+dir2;
    if (boost::filesystem::create_directory(dir1)) std::cout << "created succesfully: " << dir1 << std::endl;
    return 0;
}
