#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <queue>
#include <stdio.h>
#include <string>

typedef std::queue<std::string> StringQueue;
typedef pcl::PointCloud<pcl::PointXYZ> PCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCloudPtr;

std::string plys_path;
std::string screw_path;
std::string no_screw_path;
StringQueue ply_paths;
std::string current_ply;
int counter = 0;

StringQueue getFileNamesInDictionary(const char* path);
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
void classifyScrew(std::string path);
void nextPCloud(pcl::visualization::PCLVisualizer* viewer);

int main(int argc, char* argv[])
{
    plys_path = argv[1];
    screw_path = argv[2];
    no_screw_path = argv[3];
    ply_paths = getFileNamesInDictionary(plys_path.c_str());
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PLY Classificator"));
    nextPCloud(viewer.get());
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

    viewer->spin();
}

StringQueue getFileNamesInDictionary(const char* path)
{
    std::queue<std::string> ply_paths;
    DIR* dirp = opendir(path);
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) 
    {
        std::string filename = dp->d_name;
        ply_paths.push(path + filename);
        std::cout << path + filename << std::endl;
    }
    closedir(dirp);
    ply_paths.pop();
    ply_paths.pop();
    return ply_paths;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
  
  if(event.getKeySym() == "y" && event.keyDown())
  {
      std::cout << "y was pressed => screw" << std::endl;
      classifyScrew(screw_path);
      nextPCloud(viewer);
  }
  
  if(event.getKeySym() == "n" && event.keyDown())
  {
      std::cout << "n was pressed => no screw" << std::endl;
      classifyScrew(no_screw_path);
      nextPCloud(viewer);
  }
  return;
}

void nextPCloud(pcl::visualization::PCLVisualizer* viewer)
{
    PCloudPtr pCloud(new PCloud());
    if(ply_paths.size() > 0 && pcl::io::loadPLYFile<pcl::PointXYZ>(ply_paths.front(), *pCloud) != -1)
    {
        current_ply = ply_paths.front();
        ply_paths.pop();
        viewer->removePointCloud();
        viewer->addPointCloud(pCloud);
        std::cout << "Screw (y) or no screw (n)?" << std::endl;
        return;
    }

    std::cout << "Done." << std::endl;
    exit(0);
}

void classifyScrew(std::string path)
{
    PCloudPtr pCloud(new PCloud());
    if(pcl::io::loadPLYFile<pcl::PointXYZ>(current_ply, *pCloud) != -1)
    {
        pcl::io::savePLYFile<pcl::PointXYZ>(path + std::to_string(counter++) + ".ply", *pCloud);
    }
    return;
}
