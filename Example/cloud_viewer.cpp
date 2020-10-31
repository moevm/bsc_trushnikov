#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
    
int user_data;
    
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer){
    viewer.setBackgroundColor (0.8, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    std::cout << "i only run once" << std::endl;
    
}
       
int main (int argc, char* argv[]){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile (argv[1], *cloud);
    
    pcl::visualization::CloudViewer viewer("Cloud Viewer");   
    viewer.showCloud(cloud);
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    while (!viewer.wasStopped ()){
    user_data++;
    }
    return 0;
}
