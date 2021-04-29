#ifndef HELPFUNCTIONS_H
#define HELPFUNCTIONS_H

#ifdef _DEBUG
  #undef _DEBUG
  #include <python.h>
  #define _DEBUG
#else
  #include <python.h>
#endif
#undef B0

#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <QVector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>



QVector<pcl::PointXYZ> simplify_method_1(std::string &&, std::string &&);
std::vector<double> getCameraPos(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
double squaredEuclideanDistance(pcl::PointXYZ &, pcl::PointXYZ &);
double square(double);




#endif // HELPFUNCTIONS_H
