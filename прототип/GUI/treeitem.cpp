#include "treeitem.h"

TreeItem::TreeItem(QString & path,QString & path_folder,QString &name_source, QString &extension, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud):
    path_source(path), path_folder(path_folder) ,name_source(name_source), extension(extension), source_cloud(cloud){}
