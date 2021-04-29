#ifndef TREEITEM_H
#define TREEITEM_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QString>

class TreeItem{
public:
    TreeItem(QString &, QString &,QString &,QString &, pcl::PointCloud<pcl::PointXYZ>::Ptr &);

    QString path_source;
    QString path_folder;
    QString name_source;
    QString extension;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
};

#endif // TREEITEM_H
