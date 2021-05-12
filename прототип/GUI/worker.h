#ifndef WORKER_H
#define WORKER_H
#include <QTreeWidgetItem>
#include <QDebug>
#include <QTextEdit>
#include <QObject>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

typedef QVector<pcl::PointXYZ> MyArray;
class Worker : public QObject {
    Q_OBJECT

public:
    Worker();
    ~Worker();

    QString path_dir;
    QString path_file;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_p;
    QString file_name;
    QString file_extention;

    double time;
    double r;
    int type_method;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target;
    std::string type;
    int index_source;
    int index_target;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, double> result;


public slots:
    //void process();
    void simplificate_method_1();
    void calculateRMSE();
    void calculateDensity();

signals:
    void finished();
    void simplificationFinished();
    void RMSEFinished();
    void DensityFinished();

private:
    // add your variables here
};


#endif // WORKER_H
