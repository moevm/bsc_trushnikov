#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <QThread>

#include "QVTKWidget.h"
#include <utility>
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include "treeitem.h"
#include "worker.h"
#include <QTime>
#include "rmse.h"
#include "densityform.h"
#include "percentform.h"
#include <utility>



QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


typedef QVector<pcl::PointXYZ> MyArray;

class MainWindow : public QMainWindow{
    Q_OBJECT

public:
    MainWindow(QString, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openFile();
    void saveFile();
    void simplifyFirstMethod(int);
    void simplifySecondMethod();
    void showRMSEWindow();

    void simplificationFinished();
    void closeEvent (QCloseEvent *);
    void calculateRMSE(int,int);
    void RMSEFinished();
    void DensityFinished();
    void calculateDensity(double);


    void on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column);

    void on_treeWidget_itemChanged(QTreeWidgetItem *item, int column);

    void on_treeWidget_itemClicked(QTreeWidgetItem *item, int column);

protected:
    void keyPressEvent( QKeyEvent *event ) override;

private:
    Ui::MainWindow *ui;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PCLVisualizer::Ptr viewer1;
    pcl::visualization::PCLVisualizer::Ptr viewer2;
    std::vector <TreeItem> clouds_;
    QTime time;
    QThread *thread1;
    Worker *worker1;
    QThread *thread2;
    Worker *worker2;
    QThread *thread3;
    Worker *worker3;
    RMSE *rmse;
    DensityForm * densform;
    PercentForm * percentform;
    QString current_dir;
    QString last_dir;
    bool first_open = true;
    bool turnOnswitcher =true;


    void setConnections();
    void setTreeWidget();
    void setViewer();
    void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &, QString&, bool);
    void hidePointCloud(QString&);
    void setCameraPos(pcl::PointCloud<pcl::PointXYZ>::Ptr &);
    void showPointCloud(size_t &);
    void addtreeWidgeSourcetItem(QString &, QString &, QString &);
    void addtreeWidgeTargetItem(size_t &, QTreeWidgetItem *);
    void simplifyPC(int);

};
#endif // MAINWINDOW_H
