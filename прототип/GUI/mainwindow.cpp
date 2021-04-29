#include "helpfunctions.h"
#undef B0
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <vtkRenderWindow.h>
#include <QFileDialog>
#include <pcl/visualization/cloud_viewer.h>
#include "QVTKWidget.h"
#include <QCloseEvent>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


void MainWindow::closeEvent (QCloseEvent *event){
    if( thread1->isRunning())
        thread1->terminate();
    if( thread2->isRunning())
        thread2->terminate();
    if( thread3->isRunning())
        thread3->terminate();
   // Py_Finalize();
}


void MainWindow::keyPressEvent( QKeyEvent *event ){
    if(event->key() == 16777223){
        auto selected_list = ui->treeWidget->selectedItems();
        if (selected_list.size()){
            auto item = selected_list.at(0);
            int index = ui->treeWidget->indexOfTopLevelItem(item);
            if (index == -1){
                item = item->parent();
                index = ui->treeWidget->indexOfTopLevelItem(item);
            }
            QString name;
            if(this->clouds_.at(index).path_source == " ")
                name = this->clouds_.at(index).name_source + "." + this->clouds_.at(index).extension;
            else
                name = this->clouds_.at(index).path_source;
            hidePointCloud(name);
            delete item;
            this->clouds_.erase(this->clouds_.begin() + index);
            ui->qvtkWidget->update ();
        }
    }
}


MainWindow::MainWindow(QString curr_dir, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow){

    current_dir = curr_dir.left(curr_dir.lastIndexOf(QChar('\\')));
    int j = 0;
    while ((j = current_dir.indexOf("\\", j)) != -1) {
        current_dir.replace(j, 1, "/");
        ++j;
    }

    ui->setupUi(this);
    ui->textEdit->setReadOnly(true);
    ui->textEdit->setFontPointSize(9);
    ui->textEdit_2->setReadOnly(true);
    ui->textEdit_2->setFontPointSize(9);
    qRegisterMetaType<MyArray>("MyArray");
    setTreeWidget();
    setViewer();
    thread1 = new QThread;
    worker1 = new Worker();
    thread2 = new QThread;
    worker2 = new Worker();
    thread3 = new QThread;
    worker3 = new Worker();
    rmse = new RMSE();
    densform = new DensityForm();
    percentform = new PercentForm();
    setConnections();

}

void MainWindow::setViewer(){
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer"));
    viewer->setCameraPosition(1.0,0,0,0,0,0,0,0,0);
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    ui->qvtkWidget->update ();
}


void MainWindow::setTreeWidget(){
    ui->treeWidget->setColumnCount(1);
    ui->treeWidget->setHeaderLabel("  ");
}

void MainWindow::simplifyPC(int percent){
    auto selected_list = ui->treeWidget->selectedItems();
    if (selected_list.size()){
        auto item = selected_list.at(0);
        int index = ui->treeWidget->indexOfTopLevelItem(item);
        if (index == -1){
            auto parent = item->parent();
            if (ui->treeWidget->indexOfTopLevelItem(parent) != -1){
                TreeItem & item_ = this->clouds_.at(ui->treeWidget->indexOfTopLevelItem(parent));
                worker1->path_dir = current_dir + "/python";
                worker1->path_file = item_.path_source;
                worker1->source_p = item_.source_cloud;
                worker1->file_extention = item_.extension;
                worker1->file_name = item_.name_source;
                worker1->percent = percent;
                ui->textEdit->append(time.currentTime().toString() + ": Simplification... File (" + item_.path_source + item_.name_source + item_.extension + ")");
                ui->menuSimplify->setEnabled(false);
                thread1->start();
            }
        }
    }

}

void MainWindow::simplifySecondMethod(){
    simplifyPC(0);
}

void MainWindow::simplifyFirstMethod(int percent) {
    auto selected_list = ui->treeWidget->selectedItems();
    if (selected_list.size()){
        auto item = selected_list.at(0);
        int index = ui->treeWidget->indexOfTopLevelItem(item);
        if (index == -1){
            auto parent = item->parent();
            if (ui->treeWidget->indexOfTopLevelItem(parent) != -1){
                TreeItem & item_ = this->clouds_.at(ui->treeWidget->indexOfTopLevelItem(parent));
                worker1->path_dir = current_dir + "/python";
                worker1->path_file = item_.path_source;
                worker1->source_p = item_.source_cloud;
                worker1->file_extention = item_.extension;
                worker1->file_name = item_.name_source;
                worker1->percent = percent;
                ui->textEdit->append(time.currentTime().toString() + ": Simplification... File (" + item_.path_source + item_.name_source + item_.extension + ")");
                ui->menuSimplify->setEnabled(false);
                thread1->start();
            }
        }
    }
}

void MainWindow::simplificationFinished(/*QString path_file,MyArray points*/){

    ui->textEdit->append(time.currentTime().toString() + ": Simplification completed! Time = " + QString::number(worker1->time) +" sec. File (" + worker1->path_file + ")");

    int id = 1;
    std::string file_name_std = worker1->file_name.toStdString();

    std::for_each(clouds_.begin(),clouds_.end(),[&file_name_std, &id](const TreeItem& element){
        if(element.path_source == " " && element.name_source.size() > (file_name_std.size() + 1)){
            if(file_name_std == element.name_source.toStdString().substr(0, file_name_std.size()) && element.name_source.at(file_name_std.size()) == '_'){
                int id_ = std::stoi(element.name_source.toStdString().substr(file_name_std.size() + 1, element.name_source.size())) + 1;
                id = std::max(id,id_);
            }}});
     QString file_name = worker1->file_name + "_" + QString::fromUtf8(std::to_string(id).c_str());
     QString path = " ";
     QString path_folder = " ";
     clouds_.push_back(TreeItem(path,path_folder,file_name, worker1->file_extention, worker1->target));
     addtreeWidgeSourcetItem(path_folder, file_name, worker1->file_extention);
     file_name = file_name + "." + worker1->file_extention;
     showPointCloud(worker1->target, file_name, true);
     ui->menuSimplify->setEnabled(true);
}


void MainWindow::showRMSEWindow(){
    QStringList list;
    std::for_each(clouds_.begin(),clouds_.end(),[&list](const TreeItem& element){
        QString name = element.name_source + "." + element.extension + " (" + element.path_folder + ")";
        list.push_back(name);
            });
    rmse->setQComboBoxes(list);
    rmse->show();
}

void MainWindow::calculateDensity(double r){
    auto selected_list = ui->treeWidget->selectedItems();
    if (selected_list.size()){
        auto item = selected_list.at(0);
        int index = ui->treeWidget->indexOfTopLevelItem(item);
        if (index == -1){
            auto parent = item->parent();
            if (ui->treeWidget->indexOfTopLevelItem(parent) != -1){
                worker3->source = this->clouds_.at(ui->treeWidget->indexOfTopLevelItem(parent)).source_cloud;
                worker3->r = r;
                thread3->start();
            }
        }
    }
}

void MainWindow::DensityFinished(){
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(worker3->result.first, "intensity");
    viewer2.reset (new pcl::visualization::PCLVisualizer ("viewer"));
    viewer2->addPointCloud< pcl::PointXYZI >(worker3->result.first, point_cloud_color_handler, "cloud");
    viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
    viewer2->setBackgroundColor(255, 255, 255);

    while (!viewer2->wasStopped() ){
        viewer2->spinOnce(50);
    }

}


void MainWindow::setConnections(){
    worker1->moveToThread(thread1);
    worker2->moveToThread(thread2);
    worker3->moveToThread(thread3);
    connect(thread3, SIGNAL(started()), worker3, SLOT(calculateDensity()));
    connect(thread2, SIGNAL(started()), worker2, SLOT(calculateRMSE()));
    connect(thread1, SIGNAL(started()), worker1, SLOT(simplificate_method_1()));
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::openFile);
    connect(ui->actionSave, &QAction::triggered, this, &MainWindow::saveFile);

    connect(ui->actionSecond_method, &QAction::triggered, this, &MainWindow::simplifySecondMethod);
    connect(ui->actionfirst_method, &QAction::triggered, percentform, &PercentForm::showPercentForm);
    connect(percentform, SIGNAL(simplify(int)),this,SLOT(simplifyFirstMethod(int)));

    connect(ui->actionRMSE, &QAction::triggered, this, &MainWindow::showRMSEWindow);
    connect(ui->actionDensity,&QAction::triggered,densform,&DensityForm::showDensityForm);


    connect(worker1,SIGNAL(simplificationFinished()), this, SLOT(simplificationFinished()));

    connect(worker1, SIGNAL(finished()), thread1, SLOT(quit()));
    connect(worker2, SIGNAL(finished()), thread2, SLOT(quit()));
    connect(worker3, SIGNAL(finished()), thread3, SLOT(quit()));
    connect(worker2,SIGNAL(RMSEFinished()),this, SLOT(RMSEFinished()));
    connect(worker3,SIGNAL(DensityFinished()),this, SLOT(DensityFinished()));

    connect(rmse,SIGNAL(calculateRMSE(int,int)),this,SLOT(calculateRMSE(int,int)));
   connect(densform,SIGNAL(calculateDensity(double)),this,SLOT(calculateDensity(double)));
}

void MainWindow::calculateRMSE(int source,int target){
    worker2->source = this->clouds_.at(source).source_cloud;
    worker2->target = this->clouds_.at(target).source_cloud;
    worker2->type = "nn";
    worker2->index_source = source;
    worker2->index_target = target;
    ui->textEdit->append(time.currentTime().toString() + ": starting to calculate RMSE");
    thread2->start();
}

void MainWindow::RMSEFinished(){
    ui->textEdit->append(time.currentTime().toString() + ": RMSE calculation completed. RMSE = " + QString::number(worker2->result.second));
    if(rmse->isShowPC()){
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(worker2->result.first, "intensity");
        viewer1.reset (new pcl::visualization::PCLVisualizer ("viewer"));

        viewer1->addPointCloud< pcl::PointXYZI >(worker2->result.first, point_cloud_color_handler, "cloud");
        viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        while (!viewer1->wasStopped() ){
                viewer1->spinOnce(50);
            }
    }
}
void MainWindow::showPointCloud(size_t & index){
  //  auto & pair = clouds.at(index);
  //  showPointCloud(pair.first, pair.second);

}


void MainWindow::setCameraPos(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    auto camera_pos = getCameraPos(cloud);
    viewer->setCameraPosition(camera_pos[0], camera_pos[1], camera_pos[2],0,0,0,0,0,1);
}


void MainWindow::hidePointCloud(QString &name){
    viewer->removePointCloud(name.toStdString());
    ui->qvtkWidget->update ();
}


void MainWindow::showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, QString &name, bool moveTo){
    if(moveTo)
        setCameraPos(cloud);
    viewer->addPointCloud (cloud, name.toStdString());
    ui->qvtkWidget->update ();
}


void MainWindow::addtreeWidgeSourcetItem(QString &path_folder, QString &name_source, QString & extension){
    QTreeWidgetItem *father = new QTreeWidgetItem();
    father->setText(0, name_source + "." + extension + " (" + path_folder + ")");
    father->setIcon(0,QIcon(current_dir + "/icons/folder.jpg"));
    father->setCheckState(0, Qt::Checked);
    ui->treeWidget->addTopLevelItem(father);

    QTreeWidgetItem *child = new QTreeWidgetItem();
    child->setText(0, name_source);
    child->setIcon(0,QIcon(current_dir + "/icons/source_cloud.jpg"));
    father->addChild(child);

    ui->treeWidget->resizeColumnToContents(0);
}

void MainWindow::saveFile(){
    auto selected_list = ui->treeWidget->selectedItems();
    if (selected_list.size()){
        auto item = selected_list.at(0);
        int index = ui->treeWidget->indexOfTopLevelItem(item);
        if (index == -1){
            item = item->parent();
            index = ui->treeWidget->indexOfTopLevelItem(item);
        }
        QString filePath = QFileDialog::getSaveFileName(this,
                tr("Open Point Cloud"), last_dir + "/" + clouds_.at(index).name_source + "." + clouds_.at(index).extension,
                tr("Pcd (*.pcd)"));
        if(filePath.size() == 0)
            return;
        last_dir = QFileInfo(filePath).path();

        QString name;
        if(this->clouds_.at(index).path_source == " ")
            name = this->clouds_.at(index).name_source + "." + this->clouds_.at(index).extension;
        else
            name = this->clouds_.at(index).path_source;
        hidePointCloud(name);
        qDebug() << "Hide " << name;

        auto it = std::find_if( clouds_.begin(), clouds_.end(),
         [&filePath](const TreeItem& element){
            return element.path_source == filePath;} );
        if(it != clouds_.end()){
            if(filePath == this->clouds_.at(index).path_source){
                qDebug() << "Owerriting themself";
                return;
            }
            hidePointCloud(it->path_source);
            it->source_cloud = this->clouds_.at(index).source_cloud;
            this->clouds_.erase(this->clouds_.begin() + index);
            delete item;
            showPointCloud(it->source_cloud, it->path_source, false);
            pcl::io::savePCDFileASCII (it->path_source.toStdString(), *it->source_cloud);
        }
        else{
            std::cout << "Saving ..." << std::endl;

            this->clouds_.at(index).path_source = filePath;
            this->clouds_.at(index).path_folder = last_dir;
            std::string str = filePath.toStdString().substr(filePath.toStdString().find_last_of("/") + 1);

            this->clouds_.at(index).name_source = QString::fromUtf8(str.substr(0, str.find_last_of(".")).c_str());
            this->clouds_.at(index).extension = QString::fromUtf8(str.substr(str.find_last_of(".") + 1).c_str());


            pcl::io::savePCDFileASCII (filePath.toStdString(), *this->clouds_.at(index).source_cloud);
            turnOnswitcher = false;
            ui->treeWidget->topLevelItem(index)->setText(0, clouds_.at(index).name_source + "." + clouds_.at(index).extension + " (" + clouds_.at(index).path_folder + ")");
            ui->treeWidget->topLevelItem(index)->child(0)->setText(0,clouds_.at(index).name_source);
            ui->treeWidget->update();
            turnOnswitcher = true;
            showPointCloud(clouds_.at(index).source_cloud, clouds_.at(index).path_source, false);
        }

        /*
        QString name;
        if(this->clouds_.at(index).path_source == " ")
            name = this->clouds_.at(index).name_source + "." + this->clouds_.at(index).extension;
        else
            name = this->clouds_.at(index).path_source;
        hidePointCloud(name);
        delete item;
        this->clouds_.erase(this->clouds_.begin() + index);
        ui->qvtkWidget->update ();
        */
    }
}

void MainWindow::openFile(){
    bool cloud_exist = false;
    QString filePath = QFileDialog::getOpenFileName(this,
            tr("Open Point Cloud"), "",
            tr("Pcd (*.pcd)"));
    if(filePath.length() == 0)
        return;
    std::string filePath_std = filePath.toStdString();


    if (clouds_.size()){

        auto it = std::find_if( clouds_.begin(), clouds_.end(),
         [&filePath](const TreeItem& element){
            return element.path_source == filePath;} );
        if(it != clouds_.end()){
            cloud_exist = true;
        }
    }
    if (!cloud_exist){
        int subs_index = filePath_std.find_last_of("/\\");
        QString path_folder = QString::fromUtf8(filePath_std.substr(0, subs_index).c_str());
        filePath_std = filePath_std.substr(subs_index + 1);
        subs_index = filePath_std.find_last_of(".");
        QString name_source = QString::fromUtf8(filePath_std.substr(0, subs_index).c_str());
        QString extension = QString::fromUtf8(filePath_std.substr(subs_index + 1).c_str());
        ui->textEdit->append(time.currentTime().toString() + ": Opened new file " + filePath);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(filePath.toStdString(), *cloud_source);
        if(first_open)
            last_dir = path_folder;
        clouds_.push_back(TreeItem(filePath,path_folder,name_source,extension, cloud_source));
        addtreeWidgeSourcetItem(path_folder, name_source, extension);
        showPointCloud(cloud_source, filePath, true);

    }

}


MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column){
    int index = ui->treeWidget->indexOfTopLevelItem(item);
    if (index == -1){
        auto parent = item->parent();
        if (ui->treeWidget->indexOfTopLevelItem(parent) != -1 && parent->checkState(column) == Qt::Checked){
            TreeItem & item_ = this->clouds_.at(ui->treeWidget->indexOfTopLevelItem(parent));
            setCameraPos(item_.source_cloud);
        }
    }
}


void MainWindow::on_treeWidget_itemChanged(QTreeWidgetItem *item, int column){
    if(turnOnswitcher){
        qDebug() << "Changed";
        int index =  ui->treeWidget->indexOfTopLevelItem(item);
        QString name;
        if(this->clouds_.at(index).path_source == " ")
            name = this->clouds_.at(index).name_source + "." + this->clouds_.at(index).extension;
        else
            name = this->clouds_.at(index).path_source;
        if(item->checkState(column) == Qt::Checked){
            showPointCloud(this->clouds_.at(index).source_cloud, name, false);
        }
        else{
            hidePointCloud(name);
        }
    }
}

void MainWindow::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column){
    int index = ui->treeWidget->indexOfTopLevelItem(item);
    if (index == -1){
        ui->textEdit_2->clear();
        ui->textEdit_2->append("Name: " + this->clouds_.at(ui->treeWidget->indexOfTopLevelItem(item->parent())).name_source);
        ui->textEdit_2->append("Points: " + QString::number(this->clouds_.at(ui->treeWidget->indexOfTopLevelItem(item->parent())).source_cloud->size()));
    }
}
