#include "helpfunctions.h"
#undef B0
#include "worker.h"
#include <QDebug>
#include <QTime>
#include <boost/math/constants/constants.hpp>
#include <fstream>

Worker::Worker() {

}


Worker::~Worker() {

}


void Worker::calculateRMSE(){
    double rmse = 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_xyzi (new pcl::PointCloud<pcl::PointXYZI> ());
    output_xyzi->points.resize (source->size ());
    output_xyzi->height = source->height;
    output_xyzi->width = source->width;


    if (type == "nn"){
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
        tree->setInputCloud (target);
        for (std::size_t point_i = 0; point_i < source->size (); ++point_i){
          pcl::Indices nn_indices (1);
          std::vector<float> nn_distances (1);
          if (!tree->nearestKSearch ((*source)[point_i], 1, nn_indices, nn_distances))
            continue;
          std::size_t point_nn_i = nn_indices.front();

          double dist = squaredEuclideanDistance ((*source)[point_i], (*target)[point_nn_i]);
          rmse += dist;

          (*output_xyzi)[point_i].x = (*source)[point_i].x;
          (*output_xyzi)[point_i].y = (*source)[point_i].y;
          (*output_xyzi)[point_i].z = (*source)[point_i].z;
          (*output_xyzi)[point_i].intensity = dist;
        }
        rmse = std::sqrt (rmse / static_cast<double> (source->size ()));
      }

    this->result = std::make_pair(output_xyzi,rmse);
    emit RMSEFinished();
    emit finished();
}


void Worker::calculateDensity(){
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_xyzi (new pcl::PointCloud<pcl::PointXYZI> ());
    output_xyzi->points.resize (source->size ());
    output_xyzi->height = source->height;
    output_xyzi->width = source->width;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
    tree->setInputCloud (source);

    std::cout << "Calculate density  r = " << r << std::endl;

    for (std::size_t point_i = 0; point_i < source->size (); ++point_i){
        pcl::Indices nn_indices;
        std::vector<float> nn_distances;
        if (!tree->radiusSearch((*source)[point_i],r,nn_indices,nn_distances)){
            continue;
        }
        (*output_xyzi)[point_i].x = (*source)[point_i].x;
        (*output_xyzi)[point_i].y = (*source)[point_i].y;
        (*output_xyzi)[point_i].z = (*source)[point_i].z;
        (*output_xyzi)[point_i].intensity = (3*(nn_indices.size()-1))/(4*boost::math::constants::pi<double>()*pow(r,3));

    }

    /*
    std::cout << "Calculate density" << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_xyzi (new pcl::PointCloud<pcl::PointXYZI> ());
    output_xyzi->points.resize (source->size ());
    output_xyzi->height = source->height;
    output_xyzi->width = source->width;


    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
    tree->setInputCloud (source);
    for (std::size_t point_i = 0; point_i < source->size (); ++point_i){
      pcl::Indices nn_indices (2);
      std::vector<float> nn_distances (2);
      if (!tree->nearestKSearch ((*source)[point_i], 2, nn_indices, nn_distances))
        continue;

      (*output_xyzi)[point_i].x = (*source)[point_i].x;
      (*output_xyzi)[point_i].y = (*source)[point_i].y;
      (*output_xyzi)[point_i].z = (*source)[point_i].z;

      //(*output_xyzi)[point_i].intensity = boost::math::constants::pi<double>()*(4/3)*pow(squaredEuclideanDistance ((*source)[point_i], (*source)[nn_indices[1]]),3);
      (*output_xyzi)[point_i].intensity = 3/(4*boost::math::constants::pi<double>()*pow(squaredEuclideanDistance ((*source)[point_i], (*source)[nn_indices[1]]),3));
    }
     */
    this->result = std::make_pair(output_xyzi,0.0);
    std::cout << "Density calculated" << std::endl;

    emit DensityFinished();
    emit finished();
}



void Worker::simplificate_method_1() {
    target.reset (new pcl::PointCloud<pcl::PointXYZ>);
    PyObject *pName, *pModule, *pFunc, *pArgs;
    Py_Initialize();
    PyRun_SimpleString("import sys");
    std::string command = "sys.path.append(\"" +  path_dir.toStdString() + "\")";
    PyRun_SimpleString(command.c_str());
    if(this->percent){
        pName = PyUnicode_DecodeFSDefault("first_method");
        pModule = PyImport_Import(pName);
        pFunc = PyObject_GetAttrString(pModule, "simplify");
        pArgs = PyTuple_New(2);
        PyTuple_SetItem(pArgs, 0, PyUnicode_DecodeFSDefault(path_file.toStdString().c_str()));
        PyTuple_SetItem(pArgs, 1, Py_BuildValue("i", this->percent));
    }
    else{
        pName = PyUnicode_DecodeFSDefault("second_method");
        pModule = PyImport_Import(pName);
        pFunc = PyObject_GetAttrString(pModule, "gridding");
        pArgs = PyTuple_New(1);
        PyTuple_SetItem(pArgs, 0, PyUnicode_DecodeFSDefault(path_file.toStdString().c_str()));
    }


    PyObject * pResult = PyObject_CallObject(pFunc, pArgs);
    for(int i = 0; i < PyList_Size(pResult) - 1; ++i){
        PyObject * elem = PyList_GetItem(pResult,i);
        pcl::PointXYZ point_;
        for(int j = 0; j < PyList_Size(elem); ++j){
           if(j == 0){
                point_.x = PyFloat_AsDouble(PyList_GetItem(elem,j));
            }
           else if(j == 1)
                point_.y = PyFloat_AsDouble(PyList_GetItem(elem,j));
           else
                point_.z = PyFloat_AsDouble(PyList_GetItem(elem,j));
        }
        target->push_back(point_);
    }

    PyObject * elem = PyList_GetItem(pResult,PyList_Size(pResult)-1);
    time = PyFloat_AsDouble(elem);



   emit simplificationFinished();
   // emit simplificationFinished(path_file, target_p);
    emit finished();
}
