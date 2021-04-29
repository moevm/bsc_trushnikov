#include "helpfunctions.h"
#undef B0
#include <QApplication>
#include <pcl/PCLPointCloud2.h>
#include <utility>

std::vector<double> getCameraPos(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    std::vector<double> position_cam;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud,centroid);
    Eigen::Vector3d orientationVector(0,1,0);
    orientationVector.normalize();
    pcl::PointXYZ ptMin, ptMax;
    pcl::getMinMax3D(*cloud, ptMin, ptMax);
    Eigen::Vector3d diff(ptMax.x - ptMin.x, ptMax.y - ptMin.y, ptMax.z - ptMin.z);
    Eigen::Vector3d orth1 = orientationVector.unitOrthogonal();
    Eigen::Vector3d orth2 = orientationVector.cross(orth1).normalized();
    double camHeight = std::max(double(ptMax.x), std::max(abs(orth1.dot(diff)), abs(orth2.dot(diff))));
    Eigen::Vector3d cameraPosition;
    for (int dim = 0; dim < 3; ++dim)
        cameraPosition(dim) = centroid[dim] + camHeight * orientationVector[dim];

    position_cam.push_back(cameraPosition.x());
    position_cam.push_back(cameraPosition.y());
    position_cam.push_back(cameraPosition.z());

    return position_cam;
}

QVector<pcl::PointXYZ> simplify_method_1(std::string &&path_dir, std::string &&path_file){
    QVector<pcl::PointXYZ> target_p;
    PyObject *pName, *pModule, *pFunc, *pArgs;
    Py_Initialize();
    PyRun_SimpleString("import sys");
    std::string command = "sys.path.append(\"" +  path_dir + "\")";
    PyRun_SimpleString(command.c_str());
    pName = PyUnicode_DecodeFSDefault("first_method");
    pModule = PyImport_Import(pName);
    pFunc = PyObject_GetAttrString(pModule, "simplify");
    pArgs = PyTuple_New(1);
    PyTuple_SetItem(pArgs, 0, PyUnicode_DecodeFSDefault(path_file.c_str()));
    PyObject * pResult = PyObject_CallObject(pFunc, pArgs);
    for(int i = 0; i < PyList_Size(pResult) - 1; ++i){
        PyObject * elem = PyList_GetItem(pResult,i);
        for(int j = 0; j < PyList_Size(elem); ++j){
           if(j == 0){
                target_p.push_back(pcl::PointXYZ());
                target_p[i].x = PyFloat_AsDouble(PyList_GetItem(elem,j));
            }
           else if(j == 1)
                target_p[i].y = PyFloat_AsDouble(PyList_GetItem(elem,j));
           else
                target_p[i].z = PyFloat_AsDouble(PyList_GetItem(elem,j));
        }
    }
   // if(run_python)
   //     Py_Finalize();

    return target_p;
}

double square(double value) {
    return value * value;
}

double squaredEuclideanDistance(pcl::PointXYZ &first, pcl::PointXYZ &second) {
    return std::sqrt(square(first.x - second.x) + square(first.y - second.y) + square(first.z - second.z));
}


