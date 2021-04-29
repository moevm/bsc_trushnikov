cmake_minimum_required(VERSION 3.5)

project(prototype LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(PythonLibs REQUIRED)
find_package(PythonInterp REQUIRED)

IF (PYTHONLIBS_FOUND )

ENDIF()

SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "D:/Program Files/boost_1_75_0")
#SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "C:\\Program Files\\PCL 1.11.1\\3rdParty\\Boost\\lib")

SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "C:\\Program Files (x86)\\VTK\\include\\vtk-9.0")

find_package(Boost 1.75 REQUIRED)

find_package(PCL 1.2 REQUIRED)
find_package(VTK 8.2.0 REQUIRED)

find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)



set(PROJECT_SOURCES
        densityform.cpp
        densityform.h
        densityform.ui
        percentform.cpp
        percentform.h
        percentform.ui
        rmse.cpp
        rmse.h
        rmse.ui
        worker.cpp
        worker.h
        treeitem.cpp
        treeitem.h
        helpfunctions.cpp
        helpfunctions.h
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

#qt_add_executable(prototype${PROJECT_SOURCES})

add_executable(prototype ${PROJECT_SOURCES})



set( CMAKE_CXX_FLAGS "-lpython3.7")

include_directories(${PYTHON_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${VTK_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
ADD_DEFINITIONS( "-DHAS_BOOST" )

#target_link_libraries(prototype ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
target_link_directories(prototype PRIVATE "D:/Program Files/boost_1_75_0/lib64-msvc-14.2" "C:/Users/Anderson/AppData/Local/Programs/Python/Python37/libs")
target_link_libraries(prototype PRIVATE Qt${QT_VERSION_MAJOR}::Widgets  ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})
