# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.1...3.15)

set(CMAKE_BUILD_TYPE Release)

project(PathPlanner)


# CGAL
find_package(CGAL)

if (NOT CGAL_FOUND)

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()


# Boost and its components
find_package(Boost REQUIRED)

if (NOT Boost_FOUND)

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

# Qt5 and its components
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)

if(CGAL_Qt5_FOUND)

  add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)

endif()

# set OpenCV_DIR variable to the path of the cmake files
set(OpenCV_DIR /usr/local/Cellar/opencv/4.5.3_2/lib/cmake/opencv4)

# OpenCV and its components
find_package(OpenCV REQUIRED)

# set executable path
set(EXECUTABLE_OUTPUT_PATH "bin")

# set library output path
set(LIBRARY_OUTPUT_PATH "lib")

# set sources
file(GLOB SOURCES 
  "src/*.cpp"
)

# set header files
file(GLOB HEADER_FILES 
  "headers/*.h"
)

# add executable
add_executable(path_planner ${SOURCES} ${HEADER_FILES})

# include directories
target_include_directories(path_planner PUBLIC ${OpenCV_INCLUDE_DIRS})

# link libraries to path_planner
target_link_libraries(path_planner ${OpenCV_LIBS} CGAL::CGAL CGAL::CGAL_Qt5)