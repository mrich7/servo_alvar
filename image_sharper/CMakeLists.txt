# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(image_sharper)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs image_transport cv_bridge)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package

catkin_package(
    DEPENDS OpenCV Boost
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport cv_bridge
    INCLUDE_DIRS 
    LIBRARIES 
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(image_sharper src/image_sharper.cpp)


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)



