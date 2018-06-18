##############################
PROJECT( ipm )
##############################
cmake_minimum_required (VERSION 2.6)

# Packages
find_package( OpenCV REQUIRED )


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# include ROS
include_directories(/opt/ros/kinetic/include)
link_directories(/opt/ros/kinetic/lib)
set(ROS_LIBRARIES roscpp rosconsole roscpp_serialization 
                  rostime tf boost_system)


# Includes
include_directories(	
	${OPENCV_INCLUDE_DIR}
	${PROJECT_BINARY_DIR}
)

set(SOURCE_FILES 
	src/main.cpp
	src/IPM.cpp
	src/IPM.h
)


# Add executable and target link libraries
ADD_EXECUTABLE( ipm ${SOURCE_FILES})
TARGET_LINK_LIBRARIES( ipm ${OpenCV_LIBS} ${PCL_LIBRARIES} ${ROS_LIBRARIES})

add_executable (icp src/icp.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})
