cmake_minimum_required(VERSION 2.8.3)
project(imu_normalise)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(pigpoi_include "/usr/local/include")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_normalise
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(

  ${catkin_INCLUDE_DIRS}
  ${pigpoi_include}
)

LINK_DIRECTORIES("/usr/local/lib")

add_executable(imu_normalise src/imu_normalise.cpp)

target_link_libraries(imu_normalise
   ${catkin_LIBRARIES} -lpigpiod_if2 -lrt -lstdc++
-pthread
)
