cmake_minimum_required(VERSION 2.8.3)
project(pi_car_s_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

set(pigpoi_include "/usr/local/include")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pi_car_s_controller
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

add_executable(pi_car_s_controller src/pi_car_s_controller.cpp)

target_link_libraries(pi_car_s_controller
   ${catkin_LIBRARIES} -lpigpiod_if2 -lrt -lstdc++
-pthread
)
