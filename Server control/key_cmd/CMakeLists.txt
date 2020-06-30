cmake_minimum_required(VERSION 2.8.3)
project(key_cmd)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(pigpoi_include "/usr/local/include")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES key_cmd
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

add_executable(key_cmd src/key_cmd.cpp)

target_link_libraries(key_cmd
   ${catkin_LIBRARIES} -lrt -lstdc++
-pthread
)
