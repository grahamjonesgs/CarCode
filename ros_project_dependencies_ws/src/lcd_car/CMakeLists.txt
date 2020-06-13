cmake_minimum_required(VERSION 2.8.3)
project(lcd_car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(pigpoi_include "/usr/local/include")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lcd_car
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

add_executable(lcd_car src/lcd_car.cpp src/fontx.cpp src/wili9340.cpp)

target_link_libraries(lcd_car
   ${catkin_LIBRARIES} -L -lrt -lstdc++ -lbcm2835 -lwiringPi
-pthread
)
