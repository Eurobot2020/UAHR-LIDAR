cmake_minimum_required(VERSION 2.8.3)
project(uahr_lidar)

set(RPLIDAR_SDK_PATH "./include/sdk/")
set(UAHR_PATH "./include/UAHR/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

FILE(GLOB UAHR_SRC 
  "${UAHR_PATH}/src/*.cpp"
)

#find_package(catkin REQUIRED COMPONENTS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  std_msgs
  geometry_msgs
  rostest
  uah_msgs
  )

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${UAHR_PATH}/src
  ${UAHR_PATH}/include
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

add_executable(uahrlidarNode src/uahrlidar.cpp ${RPLIDAR_SDK_SRC} ${UAHR_SRC})
target_compile_options (uahrlidarNode PRIVATE -Wall -pedantic)
target_link_libraries(uahrlidarNode ${catkin_LIBRARIES})

add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})

install(TARGETS uahrlidarNode  rplidarNode rplidarNodeClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

catkin_add_gtest(lidarlibtest test/LidarLib.cpp ${UAHR_SRC})
target_link_libraries(lidarlibtest ${catkin_LIBRARIES})

