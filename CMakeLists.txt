cmake_minimum_required(VERSION 3.0.2)
project(easy_pid_controller_pkg)

## 编译选项
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 寻找 ROS 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## 寻找 Eigen3
find_package(Eigen3 REQUIRED)

## catkin 包配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS roscpp
)

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## 创建核心库
add_library(${PROJECT_NAME}_core
  src/easy_pid_controller.cpp
)

## 链接库
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${Eigen3_LIBS}
)

## 安装头文件（供其他包引用）
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## 安装库文件
install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
