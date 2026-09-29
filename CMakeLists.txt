cmake_minimum_required(VERSION 3.8)
project(sheep_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

# 1. 巡检机器人控制节点 (turtle1)
add_executable(patrol_node src/patrol_node.cpp)
ament_target_dependencies(patrol_node
  rclcpp
  geometry_msgs
  turtlesim
  std_msgs
  std_srvs
)

# 2. 补能系统控制节点 (turtle2)
add_executable(charger_node src/charger_node.cpp)
ament_target_dependencies(charger_node
  rclcpp
  geometry_msgs
  turtlesim
  std_msgs
  std_srvs
)

# 安装二进制可执行文件
install(TARGETS
  patrol_node
  charger_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装启动脚本
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
