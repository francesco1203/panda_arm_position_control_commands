cmake_minimum_required(VERSION 3.8)
project(pacchetto_nodi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pacchetto_interfacce REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)                    
find_package(moveit_core REQUIRED)               
find_package(moveit_ros_planning REQUIRED)   

# Aggiungi tra gli altri find_package
find_package(yaml-cpp REQUIRED)



# include cartella include dove ci sono header file, in particolare common.hpp con costanti e alias comuni a più nodi
include_directories(include)


# nodo joint_traj_generator
add_executable(joint_traj_generator src/joint_traj_generator.cpp)
ament_target_dependencies(joint_traj_generator
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs           # aggiunto per aver incluso message_alias.hpp
  pacchetto_interfacce
)

# nodo cartesian_traj_generator
add_executable(cartesian_traj_generator src/cartesian_traj_generator.cpp)
ament_target_dependencies(cartesian_traj_generator
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs           
  pacchetto_interfacce                  
  moveit_core               
  moveit_ros_planning       
)

# nodo clik_node
add_executable(clik_node src/clik_node.cpp)
ament_target_dependencies(clik_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pacchetto_interfacce
  Eigen3                    
  moveit_core               
  moveit_ros_planning       
)

# nodo task_node
add_executable(task_node src/task_node.cpp)
ament_target_dependencies(task_node
  rclcpp
  rclcpp_action
  sensor_msgs        
  geometry_msgs      
  Eigen3                    
  # moveit_core               
  # moveit_ros_planning 
  pacchetto_interfacce
)

target_link_libraries(task_node yaml-cpp)

######## INSTALL nodes #######################
install(TARGETS
  joint_traj_generator
  cartesian_traj_generator 
  clik_node
  task_node
  DESTINATION lib/${PROJECT_NAME}
)

########## INSTALL the launch files ############
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
################################################

########## INSTALL the config files ############
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
