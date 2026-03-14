cmake_minimum_required(VERSION 3.8)
project(humanoid_gazebo)

find_package(ament_cmake REQUIRED)

# Install world files
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Install Gazebo model files
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()
