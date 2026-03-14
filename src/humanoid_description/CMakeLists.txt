cmake_minimum_required(VERSION 3.8)
project(humanoid_description)

find_package(ament_cmake REQUIRED)

# Install URDF/Xacro files
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

# Install mesh files
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install RViz config
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

# Install config files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
