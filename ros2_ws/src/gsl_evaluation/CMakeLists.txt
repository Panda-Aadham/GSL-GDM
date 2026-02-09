cmake_minimum_required(VERSION 3.5)
project(vgr_pmfs_house01_env)

find_package(ament_cmake REQUIRED)

install(DIRECTORY .
DESTINATION share/${PROJECT_NAME}
)

ament_package()
