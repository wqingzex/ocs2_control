cmake_minimum_required(VERSION 3.14)
project(fa_w2_body_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY meshes xacro
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()
