cmake_minimum_required(VERSION 3.5)
project(package_name)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)    ]
find_package(rclpy REQUIRED)

# Install scripts folder
install(PROGRAMS
DESTINATION lib/${PROJECT_NAME} )

install(FILES
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

ament_package()
