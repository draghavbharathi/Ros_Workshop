cmake_minimum_required(VERSION 3.8)
project(gz_link_attacher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# ---- Gz Sim System Plugin ----
add_library(GzLinkAttacherSystem SHARED
  src/GzLinkAttacherSystem.cc
)

target_include_directories(GzLinkAttacherSystem PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(GzLinkAttacherSystem
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-common5::gz-common5
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS GzLinkAttacherSystem
  LIBRARY DESTINATION lib
)

# ---- Python scripts ----
install(PROGRAMS
  scripts/link_attacher_service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
