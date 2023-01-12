cmake_minimum_required(VERSION 3.10)

project(viz_example)
add_compile_options(-fPIC)

include_directories("/usr/include/GL")

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} XodrBuilder -lglut -lGLU -lOpenGL)

add_custom_command(
  TARGET ${PROJECT_NAME} PRE_BUILD
  COMMAND cp ${CMAKE_CURRENT_LIST_DIR}/Town01.zip ${CMAKE_CURRENT_BINARY_DIR})
