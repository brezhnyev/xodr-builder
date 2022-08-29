cmake_minimum_required(VERSION 3.10)

project(XodrBuilder)
add_compile_options(-fPIC)

add_subdirectory(thirdparty/odrparser)

include_directories(thirdparty/odrparser/include)
include_directories(thirdparty/odrparser/src)

add_library(${PROJECT_NAME} SHARED ${CMAKE_SOURCE_DIR}/XodrBuilder.cpp)
target_link_libraries(${PROJECT_NAME} odrparser)

# We use the customer version of tinyxml2.cpp
# This version will replace the library version thereof
# Reason: the legacy sscanf and atof functions are faulty on Del Precision series
add_custom_command(
  TARGET ${PROJECT_NAME} PRE_BUILD
  COMMAND cp ${CMAKE_SOURCE_DIR}/tinyxml2.cpp ${CMAKE_SOURCE_DIR}/thirdparty/odrparser/lib/tinyxml2)

set(HEADER_INCLUDE_FILES XodrBuilder.h)

add_subdirectory(${CMAKE_SOURCE_DIR}/example)

# install commands
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library)
install(FILES ${HEADER_INCLUDE_FILES}
        DESTINATION include/${PROJECT_NAME})