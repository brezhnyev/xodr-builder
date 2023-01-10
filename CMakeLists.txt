cmake_minimum_required(VERSION 3.10)

project(XodrBuilder VERSION 0.0.1)
add_compile_options(-fPIC)

add_subdirectory(thirdparty/odrparser)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/odrparser/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/odrparser/src)

add_library(${PROJECT_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/XodrBuilder.cpp)
target_link_libraries(${PROJECT_NAME} odrparser)

# We use the customer version of tinyxml2.cpp
# This version will replace the library version thereof
# Reason: the legacy sscanf and atof functions are faulty on Del Precision series
# add_custom_command(
#   TARGET ${PROJECT_NAME} PRE_BUILD
#   COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/replace/tinyxml2.cpp ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/odrparser/lib/tinyxml2)

# The tinyxml should be built first, and then odrparser, CMakListsOdrParser has the correct order or add_subdirectory calls
add_custom_command(
  TARGET ${PROJECT_NAME} PRE_BUILD
  COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/replace/CMakeListsOdrParser.txt ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/odrparser/CMakeLists.txt)

set(HEADER_INCLUDE_FILES XodrBuilder.h)

option(BUILD_EXAMPLES "Build cpp example(s)" OFF)
if (BUILD_EXAMPLES)
  add_subdirectory(example)
endif()

# install commands
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library)
install(FILES ${HEADER_INCLUDE_FILES}
        DESTINATION include/${PROJECT_NAME})