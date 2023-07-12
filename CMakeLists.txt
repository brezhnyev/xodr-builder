cmake_minimum_required(VERSION 3.10)

set (CMAKE_CXX_STANDARD 14)

project(XodrBuilder VERSION 0.0.1)
add_compile_options(-fPIC)

add_library(${PROJECT_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/XodrBuilder.cpp)
target_link_libraries(${PROJECT_NAME} odrparser)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/odrparser/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/odrparser/src)
target_include_directories(${PROJECT_NAME} PUBLIC .)

# We use the customer version of tinyxml2.cpp
# This version will replace the library version thereof
# Reason: the legacy sscanf and atof functions are faulty on Del Precision series
execute_process(COMMAND git checkout 7.0.0 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/odrparser/lib/tinyxml2)
execute_process(COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/replace/tinyxml2.cpp ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/odrparser/lib/tinyxml2)
# The tinyxml should be built first, and then odrparser, CMakListsOdrParser has the correct order or add_subdirectory calls
execute_process(COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/replace/CMakeListsOdrParser.txt ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/odrparser/CMakeLists.txt)

add_subdirectory(thirdparty/odrparser)

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