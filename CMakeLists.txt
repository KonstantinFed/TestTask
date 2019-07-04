cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(test_task)

find_package(OpenCV REQUIRED)

set(HEADERS
    src/rgbd_viewer.h
)
set(SOURCES 
    src/console.cpp
    src/rgbd_viewer.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} LINK_PRIVATE ${OpenCV_LIBS})