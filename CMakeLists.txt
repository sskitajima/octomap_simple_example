cmake_minimum_required(VERSION 3.0)
project(octomap_simple_example)

find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

message("OpenCV include dir ${OpenCV_INCLUDE_DIRS}")
message("Eigen include dir ${EIGEN3_INCLUDE_DIRS}")
message("octomap include dir ${OCTOMAP_INCLUDE_DIRS}")

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/recover_points_from_depth.cpp
)


target_link_libraries(${PROJECT_NAME}
    ${OCTOMAP_LIBRARIES}
    ${OpenCV_LIBRARIES}
)