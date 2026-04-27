cmake_minimum_required(VERSION 3.20)
project(BagPointCloudPipelineProject VERSION 0.1.0 LANGUAGES C CXX)

find_package(PCL CONFIG REQUIRED COMPONENTS common octree kdtree visualization)
find_package(BZip2 REQUIRED)

if(NOT TARGET pcl_visualization)
    message(FATAL_ERROR "PCL visualization component is missing. Ensure vcpkg installs pcl[visualization].")
endif()

add_executable(BagPointCloudPipeline
    src/bag_pipeline_main.cpp
    src/BagPointCloudPipeline.cpp
)

target_include_directories(BagPointCloudPipeline PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(BagPointCloudPipeline PRIVATE ${PCL_DEFINITIONS})
set_property(TARGET BagPointCloudPipeline PROPERTY CXX_STANDARD 17)
target_link_libraries(BagPointCloudPipeline PRIVATE ${PCL_LIBRARIES} BZip2::BZip2)
