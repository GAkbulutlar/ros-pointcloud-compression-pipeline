cmake_minimum_required(VERSION 3.20)
project(BagPointCloudPipelineProject VERSION 0.1.0 LANGUAGES C CXX)

if(WIN32 AND NOT PCL_DIR AND DEFINED CMAKE_TOOLCHAIN_FILE)
    get_filename_component(_vcpkg_root "${CMAKE_TOOLCHAIN_FILE}" DIRECTORY)
    get_filename_component(_vcpkg_root "${_vcpkg_root}" DIRECTORY)
    get_filename_component(_vcpkg_root "${_vcpkg_root}" DIRECTORY)
    set(_pcl_config_dir "${_vcpkg_root}/installed/x64-windows/share/pcl")
    if(EXISTS "${_pcl_config_dir}/PCLConfig.cmake")
        set(PCL_DIR "${_pcl_config_dir}" CACHE PATH "Path to PCL package config" FORCE)
    endif()
endif()

find_package(PCL CONFIG REQUIRED COMPONENTS common octree kdtree filters visualization)
find_package(BZip2 REQUIRED)
find_package(lz4 CONFIG REQUIRED)

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
target_link_libraries(BagPointCloudPipeline PRIVATE ${PCL_LIBRARIES} BZip2::BZip2 LZ4::lz4)
