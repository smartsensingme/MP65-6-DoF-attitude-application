cmake_minimum_required(VERSION 3.16)

# Use the checked-out components, including their local transitive dependencies.
# Set before project.cmake so Component Manager does not fetch duplicate copies.
set(ENV{IDF_COMPONENT_MANAGER} "0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mp65_attitude)
