cmake_minimum_required(VERSION 2.6)
project(apriltags)

#add_definitions(-pg) #"-fopenmp)

# pull in the pods macros. See cmake/pods.cmake for documentation
include(cmake/pods.cmake)

file(GLOB SOURCE_FILES "src/*.cc")

add_subdirectory(example)
