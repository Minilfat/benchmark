cmake_minimum_required(VERSION 3.8)
project(benchmark)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp bdm/point.h bdm/param.h unibn/octree.h bdm/bound.h bdm/spatial_tree_node.h bdm/octree_node.h)
add_executable(benchmark ${SOURCE_FILES})