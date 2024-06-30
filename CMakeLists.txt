cmake_minimum_required(VERSION 3.10)
project(kdtree)

set(CMAKE_CXX_STANDARD 11)

# Eigenのインクルードディレクトリを設定
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Eigen)

# インクルードディレクトリを設定
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 実行ファイルを追加
add_executable(kdtree main.cpp
        NearestNeighborSearch.h
        SearchResultHandler.h
        ThreeDimensionalPointHandler.h)
