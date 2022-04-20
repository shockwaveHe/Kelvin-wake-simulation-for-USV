cmake_minimum_required(VERSION 2.8.3)
include(CheckLanguage)
# check_language(CUDA)
project(KW_example  LANGUAGES CXX CUDA)
# find_package(CUDA REQUIRED)
# include_directories(
#   ${CUDA_INCLUDE_DIRS}
# )
add_executable(KW_example
    influencedpoint.cpp
    influencingpoint.cpp
    shipcourse.cpp
    surfacegrid.cpp
    main.cpp)
    # integration.cu

    add_executable(calculated_example
    influencedpoint.cpp
    influencingpoint.cpp
    shipcourse.cpp
    surfacegrid.cpp
    calculated.cpp)
# target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES})
