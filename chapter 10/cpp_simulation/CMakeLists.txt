cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_BUILD_TYPE "Debug")
project(simulator)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR} 
    ${PYTHON_INCLUDE_DIRS}
    include/controller_lib 
    include/filter_lib 
    include/plot_lib
    )

add_executable(filter_average src/filter_average.cpp)
target_link_libraries(filter_average ${PYTHON_LIBRARIES})

add_executable(filter_kalman_p src/filter_kalman_p.cpp)
target_link_libraries(filter_kalman_p ${PYTHON_LIBRARIES})  

add_executable(filter_kalman_pa src/filter_kalman_pa.cpp)
target_link_libraries(filter_kalman_pa ${PYTHON_LIBRARIES})

add_executable(filter_2D_avg src/filter_2D_avg.cpp)
target_link_libraries(filter_2D_avg ${PYTHON_LIBRARIES})

add_executable(filter_2D_avg_med src/filter_2D_avg_med.cpp)
target_link_libraries(filter_2D_avg_med ${PYTHON_LIBRARIES})

add_executable(filter_2D_first_order src/filter_2D_first_order.cpp)
target_link_libraries(filter_2D_first_order ${PYTHON_LIBRARIES})

add_executable(filter_2D_limit src/filter_2D_limit.cpp)
target_link_libraries(filter_2D_limit ${PYTHON_LIBRARIES})

add_executable(filter_2D_median src/filter_2D_median.cpp)
target_link_libraries(filter_2D_median ${PYTHON_LIBRARIES})

add_executable(filter_extend_kalman src/filter_extend_kalman.cpp)
target_link_libraries(filter_extend_kalman ${PYTHON_LIBRARIES})

add_executable(filter_unscented_kalman src/filter_unscented_kalman)
target_link_libraries(filter_unscented_kalman ${PYTHON_LIBRARIES})

add_executable(filter_particle_filter src/filter_particle_filter)
target_link_libraries(filter_particle_filter ${PYTHON_LIBRARIES})
