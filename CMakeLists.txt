cmake_minimum_required(VERSION 2.8)
project(helloworld)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## find a version of ceres compiled with suitesparse & eigensparse support
#find_package(Ceres REQUIRED COMPONENTS SuiteSparse EigenSparse)

##specify ceres version
#find_package(Ceres 1.2.3 REQUIRED)


### if you want to specify the path of ceres you need
#find_package(Ceres REQUIRED PATHS "some/where/local")

## if you don't want to install ceres into your computer you should
## consider exporting Ceres using the EXPORT_BUILD_DIR

include_directories(${CERES_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    )

#helloworld
add_executable(helloworld ceres_lession1.cpp)
target_link_libraries(helloworld ${CERES_LIBRARIES})

add_executable(curvefitting curve-fitting.cpp)
target_link_libraries(curvefitting ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(pose_graph_2d 
slam_2d/pose_graph_2d.cpp
slam_2d/normalize_angle.h
slam_2d/types.h
slam_2d/angle_local_parameterization.h
slam_2d/pose_graph_2d_error_term.h
common/read_g2o.h)
target_link_libraries(pose_graph_2d ${CERES_LIBRARIES} ${EIGEN3_LIBS})

add_executable(pose_graph_3d 
slam_3d/types.h
slam_3d/pose_graph_3d_error_term.h
slam_3d/pose_graph_3d.cpp
common/read_g2o.h
)
target_link_libraries(pose_graph_3d ${CERES_LIBRARIES} ${EIGEN3_LIBS})


#add_subdirectory(slam_2d)
#add_subdirectory(slam_3d)