PROJECT(codeins)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native ")

INCLUDE_DIRECTORIES(../src ../src/PNP)


SET(Eigen3_INCLUDE_DIRS "/usr/local/include/eigen3")#这里指定eigen的包含路径，不然这个傻子找不到
INCLUDE_DIRECTORIES(${Eigen3_INCLUDE_DIRS})
FIND_PACKAGE(OpenCV 3 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${Ceres_INCLUDE_DIRS})

MESSAGE(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(codeins 
        ../src/main.cpp 
        ../src/Marker.cpp 
        ../src/QRCodeSolve.cpp
        # ../src/PNP/epnp.cpp
        # ../src/PNP/p3p.cpp
        # ../src/PNP/polynom_solver.cpp
        # ../src/PNP/solvepnp.cpp
        )
TARGET_LINK_LIBRARIES(codeins ${OpenCV_LIBS} ${CERES_LIBRARIES})
