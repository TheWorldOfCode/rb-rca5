cmake_minimum_required(VERSION 3.0)
project(robot_control)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Find the package inforamtion for opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11

# Find the package information for gazobo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_PROTO_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Find the library fuzzylite
find_library(FuzzyLib fuzzylite)

SET(GCC_COVERAGE_COMPILE_FLAGS "-g")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}") 
  
# Includes all headers in  ../includes
include_directories(includes)

# Find all sources files
file(GLOB SOURCES src/*.cpp)

# roadmap Test
add_executable(roadmapGeneratorTest ${SOURCES} mains/mainRoadmapGeneratorTest.cpp)
target_link_libraries(roadmapGeneratorTest ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLib})

add_executable(roadmapNavigationTest ${SOURCES} mains/mainRoadmapNavigationTest.cpp)
target_link_libraries(roadmapNavigationTest ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLib})

# Particlefilter test
add_executable(particlefiltertest ${SOURCES} mains/mainParticleFilter.cpp)
target_link_libraries(particlefiltertest ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLib})

add_executable(particlefilterlookuptest ${SOURCES} mains/mainParticleFilterLookupTableTest.cpp)
target_link_libraries(particlefilterlookuptest ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLib})


# FuzzyControl
add_executable(FuzzyControlFreeMarbleCollecting ${SOURCES} mains/mainFreeMarbleCollecting.cpp)
target_link_libraries(FuzzyControlFreeMarbleCollecting ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLib})

add_executable(FuzzyControlFreeRoam ${SOURCES} mains/mainFreeroam.cpp)
target_link_libraries(FuzzyControlFreeRoam ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLib})

# FuzzyControl and Navigation
add_executable(FuzzyControlNavigationPathAndMarbles ${SOURCES} mains/mainPathAndMarbles.cpp)
target_link_libraries(FuzzyControlNavigationPathAndMarbles ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLib})

add_executable(FuzzyControlNavigationPathPlanning ${SOURCES} mains/mainPathPlanning.cpp)
target_link_libraries(FuzzyControlNavigationPathPlanning ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLib})

add_executable(FuzzyControlNavigationPathPlanningImproved ${SOURCES} mains/mainPathPlanningImproved.cpp)
target_link_libraries(FuzzyControlNavigationPathPlanningImproved ${OpenCV_LIBS} ${GAZEBO_PROTO_LIBRARIES} ${GAZEBO_LIBRARIES} ${FuzzyLib})
