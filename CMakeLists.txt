cmake_minimum_required(VERSION 3.4...3.18)
project(WiringPi)

find_package(pybind11) 
set (CMAKE_CXX_STANDARD 17)
file (GLOB SOURCES "src/[^_]*.cpp")  
file (GLOB CONFLICT_SOURCES "src/conflict/[^_]*.cpp")

# set_property(TARGET ${LIBRARY} PROPERTY INTERPROCEDURAL_OPTIMIZATION True)
pybind11_add_module(${PROJECT_NAME} ${SOURCES} ${CONFLICT_SOURCES}) 

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.

target_compile_definitions(${PROJECT_NAME}
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

target_link_libraries(${PROJECT_NAME} PRIVATE -lwiringPi -lwiringPiDev) 

    
