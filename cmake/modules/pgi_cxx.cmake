set(CMAKE_CXX_FLAGS_DEBUG   
    "${CMAKE_CXX_FLAGS_DEBUG}  -D__DEBUG -Mpreprocess -w -Mextend")
set(CMAKE_CXX_FLAGS_DEBUG   
    "${CMAKE_CXX_FLAGS_DEBUG}  -O0 -Mbounds -Mchkptr -Mchkstk")
set(CMAKE_CXX_FLAGS_RELEASE 
    "${CMAKE_CXX_FLAGS_RELEASE} -Mpreprocess -O3 -Mipa=fast -Minfo=opt")
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mp=numa")
else (OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Mnoopenmp")
endif (OPENMP_FOUND)
